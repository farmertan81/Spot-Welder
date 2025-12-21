#!/usr/bin/env python3

# Weld cooldown protection
WELD_COOLDOWN_MS = 3000  # 3 seconds minimum between welds
last_weld_time = 0
# Circuit resistance
TOTAL_RESISTANCE_OHM = 0.00296  # 2.96 mΩ total circuit resistance


"""
Spot Welder Control Server
Flask + SocketIO + ESP32 + ADS1256 + Weld Capture
"""

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
from flask_sock import Sock
import threading
from collections import deque
import time
import json
import os
from datetime import datetime
import math

# Import our drivers
from esp_link import ESP32Link
from ads1256_driver import ADS1256

app = Flask(__name__)
app.config['SECRET_KEY'] = 'weldctl_secret_2025'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
sock = Sock(app)

# Directories
WELD_HISTORY_DIR = "../weld_history"
SETTINGS_FILE = "settings.json"
PRESETS_FILE = "presets.json"
os.makedirs(WELD_HISTORY_DIR, exist_ok=True)

# Global state
esp_status = {
    "vpack": 0.0,
    "i": 0.0,
    "pulse_ms": 50,
    "state": "IDLE",
    "enabled": False,
    "cooldown_ms": 0
}

cells_status = {
    "V1": 0.0, "V2": 0.0, "V3": 0.0,
    "C1": 0.0, "C2": 0.0, "C3": 0.0
}

temperature = None
pedal_active = False
weld_counter = 0
# Pre-trigger buffer (not used now but kept for compatibility)
PRE_TRIGGER_MS = 10.0  # Capture 10 ms before weld (legacy)
SAMPLE_RATE_HZ = 1000  # 1kHz sampling (legacy)
PRE_BUFFER_SIZE = int(PRE_TRIGGER_MS * SAMPLE_RATE_HZ / 1000)
pre_trigger_buffer = deque(maxlen=PRE_BUFFER_SIZE)
continuous_sampling = True
esp_connected = False

# Weld capture
MAX_WELD_HISTORY = 15
current_weld_data = []  # list of {"t": usec, "v": volts, "i": amps}
is_capturing = False
weld_start_time = None

# Logs buffer
log_buffer = deque(maxlen=500)

# Thread locks
status_lock = threading.Lock()
weld_lock = threading.Lock()

# Hardware instances
esp_link = None
esp32_ws = None  # Global WebSocket connection to ESP32
adc = None


def log(msg):
    """Add message to log buffer and print"""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log_msg = f"[{timestamp}] {msg}"
    log_buffer.append(log_msg)
    print(log_msg, flush=True)


def on_esp_status(data):
    """Callback when ESP32Link pushes combined STATUS+CELLS payload"""
    global esp_connected, temperature
    with status_lock:
        # Core ESP status
        esp_status.update(data)
        esp_connected = data.get("esp_connected", True)

        # Temperature if present
        if 'temp' in data:
            temperature = data['temp']

        # Cell info coming from ESP32Link._emit_status_update()
        for key in ("V1", "V2", "V3", "C1", "C2", "C3"):
            if key in data:
                cells_status[key] = data[key]


def on_esp_cells(data):
    """Callback when ESP32 sends CELLS"""
    with status_lock:
        cells_status.update(data)


def on_esp_log(msg):
    """Handle ESP32 log messages including weld data (WDATA) and FIRED events."""
    global esp_status, is_capturing, weld_start_time, current_weld_data, pedal_active, weld_counter

    # Always log for debugging
    log(msg)

    # --- WDATA: per-sample weld data from ESP32 ---
    if msg.startswith("WDATA,"):
        try:
            # Format: WDATA,voltage,current,time_us
            parts = msg.split(',')
            if len(parts) >= 4:
                v = float(parts[1])
                i = float(parts[2])
                t_us = int(parts[3])

                with weld_lock:
                    # If we weren't capturing yet, start now on first WDATA
                    if not is_capturing:
                        is_capturing = True
                        weld_start_time = time.time()
                        current_weld_data = []
                        pedal_active = True
                        socketio.emit('pedal_active', {"active": True})
                        log("🔥 Weld capture started on first WDATA")

                        # Baseline point at the first ESP timestamp, i = 0
                        current_weld_data.append({
                            "t": t_us,  # µs, will normalize later
                            "v": v,
                            "i": 0.0
                        })

                    # Store real sample using raw ESP time_us
                    current_weld_data.append({
                        "t": t_us,   # µs
                        "v": v,
                        "i": i
                    })

                log(f"📊 WDATA: t={t_us/1000.0:.2f}ms, v={v:.3f}V, i={i:.1f}A")

        except Exception as e:
            log(f"⚠️ Failed to parse WDATA: {e}")

    # --- WDATA_END: summary line from ESP32 (optional) ---
    elif msg.startswith("WDATA_END,"):
        log(f"ℹ️ WDATA_END summary: {msg}")

    # --- FIRED: weld completed ---
    elif msg.startswith("FIRED,"):
        # Parse weld duration from FIRED,<ms> (optional)
        weld_duration_ms = 0
        try:
            parts = msg.split(',')
            if len(parts) >= 2:
                weld_duration_ms = int(parts[1])
                log(f"Parsed weld duration: {weld_duration_ms}ms")
        except Exception:
            pass

        # Stop capture and save, if we have been capturing
        if is_capturing:
            is_capturing = False
            log("✅ Weld ended - saving data")
            with weld_lock:
                # Append landing point at i = 0 after the last sample
                if isinstance(current_weld_data, list) and current_weld_data:
                    last = current_weld_data[-1]
                    last_t_us = last["t"]
                    last_v = last["v"]
                    current_weld_data.append({
                        "t": last_t_us + 200,  # +0.2 ms in µs
                        "v": last_v,
                        "i": 0.0
                    })

                weld_record = save_weld_history(current_weld_data)

            pedal_active = False
            socketio.emit('pedal_active', {"active": False})
            socketio.emit('weld_complete', {
                "weld_number": weld_counter,
                "energy_joules": weld_record["energy_joules"],
                "peak_current_amps": weld_record["peak_current_amps"],
                "duration_ms": weld_record["duration_ms"]
            })

        # Update state regardless
        with status_lock:
            esp_status["state"] = "IDLE"

    else:
        # Already logged above
        pass


def load_settings():
    """Load settings from JSON"""
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                return json.load(f)
        except Exception:
            pass
    return {
        "mode": 1,
        "d1": 50,
        "gap1": 0,
        "d2": 0,
        "gap2": 0,
        "d3": 0,
        "pulse_ms": 50,
        "weld_counter": 0
    }


def save_settings(data):
    """Save settings to JSON"""
    with open(SETTINGS_FILE, 'w') as f:
        json.dump(data, f, indent=2)
    log(f"Settings saved: mode={data.get('mode')}, d1={data.get('d1')}")


def load_presets():
    """Load presets from JSON"""
    if os.path.exists(PRESETS_FILE):
        try:
            with open(PRESETS_FILE, 'r') as f:
                return json.load(f)
        except Exception:
            pass
    return {
        "P1": {"name": "Preset 1", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0},
        "P2": {"name": "Preset 2", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0},
        "P3": {"name": "Preset 3", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0},
        "P4": {"name": "Preset 4", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0},
        "P5": {"name": "Preset 5", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0}
    }


def save_presets(data):
    """Save presets to JSON"""
    with open(PRESETS_FILE, 'w') as f:
        json.dump(data, f, indent=2)
    log(f"Presets saved")


def calculate_energy(weld_data):
    """Calculate energy delivered in Joules using I²R integration (only during actual weld)"""
    TOTAL_R = 0.00296  # 2.96 mΩ total circuit resistance
    CURRENT_THRESHOLD = 100.0  # Only count energy when current > 100A
    timestamps = weld_data["timestamps"]
    currents = weld_data["current"]

    if len(timestamps) < 2:
        return 0.0

    energy = 0.0
    for i in range(1, len(timestamps)):
        i_avg = (currents[i] + currents[i - 1]) / 2.0  # Amps
        t_avg = (timestamps[i] + timestamps[i - 1]) / 2.0  # seconds

        if abs(i_avg) > CURRENT_THRESHOLD and t_avg > 0:
            dt = abs(timestamps[i] - timestamps[i - 1])  # seconds
            power = (i_avg ** 2) * TOTAL_R              # Watts = I²R
            energy += power * dt                        # Joules

    return energy


def compute_rise_time_ms(timestamps, currents):
    """
    Compute rise time between 10% and 90% of peak current
    using the filtered waveform.
    timestamps: list of seconds (0 -> end)
    currents:   list of amps
    """
    if len(timestamps) < 2:
        return 0.0

    peak = max(currents) if currents else 0.0
    if peak <= 0:
        return 0.0

    threshold10 = 0.10 * peak
    threshold90 = 0.90 * peak
    t10 = None
    t90 = None

    for t, i in zip(timestamps, currents):
        if t10 is None and i >= threshold10:
            t10 = t
        if t90 is None and i >= threshold90:
            t90 = t
            break

    if t10 is None or t90 is None or t90 <= t10:
        return 0.0

    return (t90 - t10) * 1000.0  # seconds -> ms


def save_weld_history(weld_data):
    """Save weld to history, keep last MAX_WELD_HISTORY"""
    global weld_counter
    weld_counter += 1

    # Handle both formats: list of dicts or dict of lists
    if isinstance(weld_data, list):
        # ESP32 format: [{"t": <time_us>, "v": <V>, "i": <A>}, ...]
        raw_t_us = [d["t"] for d in weld_data]

        if raw_t_us:
            t0 = raw_t_us[0]
            # Normalize so first point is t = 0 (seconds)
            timestamps = [(t - t0) / 1_000_000.0 for t in raw_t_us]
        else:
            timestamps = []

        voltages = [d["v"] for d in weld_data]
        currents = [d["i"] for d in weld_data]

    else:
        # Legacy ADC format: {"timestamps": [...], "voltage": [...], "current": [...]}
        timestamps = weld_data.get("timestamps", [])
        voltages = weld_data.get("voltage", [])
        currents = weld_data.get("current", [])

    # Calculate stats from timestamps (seconds), currents (A), voltages (V)
    energy_j = 0.0
    if len(currents) > 1 and len(timestamps) > 1:
        for j in range(len(currents) - 1):
            dt = timestamps[j + 1] - timestamps[j]   # seconds
            i_avg = (currents[j] + currents[j + 1]) / 2.0
            v_avg = (voltages[j] + voltages[j + 1]) / 2.0
            power = v_avg * i_avg                    # Watts = V * A
            energy_j += power * dt                   # Joules

    peak_current = max(currents) if currents else 0.0
    duration_ms = (timestamps[-1] - timestamps[0]) * 1000 if len(timestamps) > 1 else 0.0
    rise_time_ms = compute_rise_time_ms(timestamps, currents)

    # Save weld data
    filename = f"weld_{weld_counter:04d}.json"
    filepath = os.path.join(WELD_HISTORY_DIR, filename)

    weld_record = {
        "weld_number": weld_counter,
        "timestamp": datetime.now().isoformat(),
        "energy_joules": round(energy_j, 2),
        "peak_current_amps": round(peak_current, 1),
        "duration_ms": round(duration_ms, 1),
        "rise_time_ms": round(rise_time_ms, 2),
        "settings": load_settings(),
        "data": weld_data
    }

    with open(filepath, 'w') as f:
        json.dump(weld_record, f, indent=2)

    # Clean up old welds
    weld_files = sorted([f for f in os.listdir(WELD_HISTORY_DIR) if f.startswith("weld_")])
    if len(weld_files) > MAX_WELD_HISTORY:
        for old_file in weld_files[:-MAX_WELD_HISTORY]:
            os.remove(os.path.join(WELD_HISTORY_DIR, old_file))

    # Update counter in settings
    settings = load_settings()
    settings['weld_counter'] = weld_counter
    save_settings(settings)

    log(f"Weld #{weld_counter} saved: {energy_j:.2f}J, {peak_current:.1f}A peak, "
        f"{duration_ms:.1f}ms, rise(10–90%)={rise_time_ms:.2f}ms")

    return weld_record


def get_weld_history_list():
    """Get list of available weld history files"""
    weld_files = sorted([f for f in os.listdir(WELD_HISTORY_DIR) if f.startswith("weld_")], reverse=True)
    return weld_files[:MAX_WELD_HISTORY]


def calculate_cell_balance():
    """Calculate cell balance stats"""
    c1 = cells_status.get("C1", 0.0)
    c2 = cells_status.get("C2", 0.0)
    c3 = cells_status.get("C3", 0.0)

    if c1 == 0 and c2 == 0 and c3 == 0:
        return {"delta": 0.0, "status": "unknown", "color": "gray"}

    delta = max(c1, c2, c3) - min(c1, c2, c3)

    if delta < 0.05:
        status = "balanced"
        color = "green"
    elif delta < 0.10:
        status = "slight_imbalance"
        color = "yellow"
    else:
        status = "imbalanced"
        color = "red"

    return {"delta": round(delta, 3), "status": status, "color": color}


# Routes
@sock.route('/ws')
def websocket_route(ws):
    global esp32_ws, esp_connected, weld_counter
    esp32_ws = ws
    esp_connected = True

    log(f"[WebSocket] ESP32 connected from {request.remote_addr}")

    try:
        while True:
            message = ws.receive()
            if message is None:
                break

            log(f"[WebSocket] RX: {message}")

            # Process ESP32 messages
            if message.startswith("HELLO"):
                log("✅ ESP32 handshake complete")
                ws.send("STATUS")

            elif message.startswith("ACK:"):
                log(f"✅ ESP32 acknowledged: {message[4:]}")

            elif message.startswith("FIRED,"):
                parts = message.split(",")
                if len(parts) >= 2:
                    duration = int(parts[1])
                    weld_counter += 1
                    log(f"🔥 Weld #{weld_counter} fired! Duration: {duration} ms")
                    socketio.emit('weld_fired', {'weld_number': weld_counter, 'duration_ms': duration})

            elif message.startswith("STATUS,"):
                log(f"📊 ESP32 status: {message}")
                # Parse and broadcast to web UI
                parts = message.split(',')
                status_data = {}
                for part in parts[1:]:  # Skip "STATUS"
                    if '=' in part:
                        key, val = part.split('=', 1)
                        try:
                            status_data[key] = float(val)
                        except Exception:
                            status_data[key] = val
                socketio.emit('esp32_status', status_data)
                # Update global esp_status for status_update broadcasts
                with status_lock:
                    esp_status.update(status_data)

            elif message.startswith("ERROR:"):
                log(f"⚠️ ESP32 error: {message[6:]}")

    except Exception as e:
        log(f"[WebSocket] Error: {e}")
    finally:
        esp32_ws = None
        esp_connected = False
        log("[WebSocket] ESP32 disconnected")


@app.route('/')
def index():
    return render_template('control.html')


@app.route('/control')
def control():
    return render_template('control.html')


@app.route('/monitor')
def monitor():
    return render_template('monitor.html')


@app.route('/logs')
def logs():
    return render_template('logs.html')


@app.route('/api/status')
def api_status():
    """Current system status"""
    with status_lock:
        balance = calculate_cell_balance()
        return jsonify({
            **esp_status,
            **cells_status,
            "temperature": temperature,
            "pedal_active": pedal_active,
            "weld_counter": weld_counter,
            "esp_connected": esp_connected,
            "cell_balance": balance
        })


@app.route('/api/get_settings')
def get_settings_route():
    """Get saved settings"""
    settings = load_settings()
    return jsonify({"status": "ok", "settings": settings})


@app.route('/api/save_settings', methods=['POST'])
def save_settings_route():
    """Save settings"""
    global esp_link

    data = request.get_json()
    save_settings(data)

    # Send updated pulse to ESP32 via TCP link (ESP32Link), same as on startup
    d1 = data.get('d1', 50)
    cmd = f"SET_PULSE,{d1}"

    if esp_link:
        try:
            log(f"📤 Sending to ESP32 (TCP): {cmd}")
            esp_link.send_command(cmd)
        except Exception as e:
            log(f"⚠️ Failed to send to ESP32 via TCP: {e}")
    else:
        log("⚠️ ESP32 TCP link not initialized")

    return jsonify({"status": "ok"})


@app.route('/api/get_presets')
def get_presets_route():
    """Get all presets"""
    presets = load_presets()
    return jsonify({"status": "ok", "presets": presets})


@app.route('/api/save_preset', methods=['POST'])
def save_preset_route():
    """Save a preset"""
    data = request.get_json()
    preset_id = data.get('preset_id')
    preset_data = data.get('data')

    presets = load_presets()
    presets[preset_id] = preset_data
    save_presets(presets)

    return jsonify({"status": "ok"})


@app.route('/api/weld_history')
def weld_history():
    """Get list of weld history files"""
    files = get_weld_history_list()

    # Get summary info for each weld
    welds = []
    for filename in files:
        filepath = os.path.join(WELD_HISTORY_DIR, filename)
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                welds.append({
                    "filename": filename,
                    "weld_number": data.get("weld_number"),
                    "timestamp": data.get("timestamp"),
                    "energy_joules": data.get("energy_joules"),
                    "peak_current_amps": data.get("peak_current_amps"),
                    "duration_ms": data.get("duration_ms"),
                    "rise_time_ms": data.get("rise_time_ms", None)
                })
        except Exception:
            pass

    return jsonify({"status": "ok", "welds": welds})


@app.route('/api/weld_data/<filename>')
def weld_data(filename):
    """Get specific weld data"""
    filepath = os.path.join(WELD_HISTORY_DIR, filename)
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            return jsonify(json.load(f))
    return jsonify({"status": "error", "message": "Weld not found"}), 404


@app.route('/api/clear_weld_history', methods=['POST'])
def clear_weld_history():
    """
    Clear all weld history JSON files and reset weld counter.
    Also resets the counter in settings so it stays consistent.
    """
    global weld_counter

    # 1. Delete weld history JSONs in WELD_HISTORY_DIR
    try:
        if os.path.isdir(WELD_HISTORY_DIR):
            for f in os.listdir(WELD_HISTORY_DIR):
                if f.startswith("weld_") and f.endswith(".json"):
                    try:
                        os.remove(os.path.join(WELD_HISTORY_DIR, f))
                    except OSError as e:
                        log(f"⚠️ Failed to remove {f}: {e}")
    except Exception as e:
        log(f"⚠️ Error while clearing weld history files: {e}")

    # 2. Reset weld counter
    weld_counter = 0

    # 3. Persist reset counter into settings.json
    settings = load_settings()
    settings['weld_counter'] = 0
    save_settings(settings)

    log("🧹 Weld history cleared and weld_counter reset to 0")
    return jsonify({"status": "ok"})


@app.route('/api/logs')
def api_logs():
    """Get recent logs"""
    return jsonify({"logs": list(log_buffer)})


# SocketIO events
@socketio.on('connect')
def handle_connect():
    log("WebSocket client connected")
    with status_lock:
        balance = calculate_cell_balance()
        emit('status_update', {
            **esp_status,
            **cells_status,
            "temperature": temperature,
            "weld_counter": weld_counter,
            "esp_connected": esp_connected,
            "cell_balance": balance
        })


@socketio.on('disconnect')
def handle_disconnect():
    log("WebSocket client disconnected")


@socketio.on('clear_weld_data')
def handle_clear_weld_data():
    """
    Socket.IO handler: Clear weld history files and reset weld counter.
    Emits 'weld_data_cleared' back to all clients when done.
    """
    global weld_counter

    log("🧹 Received clear_weld_data request from client")

    # 1. Delete weld history JSONs
    try:
        if os.path.isdir(WELD_HISTORY_DIR):
            for f in os.listdir(WELD_HISTORY_DIR):
                if f.startswith("weld_") and f.endswith(".json"):
                    try:
                        os.remove(os.path.join(WELD_HISTORY_DIR, f))
                    except OSError as e:
                        log(f"⚠️ Failed to remove {f}: {e}")
    except Exception as e:
        log(f"⚠️ Error while clearing weld history files: {e}")

    # 2. Reset weld counter in memory
    weld_counter = 0

    # 3. Persist reset counter into settings.json
    settings = load_settings()
    settings['weld_counter'] = 0
    save_settings(settings)

    log("🧹 Weld history cleared and weld_counter reset to 0")

    # 4. Notify all connected clients so they can refresh
    socketio.emit('weld_data_cleared')


# Background threads
def status_broadcast_thread():
    """Broadcast status updates (weld capture handled by on_esp_log)."""
    global pedal_active

    while True:
        time.sleep(0.1)  # 10 Hz

        with status_lock:
            balance = calculate_cell_balance()
            socketio.emit('status_update', {
                **esp_status,
                **cells_status,
                "temperature": temperature,
                "pedal_active": pedal_active,
                "weld_counter": weld_counter,
                "esp_connected": esp_connected,
                "cell_balance": balance
            })


def cleanup():
    """Cleanup GPIO on exit"""
    try:
        import RPi.GPIO as GPIO
        GPIO.cleanup()
        log("GPIO cleanup complete")
    except Exception:
        pass


import atexit
atexit.register(cleanup)


if __name__ == '__main__':
    print("=" * 60, flush=True)
    print("🔥 Spot Welder Control Server", flush=True)
    print("=" * 60, flush=True)

    # Load weld counter
    settings = load_settings()
    weld_counter = settings.get('weld_counter', 0)
    log(f"Weld counter: {weld_counter}")

    # Skip Pi ADS1256 – ADC is handled on ESP32 via WDATA
    log("Skipping Pi ADS1256 init (handled on ESP32)")
    adc = None

    # Initialize ESP32 link
    log("Initializing ESP32 link...")
    try:
        esp_link = ESP32Link(
            host='192.168.68.56',  # UPDATE THIS IP if your ESP32 has a different address
            port=8888,
            status_callback=on_esp_status,
            weld_data_callback=on_esp_log
        )

        if esp_link.start():
            log("✅ ESP32 link started")

            # Send saved settings to ESP32 after brief delay
            time.sleep(0.5)  # Brief delay for ESP32 to be ready
            settings = load_settings()
            d1 = settings.get('d1', 50)
            cmd = f"SET_PULSE,{d1}"
            log(f"📤 Syncing settings to ESP32: {cmd}")
            esp_link.send_command(cmd)
        else:
            log("⚠️ ESP32 link failed to start")
    except Exception as e:
        log(f"⚠️ ESP32 link init error: {e}")

    # Start background thread
    broadcast_thread = threading.Thread(target=status_broadcast_thread, daemon=True)
    broadcast_thread.start()
    log("Background broadcast thread started")

    # Run server
    log("Starting Flask-SocketIO server on port 8080")
    socketio.run(app, host='0.0.0.0', port=8080, debug=False, allow_unsafe_werkzeug=True)


@app.route('/api/arm', methods=['POST'])
def api_arm():
    """ARM the welder"""
    global esp32_ws
    if esp32_ws:
        try:
            esp32_ws.send("ARM")
            log("🔓 System ARMED")
            return jsonify({"status": "ok"})
        except Exception as e:
            log(f"⚠️ Failed to ARM: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500
    return jsonify({"status": "error", "message": "ESP32 not connected"}), 503


@app.route('/api/disarm', methods=['POST'])
def api_disarm():
    """DISARM the welder"""
    global esp32_ws
    if esp32_ws:
        try:
            esp32_ws.send("DISARM")
            log("🔒 System DISARMED")
            return jsonify({"status": "ok"})
        except Exception as e:
            log(f"⚠️ Failed to DISARM: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500
    return jsonify({"status": "error", "message": "ESP32 not connected"}), 503


@app.route('/api/fire', methods=['POST'])
def api_fire():
    """Manually fire a weld"""
    global esp32_ws
    if esp32_ws:
        try:
            # esp32_ws.send("FIRE")  # disabled: pedal-only mode
            # log("🔥 Manual FIRE command sent")  # disabled: pedal-only mode
            return jsonify({"status": "ok"})
        except Exception as e:
            # log(f"⚠️ Failed to FIRE: {e}")  # disabled: pedal-only mode
            return jsonify({"status": "error", "message": str(e)}), 500
    return jsonify({"status": "error", "message": "ESP32 not connected"}), 503


@app.route('/api/charge_on', methods=['POST'])
def api_charge_on():
    """Turn charging ON"""
    global esp_link
    if esp_link:
        try:
            cmd = "CHARGE_ON"
            log(f"⚡ Sending to ESP32 (TCP): {cmd}")
            esp_link.send_command(cmd)
            return jsonify({"status": "ok"})
        except Exception as e:
            log(f"⚠️ Failed to turn charging ON via TCP: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500
    else:
        log("⚠️ ESP32 TCP link not initialized for CHARGE_ON")
        return jsonify({"status": "error", "message": "ESP32 not connected"}), 503


@app.route('/api/charge_off', methods=['POST'])
def api_charge_off():
    """Turn charging OFF"""
    global esp_link
    if esp_link:
        try:
            cmd = "CHARGE_OFF"
            log(f"⏸️ Sending to ESP32 (TCP): {cmd}")
            esp_link.send_command(cmd)
            return jsonify({"status": "ok"})
        except Exception as e:
            log(f"⚠️ Failed to turn charging OFF via TCP: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500
    else:
        log("⚠️ ESP32 TCP link not initialized for CHARGE_OFF")
        return jsonify({"status": "error", "message": "ESP32 not connected"}), 503