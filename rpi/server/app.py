"""
Flask + SocketIO server for ESP32 spot welder control
Handles TCP connection, web UI, and real-time updates
"""

import os
import sys
import json
import time
import socket
import threading
from datetime import datetime
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit

# ========== CONFIGURATION ==========
ESP32_IP = "192.168.68.56"  # Change to your ESP32's IP
ESP32_PORT = 8888
SETTINGS_FILE = "settings.json"
PRESETS_FILE = "presets.json"
LOG_FILE = "welder.log"

# ========== FLASK SETUP ==========
app = Flask(__name__)
app.config['SECRET_KEY'] = 'spot-welder-secret-2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ========== GLOBAL STATE ==========
esp_link = None
esp_connected = False
last_status = {}

# ========== LOGGING ==========
def log(msg):
    """Log message to console and file"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_line = f"[{timestamp}] {msg}"
    print(log_line)
    try:
        with open(LOG_FILE, 'a') as f:
            f.write(log_line + '\n')
    except Exception as e:
        print(f"⚠️ Failed to write log: {e}")

# ========== ESP32 TCP LINK ==========
# ========== ESP32 TCP LINK ==========
# ========== ESP32 TCP LINK ==========
class ESP32Link:
    """Manages TCP connection to ESP32"""
    
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False
        self.running = True
        self.rx_thread = None
        
    def connect(self):
        """Establish TCP connection"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.connected = True
            log(f"✅ Connected to ESP32 at {self.host}:{self.port}")
            
            # Start receive thread
            self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.rx_thread.start()
            
            return True
        except Exception as e:
            log(f"❌ Failed to connect to ESP32: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close TCP connection"""
        self.running = False
        self.connected = False
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except:
                pass
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        log("🔌 Disconnected from ESP32")
    
    def send_command(self, cmd):
        """Send command to ESP32"""
        if not self.connected or not self.sock:
            log(f"⚠️ Cannot send command (not connected): {cmd}")
            return False
        
        try:
            if not cmd.endswith('\n'):
                cmd += '\n'
            self.sock.sendall(cmd.encode('utf-8'))
            log(f"📤 Sent: {cmd.strip()}")
            return True
        except Exception as e:
            log(f"❌ Failed to send command: {e}")
            self.disconnect()
            return False
    
    def _receive_loop(self):
        """Background thread to receive data from ESP32"""
        global esp_connected, last_status
        buffer = ""
        last_data_time = time.time()
        TIMEOUT_SECONDS = 10  # If no data for 10 seconds, assume disconnected
        
        while self.running and self.connected:
            try:
                data = self.sock.recv(1024).decode('utf-8', errors='ignore')
                if not data:
                    log("⚠️ ESP32 connection closed (recv returned empty)")
                    break
                
                # Update last data timestamp
                last_data_time = time.time()
                buffer += data
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self._handle_line(line)
                        
            except socket.timeout:
                # Check if we've gone too long without data
                if time.time() - last_data_time > TIMEOUT_SECONDS:
                    log(f"⚠️ No data received for {TIMEOUT_SECONDS}s, assuming disconnected")
                    break
                continue
            except Exception as e:
                log(f"❌ Receive error: {e}")
                break
        
        # Clean up when loop exits (connection lost)
        log("🧹 Receive loop exiting, cleaning up connection...")
        self.disconnect()
        esp_connected = False
        socketio.emit('status_update', {'esp_connected': False})
    
    def _handle_line(self, line):
        """Parse and handle incoming ESP32 data"""
        global esp_connected, last_status
        
        log(f"📥 Received: {line}")
        
        # Update connection status
        if not esp_connected:
            esp_connected = True
            socketio.emit('status_update', {'esp_connected': True})
        
        # Parse different message types (comma-separated format)
        if line.startswith("STATUS,"):
            self._parse_status(line[7:])
        elif line.startswith("CELLS,"):
            self._parse_cells(line[6:])
        elif line.startswith("WELD:") or line.startswith("WELD,"):
            socketio.emit('weld_event', {'message': line.split(',', 1)[1] if ',' in line else line})
        elif line.startswith("PEDAL:") or line.startswith("PEDAL,"):
            active = "pressed" in line.lower()
            socketio.emit('pedal_active', {'active': active})
        elif line.startswith("CHARGER:") or line.startswith("CHARGER,"):
            self._parse_charger(line.split(',', 1)[1] if ',' in line else line.split(':', 1)[1])
        else:
            socketio.emit('esp32_message', {'message': line})
    
    def _parse_status(self, data):
        """Parse STATUS line"""
        global last_status
        try:
            status = {}
            for pair in data.split(','):
                if '=' in pair:
                    key, val = pair.split('=', 1)
                    key = key.strip()
                    val = val.strip()
                    try:
                        if '.' in val:
                            status[key] = float(val)
                        else:
                            status[key] = int(val)
                    except:
                        status[key] = val
            
            if 'vpack' in status:
                status['vpack'] = status['vpack']
            if 'temp' in status:
                status['temperature'] = status['temp']
            if 'i' in status:
                status['current'] = status['i']
            
            last_status.update(status)
            status['esp_connected'] = True
            socketio.emit('status_update', status)
            socketio.emit('esp32_status', status)
            
        except Exception as e:
            log(f"⚠️ Failed to parse STATUS: {e}")
    
    def _parse_cells(self, data):
        """Parse CELLS line"""
        global last_status
        try:
            cells = {}
            for pair in data.split(','):
                if '=' in pair:
                    key, val = pair.split('=', 1)
                    key = key.strip()
                    val = val.strip()
                    if key.startswith('C'):
                        try:
                            cells[key] = float(val)
                        except:
                            pass
            
            last_status.update(cells)
            socketio.emit('status_update', cells)
            
        except Exception as e:
            log(f"⚠️ Failed to parse CELLS: {e}")
    
    def _parse_charger(self, data):
        """Parse CHARGER line"""
        global last_status
        try:
            if 'current=' in data:
                val = data.split('current=')[1].split('A')[0].strip()
                current = float(val)
                last_status['current'] = current
                socketio.emit('esp32_status', {'current': current, 'i': current})
        except Exception as e:
            log(f"⚠️ Failed to parse CHARGER: {e}")
# ========== SETTINGS MANAGEMENT ==========
def load_settings():
    """Load settings from JSON file"""
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            log(f"⚠️ Failed to load settings: {e}")
    
    # Default settings
    return {
        "mode": 1,
        "d1": 50,
        "gap1": 0,
        "d2": 0,
        "gap2": 0,
        "d3": 0,
        "power": 100,
        "preheat_enabled": False,
        "preheat_duration": 20,
        "preheat_power": 30,
        "active_preset": None
    }

def save_settings(settings):
    """Save settings to JSON file"""
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(settings, f, indent=2)
        return True
    except Exception as e:
        log(f"⚠️ Failed to save settings: {e}")
        return False

def load_presets():
    """Load presets from JSON file"""
    if os.path.exists(PRESETS_FILE):
        try:
            with open(PRESETS_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            log(f"⚠️ Failed to load presets: {e}")
    
    # Default presets
    return {
        "P1": {"name": "Preset 1", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0, "power": 100, "preheat_enabled": False, "preheat_duration": 20, "preheat_power": 30},
        "P2": {"name": "Preset 2", "mode": 1, "d1": 80, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0, "power": 100, "preheat_enabled": False, "preheat_duration": 20, "preheat_power": 30},
        "P3": {"name": "Preset 3", "mode": 2, "d1": 50, "gap1": 10, "d2": 50, "gap2": 0, "d3": 0, "power": 100, "preheat_enabled": False, "preheat_duration": 20, "preheat_power": 30},
        "P4": {"name": "Preset 4", "mode": 1, "d1": 100, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0, "power": 100, "preheat_enabled": False, "preheat_duration": 20, "preheat_power": 30},
        "P5": {"name": "Preset 5", "mode": 3, "d1": 40, "gap1": 10, "d2": 40, "gap2": 10, "d3": 40, "power": 100, "preheat_enabled": False, "preheat_duration": 20, "preheat_power": 30}
    }

def save_presets(presets):
    """Save presets to JSON file"""
    try:
        with open(PRESETS_FILE, 'w') as f:
            json.dump(presets, f, indent=2)
        return True
    except Exception as e:
        log(f"⚠️ Failed to save presets: {e}")
        return False

# ========== WEB ROUTES ==========
@app.route('/')
def index():
    """Redirect to control page"""
    return render_template('control.html')

@app.route('/control')
def control():
    """Main control page"""
    return render_template('control.html')

@app.route('/logs')
def logs():
    """Logs page"""
    try:
        with open(LOG_FILE, 'r') as f:
            log_lines = f.readlines()[-100:]  # Last 100 lines
        return render_template('logs.html', logs=log_lines)
    except:
        return render_template('logs.html', logs=[])

# ========== API ROUTES ==========
@app.route('/api/status')
def api_status():
    """Get current status"""
    global esp_connected, last_status
    return jsonify({
        "status": "ok",
        "esp_connected": esp_connected,
        "data": last_status
    })

@app.route('/api/get_settings')
def api_get_settings():
    """Get current settings"""
    settings = load_settings()
    return jsonify({"status": "ok", "settings": settings})

@app.route('/api/save_settings', methods=['POST'])
def api_save_settings():
    """Save settings and send to ESP32"""
    global esp_link
    
    data = request.get_json()
    if not data:
        return jsonify({"status": "error", "message": "No data provided"}), 400
    
    # Save to file
    if not save_settings(data):
        return jsonify({"status": "error", "message": "Failed to save settings"}), 500
    
    # Send to ESP32
    if esp_link and esp_link.connected:
        try:
            # Send pulse settings
            mode = data.get('mode', 1)
            d1 = data.get('d1', 50)
            gap1 = data.get('gap1', 0)
            d2 = data.get('d2', 0)
            gap2 = data.get('gap2', 0)
            d3 = data.get('d3', 0)
            
            cmd = f"SET_PULSE,{mode},{d1},{gap1},{d2},{gap2},{d3}"
            log(f"⚡ Sending to ESP32 (TCP): {cmd}")
            esp_link.send_command(cmd)
            
        except Exception as e:
            log(f"⚠️ Failed to send settings to ESP32: {e}")
    
    return jsonify({"status": "ok"})

@app.route('/api/set_power', methods=['POST'])
def api_set_power():
    """Set weld power percentage"""
    global esp_link
    data = request.get_json()
    power = data.get('power', 100)
    
    if esp_link and esp_link.connected:
        try:
            cmd = f"SET_POWER,{power}"
            log(f"⚡ Sending to ESP32 (TCP): {cmd}")
            esp_link.send_command(cmd)
            return jsonify({"status": "ok"})
        except Exception as e:
            log(f"⚠️ Failed to set power: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500
    else:
        log("⚠️ ESP32 TCP link not initialized for SET_POWER")
        return jsonify({"status": "error", "message": "ESP32 not connected"}), 503

@app.route('/api/set_preheat', methods=['POST'])
def api_set_preheat():
    """Set preheat configuration"""
    global esp_link
    data = request.get_json()
    enabled = 1 if data.get('enabled', False) else 0
    duration = data.get('duration', 20)
    power = data.get('power', 30)
    
    if esp_link and esp_link.connected:
        try:
            cmd = f"SET_PREHEAT,{enabled},{duration},{power}"
            log(f"🔥 Sending to ESP32 (TCP): {cmd}")
            esp_link.send_command(cmd)
            return jsonify({"status": "ok"})
        except Exception as e:
            log(f"⚠️ Failed to set preheat: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500
    else:
        log("⚠️ ESP32 TCP link not initialized for SET_PREHEAT")
        return jsonify({"status": "error", "message": "ESP32 not connected"}), 503

@app.route('/api/get_presets')
def api_get_presets():
    """Get all presets"""
    presets = load_presets()
    settings = load_settings()
    return jsonify({
        "status": "ok",
        "presets": presets,
        "active_preset": settings.get('active_preset')
    })

@app.route('/api/save_preset', methods=['POST'])
def api_save_preset():
    """Save a preset"""
    data = request.get_json()
    preset_id = data.get('preset_id')
    preset_data = data.get('data')
    
    if not preset_id or not preset_data:
        return jsonify({"status": "error", "message": "Missing preset_id or data"}), 400
    
    presets = load_presets()
    presets[preset_id] = preset_data
    
    if not save_presets(presets):
        return jsonify({"status": "error", "message": "Failed to save preset"}), 500
    
    return jsonify({"status": "ok"})

@app.route('/api/arm', methods=['POST'])
def api_arm():
    """Arm the welder"""
    global esp_link
    if esp_link and esp_link.connected:
        esp_link.send_command("ARM")
        return jsonify({"status": "ok"})
    return jsonify({"status": "error", "message": "ESP32 not connected"}), 503

@app.route('/api/disarm', methods=['POST'])
def api_disarm():
    """Disarm the welder"""
    global esp_link
    if esp_link and esp_link.connected:
        esp_link.send_command("DISARM")
        return jsonify({"status": "ok"})
    return jsonify({"status": "error", "message": "ESP32 not connected"}), 503

@app.route('/api/fire', methods=['POST'])
def api_fire():
    """Manual fire"""
    global esp_link
    if esp_link and esp_link.connected:
        esp_link.send_command("FIRE")
        return jsonify({"status": "ok"})
    return jsonify({"status": "error", "message": "ESP32 not connected"}), 503

@app.route('/api/charge_on', methods=['POST'])
def api_charge_on():
    """Turn charging ON"""
    global esp_link
    if esp_link and esp_link.connected:
        esp_link.send_command("CHARGE_ON")
        return jsonify({"status": "ok"})
    return jsonify({"status": "error", "message": "ESP32 not connected"}), 503

@app.route('/api/charge_off', methods=['POST'])
def api_charge_off():
    """Turn charging OFF"""
    global esp_link
    if esp_link and esp_link.connected:
        esp_link.send_command("CHARGE_OFF")
        return jsonify({"status": "ok"})
    return jsonify({"status": "error", "message": "ESP32 not connected"}), 503

# ========== SOCKETIO EVENTS ==========
@socketio.on('connect')
def handle_connect():
    """Client connected"""
    log(f"🌐 Client connected: {request.sid}")
    # Send current status
    emit('status_update', {**last_status, 'esp_connected': esp_connected})

@socketio.on('disconnect')
def handle_disconnect():
    """Client disconnected"""
    log(f"🌐 Client disconnected: {request.sid}")

# ========== STARTUP ==========
# ========== STARTUP ==========
# ========== STARTUP ==========
def init_esp32_connection():
    """Initialize ESP32 connection in background"""
    global esp_link, esp_connected
    
    log("🔌 Starting ESP32 connection manager...")
    
    while True:
        try:
            # Check if we need to reconnect
            if esp_link is None or not esp_link.connected:
                
                # Clean up old link if it exists
                if esp_link is not None:
                    try:
                        esp_link.disconnect()
                    except:
                        pass
                
                # Create fresh connection
                esp_link = ESP32Link(ESP32_IP, ESP32_PORT)
                
                if esp_link.connect():
                    esp_connected = True
                    socketio.emit('status_update', {'esp_connected': True})
                    log("✅ ESP32 connected successfully!")
                else:
                    esp_connected = False
                    socketio.emit('status_update', {'esp_connected': False})
                    log("❌ Connection failed, retrying in 3 seconds...")
                    time.sleep(3)
            else:
                # Already connected, just wait
                time.sleep(1)
                
        except Exception as e:
            log(f"❌ Connection manager error: {e}")
            esp_connected = False
            socketio.emit('status_update', {'esp_connected': False})
            time.sleep(3)

# ========== MAIN ==========
if __name__ == '__main__':
    log("🚀 Starting Spot Welder Control Server")
    log(f"📡 ESP32 Target: {ESP32_IP}:{ESP32_PORT}")
    
    # Start ESP32 connection thread
    esp_thread = threading.Thread(target=init_esp32_connection, daemon=True)
    esp_thread.start()
    
    # Start Flask server
    log("🌐 Starting web server on http://0.0.0.0:8080")
    socketio.run(app, host='0.0.0.0', port=8080, debug=False, allow_unsafe_werkzeug=True)