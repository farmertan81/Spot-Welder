# ====
#   Supercap Charger Control — ESP32-S3-Zero + INA226 + WS2812
#   Safe-boot + USB-stable + Boot-escape key (press any key at boot)
#   With per-address VBUS calibration scales + DEBUG telemetry
#   Button: short-press toggles enable; long-press enters deep sleep
#   Pedal: active-low, debounced, ignored when disabled
#   Shunt: CSS2H-3920R-L200F (0.0002 Ω)
#   ASCII Command Interface over USB-CDC AND UART1 (GPIO43/44)
#    PING | GET_STATUS | GET_CELLS | ENABLE | DISABLE | FIRE,<ms> | ABORT
#    DEBUG_ON | DEBUG_OFF | GET_TEMP
# ====

from machine import Pin, SoftI2C, deepsleep, UART, ADC

# ---- ADS1256 Current Sensor Import ----
ADS1256_AVAILABLE = False
try:
    import ads1256_esp32
    import sys
    if "ads1256_esp32" in sys.modules:
        del sys.modules["ads1256_esp32"]
        import ads1256_esp32
    from machine import SPI
    ADS1256_AVAILABLE = True
    print("✓ ADS1256 driver imported")
except Exception as e:
    print(f"ADS1256 import failed: {e}")
    ADS1256_AVAILABLE = False
import time, struct, neopixel
import machine, esp32
import math, sys, select

# ---- WiFi & FTP Setup ----
try:
    from wifi_config import WIFI_SSID, WIFI_PASSWORD
    from wifi_manager import connect_wifi, start_ftp_server
    
    print("\n" + "="*50)
    print("  WiFi Initialization")
    print("="*50)
    
    wifi_ip = connect_wifi(WIFI_SSID, WIFI_PASSWORD, timeout=15)
    if wifi_ip:
        print(f"✓ WiFi connected: {wifi_ip}")
        if start_ftp_server():
            print(f"✓ FTP server running")
            print(f"  Connect: ftp://{wifi_ip}")
            print(f"  User: esp32 / Pass: welder123")
    else:
        print("✗ WiFi connection failed - continuing without network")
    print("="*50 + "\n")
except Exception as e:
    print(f"WiFi setup error: {e}")
    print("Continuing without WiFi...\n")

# ---- TCP Server Setup ----
tcp_server = None
try:
    from tcp_server import TCPServer
    print("TCP: imported TCPServer OK")  # DEBUG
    tcp_server = TCPServer(port=8888)
    print("TCP: instance created")       # DEBUG
    if tcp_server.start():
        print("✓ TCP server started on port 8888")
        try:
            print("✓ TCP server started on %s:8888" % wifi_ip)
        except Exception:
            pass
    else:
        print("TCP: start() returned False")
        tcp_server = None
except Exception as e:
    print("TCP server setup error:")
    print(repr(e))
    tcp_server = None

# ---- DEBUG TOGGLE ----
DEBUG = False  # Set False to quiet detailed telemetry

def dbg(msg: str):
    if DEBUG:
        print_both("DBG " + msg)

I2C_SCL = 2
I2C_SDA = 3
INA_ADDR = 0x40  # Charger INA226 (pack shunt + Vpack)

FET_CHARGE = Pin(4, Pin.OUT, value=0)
FET_WELD1  = Pin(5, Pin.OUT, value=0)
FET_WELD2  = Pin(6, Pin.OUT, value=0)

BUTTON_PIN = 1
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
LONG_PRESS_MS = 1000
BUTTON_DEBOUNCE_MS = 30

# Weld pedal (active-low)
PEDAL_PIN = 7
pedal = Pin(PEDAL_PIN, Pin.IN, Pin.PULL_UP)
PEDAL_DEBOUNCE_MS = 25
WELD_DEBOUNCE_MS = 500  # Minimum time between welds (ms)
WELD_LOCKOUT_MS = 300
next_weld_ok_at = 0
last_pedal_change = 0
last_weld_time = 0
pedal_state = 1  # 1=idle, 0=pressed

# ---- Persistent weld pulse (used by pedal and can be set over ASCII) ----
PULSE_MS_DEFAULT = 10
PULSE_MS = PULSE_MS_DEFAULT

# Thermistor on GPIO8
THERM_PIN = 8
therm_adc = ADC(Pin(THERM_PIN))
therm_adc.atten(ADC.ATTN_11DB)  # 0-3.3V range

# Thermistor constants (CALIBRATED - your proven values)
SERIES_RESISTOR = 10000
THERMISTOR_NOMINAL = 173000
TEMPERATURE_NOMINAL = 20
BETA = 3950

# Thermistor smoothing (Klipper-style)
TEMP_EMA_ALPHA = 0.05  # Lower = smoother (0.05-0.2 typical)
TEMP_OUTLIER_THRESHOLD = 5.0  # Reject readings >5°C from current
temp_ema = None
temp_last_valid = None

# System enable latch (short-press toggle)
system_enabled = True
button_last_state = 1
button_pressed_ms = 0
button_debounce_ms = 0
long_press_fired = False

LED_PIN = 21
led = neopixel.NeoPixel(Pin(LED_PIN), 1)

def set_led(r, g, b):
    led[0] = (g, r, b)  # GRB order
    led.write()
def led_white(): set_led(80,80,80)
def led_green(): set_led(0,120,0)
def led_blue():  set_led(0,0,120)
def led_red():   set_led(120,0,0)
def led_orange():set_led(120,40,0)
def led_off():   set_led(0,0,0)

# --- Wake reason LED hint ---
if machine.reset_cause() == machine.DEEPSLEEP_RESET:
    print("Woke from deep sleep")
    for _ in range(2):
        led_white(); time.sleep(0.3); led_off(); time.sleep(0.2)
else:
    print("Cold boot / power-on")
    led_white()

# Give USB-CDC time to settle after soft/hard reboot (helps COM flakiness)
time.sleep(0.75)

# ---- SAFE MODE (hold BOOT=GPIO0 low during reset) ----
try:
    SAFE_MODE = (Pin(0, Pin.IN, Pin.PULL_UP).value() == 0)
except Exception:
    SAFE_MODE = False
if SAFE_MODE:
    print("SAFE MODE: skipping INA and charger logic; REPL available.")
    led_blue()

# ---- BOOT-ESCAPE: press any key within 1 s to skip charger/sensors ----
ESCAPE_MODE = False
if not SAFE_MODE:
    esc_deadline = time.ticks_add(time.ticks_ms(), 1000)
    sp_boot = select.poll()
    sp_boot.register(sys.stdin, select.POLLIN)
    print("Press any key in 1s to enter ESCAPE mode...")
    while time.ticks_diff(esc_deadline, time.ticks_ms()) > 0:
        ev = sp_boot.poll(0)
        if ev:
            try:
                _ = sys.stdin.read(1)  # consume one byte
            except Exception:
                pass
            ESCAPE_MODE = True
            break
        time.sleep(0.01)
    sp_boot.unregister(sys.stdin)
    if ESCAPE_MODE:
        print("ESCAPE MODE: charger/sensors skipped; REPL available.")
        led_blue()

# ---- UART COMMAND INTERFACE (GPIO12/13 -> Pi UART GPIO14/15) ----
UART_TX_PIN = 12  # ESP TX -> Pi RXD0 (GPIO15)
UART_RX_PIN = 13  # ESP RX -> Pi TXD0 (GPIO14)

uart = None
try:
    uart = UART(1, 115200)
    uart.init(115200, bits=8, parity=None, stop=1, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
    time.sleep(0.05)
    # Test burst
    if uart:
        for i in range(10):
            uart.write(b"HELLO_PI\n")
            time.sleep(0.2)
    print("UART1 ready @115200 on GPIO12/13")

# ---- ADS1256 Current Sensor Initialization ----
except Exception as e:
    print("UART init failed:", e)
    uart = None

# ---- ADS1256 Current Sensor Initialization ----
weld_samples = []
weld_interrupt_samples = []
weld_interrupt_active = False
weld_start_time = 0
MAX_WELD_SAMPLES = 1000

if ADS1256_AVAILABLE:
    try:
        spi_ads = SPI(1, baudrate=1000000, polarity=0, phase=1,
                      sck=Pin(40), mosi=Pin(39), miso=Pin(38))
        adc_current = ads1256_esp32.ADS1256(spi_ads, cs_pin=17, drdy_pin=18)
        print("✓ ADS1256 current sensor initialized")
        print(f"ADS1256 offset: {adc_current.offset_voltage:.6f} V")
    except Exception as e:
        print(f"ADS1256 init failed: {e}")
        ADS1256_AVAILABLE = False
        adc_current = None
else:
    adc_current = None
    print("ADS1256 driver not available")

# ---- UART line reader (robust) ----
_uart_buf = b""
def uart_try_read_line():
    global _uart_buf
    if not uart:
        return None
    try:
        while uart.any():
            d = uart.read()
            if d:
                _uart_buf += d
        if len(_uart_buf) > 2048:
            _uart_buf = _uart_buf[-1024:]
        nl = _uart_buf.find(b"\n")
        cr = _uart_buf.find(b"\r")
        idx = -1
        if nl != -1 and cr != -1:
            idx = min(nl, cr)
        else:
            idx = nl if nl != -1 else cr
        if idx != -1:
            line = _uart_buf[:idx]
            _uart_buf = _uart_buf[idx+1:]
            try:
                return line.decode('utf-8').strip()
            except:
                return ''
    except Exception:
        pass
    return None

def print_both(s: str):
    global uart, tcp_server
    # Print to console
    try:
        print(s)
    except Exception as e:
        print("ERR: print() failed:", e)
    # Send to UART
    try:
        if uart:
            uart.write((s + "\n").encode())
    except Exception as e:
        print("ERR: uart.write() failed:", e)
    # Send to TCP clients
    try:
        if tcp_server:
            tcp_server.send_to_all(s)
    except Exception as e:
        print("ERR: tcp send failed:", e)

# ---- Thermistor reading ----
def read_thermistor_temp():
    global temp_ema, temp_last_valid
    try:
        adc_value = therm_adc.read()
        voltage = (adc_value / 4095.0) * 3.3

        # If ADC returned None or 0, bail
        if adc_value is None:
            return float('nan')

        if voltage <= 0.0 or voltage >= 3.3:
            return float('nan')

        resistance = SERIES_RESISTOR * voltage / (3.3 - voltage)

        steinhart = resistance / THERMISTOR_NOMINAL
        steinhart = math.log(steinhart)
        steinhart /= BETA
        steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15)
        steinhart = 1.0 / steinhart
        steinhart -= 273.15

        # Outlier rejection
        if temp_last_valid is not None:
            if abs(steinhart - temp_last_valid) > TEMP_OUTLIER_THRESHOLD:
                return temp_last_valid  # Reject spike, return last good value
        
        # Exponential moving average
        if temp_ema is None:
            temp_ema = steinhart  # First reading
        else:
            temp_ema = (TEMP_EMA_ALPHA * steinhart) + ((1 - TEMP_EMA_ALPHA) * temp_ema)
        
        temp_last_valid = temp_ema
        return round(temp_ema, 1)
    except Exception as e:
        return temp_last_valid if temp_last_valid is not None else float('nan')

# ---- INA226 / Scaling ----
def make_i2c():
    try:
        return SoftI2C(sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=40000)
    except Exception as e:
        print("I2C init error:", e)
        return None

i2c = make_i2c()

REG_CONFIG      = 0x00
REG_SHUNT_VOLT  = 0x01
REG_BUS_VOLT    = 0x02
REG_POWER       = 0x03
REG_CURRENT     = 0x04
REG_CALIB       = 0x05

# CONFIG: AVG=16, VBUSCT=1.1ms, VSHCT=1.1ms, MODE=cont shunt+bus -> 0x4527
CONFIG = ((0b010 << 9) | (0b100 << 6) | (0b100 << 3) | 0b111)

# Shunt and LSBs for CSS2H-3920R-L200F
R_SHUNT         = 0.0002
VSHUNT_LSB      = 2.5e-6
VBUS_LSB        = 0.00125

CURRENT_LSB     = 0.00278
POWER_LSB       = 25.0 * CURRENT_LSB
EXPECTED_CAL    = 0x23F8

print_both("CAL target (locked) = 0x%04X (dec %d), CURRENT_LSB=%.6f A/bit" %
           (EXPECTED_CAL, EXPECTED_CAL, CURRENT_LSB))

VBUS_SCALE = {0x44: 0.9682, 0x41: 0.9977, 0x40: 0.9982}
print_both("VBUS_SCALE in use: " + str(VBUS_SCALE))
print_both("Address mapping: V1=0x44  V2=0x41  V3/Vpack=0x40")

I_SCALE         = 1.0
I_USER_SCALE    = 2.65  # empirical correction: clamp / INA (≈15.3 / 4.83)

last_vpack = None
V_JUMP_MAX = 2.0
I_OFFSET = 0.0

CAL_CHECK_MS = 5000
next_cal_check = 0

ADDR_CELL1 = 0x44
ADDR_CELL2 = 0x41

# ---- Robust I2C helpers ----
def i2c_scan():
    if not i2c: return []
    try: return i2c.scan()
    except Exception as e:
        print("I2C scan error:", e)
        return []

def ina_present():
    try:
        return (i2c is not None) and (INA_ADDR in i2c_scan())
    except Exception:
        return False

def ina_write16(reg, val):
    i2c.writeto_mem(INA_ADDR, reg, bytes(((val >> 8) & 0xFF, val & 0xFF)))

def ina_read16_u(reg):
    return struct.unpack('>H', i2c.readfrom_mem(INA_ADDR, reg, 2))[0]

def recover_i2c_and_reinit():
    global i2c
    time.sleep(0.01)
    i2c = make_i2c()
    if not (SAFE_MODE or ESCAPE_MODE):
        ina_init()

def i2c_read_u16(reg):
    for _ in range(2):
        try:
            return ina_read16_u(reg)
        except OSError:
            if not ina_present():
                recover_i2c_and_reinit()
            time.sleep(0.01)
        except Exception:
            break
    return None

def update_I_SCALE_from_CAL(cal_now):
    global I_SCALE
    if cal_now and cal_now != EXPECTED_CAL:
        I_SCALE = EXPECTED_CAL / cal_now
        print_both("I_SCALE updated = %.6f (CAL=0x%04X)" % (I_SCALE, cal_now))
    else:
        I_SCALE = 1.0

def ina_init():
    if not i2c:
        print_both("WARN: No I2C; skipping INA init.")
        return False
    if not ina_present():
        return False
    try:
        ina_write16(REG_CONFIG, 0x8000)
    except OSError:
        pass
    time.sleep(0.02)

    ina_write16(REG_CONFIG, CONFIG)
    time.sleep(0.006)
    try:
        cfg = ina_read16_u(REG_CONFIG)
    except OSError:
        print_both("INA read cfg failed")
        return False

    ina_write16(REG_CALIB, EXPECTED_CAL)
    time.sleep(0.004)
    try:
        cal = ina_read16_u(REG_CALIB)
    except OSError:
        print_both("INA read cal failed")
        return False

    try:
        _ = ina_read16_u(REG_CURRENT)
    except OSError:
        pass
    time.sleep(0.003)

    print_both("I2C scan: " + str([hex(x) for x in i2c_scan()]))
    print_both("INA226 CONFIG=0x%04X CAL=0x%04X (expected 0x%04X)" % (cfg, cal, EXPECTED_CAL))
    update_I_SCALE_from_CAL(cal)

    # Debug snapshot at init
    try:
        raw_bus = ina_read16_u(REG_BUS_VOLT)
        raw_sh  = ina_read16_u(REG_SHUNT_VOLT)
        if raw_sh & 0x8000: raw_sh -= 0x10000
        raw_cur = ina_read16_u(REG_CURRENT)
        if raw_cur & 0x8000: raw_cur -= 0x10000

        v_raw       = raw_bus * VBUS_LSB
        v_scaled    = v_raw * VBUS_SCALE.get(INA_ADDR, 1.0)
        vsh         = raw_sh * VSHUNT_LSB
        i_phys      = vsh / R_SHUNT
        i_inareg    = raw_cur * CURRENT_LSB * I_SCALE

        dbg("Vraw=%.3f V  Vscaled=%.3f V  Scale=%.5f" % (v_raw, v_scaled, VBUS_SCALE.get(INA_ADDR, 1.0)))
        dbg("Ishunt=%.3f A  Ireg=%.3f A  (raw_shunt=%d)" % (i_phys, i_inareg, raw_sh))
    except Exception:
        pass

    t0 = time.ticks_ms()
    last = cal
    while time.ticks_diff(time.ticks_ms(), t0) < 800:
        time.sleep(0.2)
        cnow = i2c_read_u16(REG_CALIB)
        if cnow is None: break
        if cnow != last:
            print_both("NOTE: CAL changed 0x%04X -> 0x%04X" % (last, cnow))
            last = cnow
            update_I_SCALE_from_CAL(cnow)
    return True

# ---- Read helpers ----
def read_voltage_avg(n=3, delay=0.02):
    total = 0.0; k = 0
    for _ in range(n):
        raw = i2c_read_u16(REG_BUS_VOLT)
        if raw is not None:
            total += raw * VBUS_LSB
            k += 1
        time.sleep(delay)
    return (total / k * VBUS_SCALE.get(INA_ADDR, 1.0)) if k else float('nan')

def read_voltage_filtered():
    global last_vpack
    v = read_voltage_avg()
    if not (v == v):
        return v
    if last_vpack is not None and abs(v - last_vpack) > V_JUMP_MAX:
        return last_vpack
    last_vpack = v
    return v

def read_shunt_V():
    raw = i2c_read_u16(REG_SHUNT_VOLT)
    if raw is None: return float('nan')
    if raw & 0x8000: raw -= 0x10000
    return raw * VSHUNT_LSB

def ina_current_once():
    raw = i2c_read_u16(REG_CURRENT)
    if raw is None: return float('nan')
    if raw & 0x8000: raw -= 0x10000
    # Apply hardware CAL scaling *and* user correction factor
    return (raw * CURRENT_LSB) * I_SCALE * I_USER_SCALE

def ina_current_avg(n=3, delay=0.01):
    acc = 0.0; k = 0
    for _ in range(n):
        i = ina_current_once()
        if i == i:
            acc += i; k += 1
        time.sleep(delay)
    return (acc / k) if k else float('nan')

# ---- Cells (per-address scaling + ordering guard) ----
def read_bus_volts_addr(addr):
    if not i2c: return None
    try:
        raw = struct.unpack('>H', i2c.readfrom_mem(addr, REG_BUS_VOLT, 2))[0]
        v = (raw * VBUS_LSB) * VBUS_SCALE.get(addr, 1.0)
        return v
    except OSError:
        return None

def read_cells_once():
    V1 = read_bus_volts_addr(ADDR_CELL1)
    V2 = read_bus_volts_addr(ADDR_CELL2)
    V3 = read_voltage_filtered()
    if (V1 is None) or (V2 is None) or not (V3 == V3):
        return None
    return (V1, V2, V3, V1, V2 - V1, V3 - V2)

# ---- Weld trigger helpers ----
def weld_safe_to_fire(v_pack_now):
    if not system_enabled:
        print_both("WARN: FIRE ignored — system disabled")
        return False, "DISABLED"
    if v_pack_now != v_pack_now:
        print_both("WARN: FIRE ignored — Vpack NaN")
        return False, "NO_VPACK"
    if v_pack_now < 7.80:
        print_both("WARN: FIRE ignored — pack below hard min (7.80 V)")
        return False, "LOW_V"
    if time.ticks_diff(time.ticks_ms(), next_weld_ok_at) < 0:
        return False, "COOLDOWN"
    if FET_CHARGE.value():
        print_both("WARN: FIRE ignored — charge FET ON")
        return False, "CHG_ON"
    return True, "OK"

# Global for interrupt-based sampling
weld_interrupt_samples = []
weld_interrupt_active = False

def drdy_interrupt_handler(pin):
    """ISR: Called when DRDY goes low (data ready)"""
    global weld_interrupt_samples, weld_interrupt_active
    if weld_interrupt_active and ADS1256_AVAILABLE and adc_current:
        try:
            t_us = time.ticks_diff(time.ticks_us(), weld_start_time)
            current = adc_current.read_current_isr()
            if current is not None:
                weld_interrupt_samples.append((t_us, current))
        except:
            pass

def do_weld_ms(pulse_ms):
    global next_weld_ok_at, weld_samples, weld_interrupt_samples, weld_interrupt_active, weld_start_time
    if pulse_ms < 1: pulse_ms = 1
    if pulse_ms > 5000: pulse_ms = 5000
    try:
        prev = led[0]
    except Exception:
        prev = (0,0,0)
    led_red()

    # Turn OFF charger FET during weld (critical!)
    FET_CHARGE.off()
    time.sleep(0.001)

    # Clear sample buffers
    weld_samples = []

    # Read starting voltage ONCE
    v_start = None
    raw = i2c_read_u16(REG_BUS_VOLT)
    if raw is not None:
        v_start = (raw * VBUS_LSB) * VBUS_SCALE.get(INA_ADDR, 1.0)

    # Start ADS1256 continuous mode (NO INTERRUPT)
    if ADS1256_AVAILABLE and adc_current:
        adc_current.start_continuous()
        time.sleep_us(500)

    # Set t=0 and FIRE!
    weld_start_time = time.ticks_us()
    t_end_us = weld_start_time + (pulse_ms * 1000)
    FET_WELD1.on(); FET_WELD2.on()

    # POLLING LOOP - read as fast as possible
    while time.ticks_diff(t_end_us, time.ticks_us()) > 0:
        if ADS1256_AVAILABLE and adc_current:
            current = adc_current.read_current_fast()
            if current is not None:
                t_us = time.ticks_diff(time.ticks_us(), weld_start_time)
                weld_samples.append((t_us, current))

    # Turn off FETs
    FET_WELD1.off(); FET_WELD2.off()
    t_actual_us = time.ticks_diff(time.ticks_us(), weld_start_time)

    # Stop ADS1256
    if ADS1256_AVAILABLE and adc_current:
        adc_current.stop_continuous()

    # Read ending voltage ONCE
    v_end = None
    raw = i2c_read_u16(REG_BUS_VOLT)
    if raw is not None:
        v_end = (raw * VBUS_LSB) * VBUS_SCALE.get(INA_ADDR, 1.0)

    # Now send all the data over UART
    for t_us, current in weld_samples:
        v_interp = v_start if v_start else 0.0
        if v_start and v_end and t_actual_us > 0:
            v_interp = v_start - (v_start - v_end) * (t_us / t_actual_us)
        print_both("WDATA,%.3f,%.1f,%d" % (v_interp, current, t_us))

    # Calculate statistics
    peak_current = 0.0
    energy_j = 0.0

    if len(weld_samples) > 1:
        for _, i in weld_samples:
            if abs(i) > abs(peak_current):
                peak_current = i

        R_total = 0.00106
        for idx in range(len(weld_samples) - 1):
            t1, i1 = weld_samples[idx]
            t2, i2 = weld_samples[idx + 1]
            dt = (t2 - t1) / 1e6
            p_avg = ((i1**2 + i2**2) / 2.0) * R_total
            energy_j += p_avg * dt

    v_drop = 0.0
    if v_start is not None and v_end is not None:
        v_drop = v_start - v_end

    print_both("WDATA_END,%d,%d,%.1f,%.2f,%.3f" % (len(weld_samples), t_actual_us, peak_current, energy_j, v_drop))

    try:
        led[0] = prev
        led.write()
    except Exception:
        pass


def trigger_weld():
    global PULSE_MS, last_weld_time
    now = time.ticks_ms()
    if time.ticks_diff(now, last_weld_time) < WELD_DEBOUNCE_MS:
        print_both("PEDAL: ignored (weld debounce)")
        return
    last_weld_time = now
    print_both("PEDAL: trigger weld (both)")
    t0_us = time.ticks_us()
    do_weld_ms(PULSE_MS)
    t1_us = time.ticks_us()
    print_both("FIRED,%d,%d,%d" % (PULSE_MS, t0_us, t1_us))
    
def shutdown_procedure():
    print_both("Long‑press detected — shutting down charger...")
    led_red()
    FET_CHARGE.off(); FET_WELD1.off(); FET_WELD2.off()
    time.sleep(0.5)
    led_off()
    print_both("Entering deep sleep. Press power button to wake.")
    wake_button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    esp32.wake_on_ext0(pin=wake_button, level=0)
    deepsleep()

# ---- Utility: idle current offset ----
def measure_idle_current_offset(duration_s=1.0):
    # Ensure charger FET is off during measurement
    FET_CHARGE.off()
    t_end = time.ticks_add(time.ticks_ms(), int(duration_s * 1000))
    n = 0; acc = 0.0
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        i = ina_current_avg(2, 0.005)
        if i == i:
            acc += i; n += 1
        time.sleep(0.02)
    return (acc / n) if n > 0 else 0.0

print_both("\nSMART CHARGER TEST — ESP32‑S3‑Zero")
print_both("----")

# ---- Charger thresholds ----
CHARGE_LIMIT    = 9.02
CHARGE_RESUME   = 8.70
HARD_LIMIT      = 9.05

high_count = 0
CAL_CHECK_MS = 5000
next_cal_check = time.ticks_add(time.ticks_ms(), CAL_CHECK_MS)

_last_print = 0
PRINT_MS = 1500

# ---- ASCII command over USB + UART ----
# USB non-blocking reader
sp_cmd = select.poll()
sp_cmd.register(sys.stdin, select.POLLIN)
cmd_buf = ""

def usb_try_read_line():
    global cmd_buf
    ev = sp_cmd.poll(0)
    if not ev:
        return None
    try:
        data = sys.stdin.read()
    except Exception:
        return None
    if not data:
        return None
    cmd_buf += data
    if "\n" in cmd_buf:
        line, _, rest = cmd_buf.partition("\n")
        cmd_buf = rest
        return line.strip()
    return None

def cmd_handle(line):
    global system_enabled, next_weld_ok_at, DEBUG, PULSE_MS
    try:
        if not line:
            return
        up = line.strip().upper()

        if up == "PING":
            print_both("PONG")
            return

        if up == "DEBUG_ON":
            DEBUG = True
            print_both("OK,DEBUG_ON")
            return

        if up == "DEBUG_OFF":
            DEBUG = False
            print_both("OK,DEBUG_OFF")
            return

        if up == "GET_TEMP":
            temp = read_thermistor_temp()
            if temp == temp:
                print_both("TEMP,%.1f" % temp)
            else:
                print_both("TEMP,NaN")
            return

        if up == "GET_STATUS":
            v_pack = read_voltage_filtered()
            i_now = ina_current_avg(2, 0.005) - I_OFFSET
            temp = read_thermistor_temp()
            state = ("DISABLED" if not system_enabled else
                     ("SAFE" if SAFE_MODE else
                      ("ESCAPE" if ESCAPE_MODE else
                       ("ON" if FET_CHARGE.value() else "OFF"))))
            vp = ("%.3f" % v_pack) if (v_pack == v_pack) else "NaN"
            ip = ("%.3f" % i_now) if (i_now == i_now) else "NaN"
            tp = ("%.1f" % temp) if (temp == temp) else "NaN"
            print_both("STATUS,enabled=%d,state=%s,vpack=%s,i=%s,temp=%s,cooldown_ms=%d,pulse_ms=%d" %
                       (1 if system_enabled else 0, state, vp, ip, tp,
                        max(0, time.ticks_diff(next_weld_ok_at, time.ticks_ms())), PULSE_MS))
            return

        if up == "GET_CELLS":
            cells = read_cells_once()
            if cells:
                V1, V2, V3, C1, C2, C3 = cells
                print_both("CELLS,V1=%.3f,V2=%.3f,V3=%.3f,C1=%.3f,C2=%.3f,C3=%.3f" %
                           (V1, V2, V3, C1, C2, C3))
            else:
                print_both("CELLS,NONE")
            return

        if up == "ENABLE":
            system_enabled = True
            print_both("OK,ENABLED")
            return

        if up == "DISABLE":
            system_enabled = False
            FET_CHARGE.off(); FET_WELD1.off(); FET_WELD2.off()
            print_both("OK,DISABLED")
            return

        if up.startswith("SET,"):
            # SET,mode=1,d1=50,gap1=10,d2=30,gap2=10,d3=20
            try:
                parts = line.split(",")[1:]
                params = {}
                for p in parts:
                    if "=" in p:
                        k, v = p.split("=", 1)
                        params[k.strip()] = int(float(v.strip()))

                # For now, just handle d1 (single pulse mode)
                if "d1" in params:
                    ms = params["d1"]
                    if ms < 1: ms = 1
                    if ms > 5000: ms = 5000
                    PULSE_MS = ms

                print_both("OK,SET")
            except Exception as e:
                print_both("ERR,SET:%s" % repr(e))
            return

        if up.startswith("SET_PULSE"):
            parts = line.split(",")
            if len(parts) != 2:
                print_both("ERR,USAGE_SET_PULSE")
                return
            try:
                ms = int(float(parts[1]))
            except Exception:
                print_both("ERR,BAD_MS")
                return
            if ms < 1: ms = 1
            if ms > 5000: ms = 5000
            PULSE_MS = ms
            print_both("OK,SET_PULSE,%d" % ms)
            # Emit STATUS immediately so monitor updates instantly
            v_pack = read_voltage_filtered()
            i_now = ina_current_avg(2, 0.005) - I_OFFSET
            temp = read_thermistor_temp()
            state = ("DISABLED" if not system_enabled else ("ON" if FET_CHARGE.value() else "OFF"))
            vp = ("%.3f" % v_pack) if (v_pack == v_pack) else "NaN"
            ip = ("%.3f" % i_now) if (i_now == i_now) else "NaN"
            tp = ("%.1f" % temp) if (temp == temp) else "NaN"
            print_both("STATUS,enabled=%d,state=%s,vpack=%s,i=%s,temp=%s,cooldown_ms=%d,pulse_ms=%d" %
                       (1 if system_enabled else 0, state, vp, ip, tp,
                        max(0, time.ticks_diff(next_weld_ok_at, time.ticks_ms())), PULSE_MS))
            return

        if up == "GET_PULSE":
            print_both("PULSE,%d" % PULSE_MS)
            return

        if up.startswith("SET_PATTERN"):
            try:
                parts = line.split(",")[1:]
                p1_ms = None
                for p in parts:
                    p = p.strip()
                    if p.upper().startswith("P1="):
                        p1_ms = int(float(p.split("=", 1)[1]))
                        break
                if p1_ms is None:
                    print_both("ERR,NO_P1")
                    return
                if p1_ms < 1: p1_ms = 1
                if p1_ms > 5000: p1_ms = 5000
                PULSE_MS = p1_ms
                print_both("OK,SET_PATTERN_P1,%d" % p1_ms)
            except Exception:
                print_both("ERR,SET_PATTERN")
            return

        if up.startswith("FIRE"):
            parts = line.split(",")
            if len(parts) == 2:
                try:
                    ms = int(float(parts[1]))
                except Exception:
                    print_both("ERR,BAD_MS")
                    return
            else:
                print_both("ERR,USAGE_FIRE")
                return

            v_now = read_voltage_filtered()
            safe, reason = weld_safe_to_fire(v_now)
            if not safe:
                print_both("ERR,NOFIRE,%s" % reason)
                return

            t0_us = time.ticks_us()
            do_weld_ms(ms)
            t1_us = time.ticks_us()
            print_both("FIRED,%d,%d,%d" % (ms, t0_us, t1_us))
            return

        if up == "ABORT":
            FET_WELD1.off(); FET_WELD2.off()
            print_both("OK,ABORTED")
            return

        print_both("ERR,UNKNOWN_CMD")

    except Exception as e:
        print_both("ERR_CMD:%s" % repr(e))
# Initialize INA now
if not (SAFE_MODE or ESCAPE_MODE):
    if not ina_init():
        print_both("WARN: INA226 not detected — charger disabled until present.")
        system_enabled = False
        led_orange()
    else:
        print_both("Measuring idle current offset...")
        I_OFFSET = measure_idle_current_offset(1.0)
        print_both("DBG I_OFFSET (idle trim) = %.4f A" % I_OFFSET)
        
heartbeat = 0
try:
    while True:
        # Heartbeat out over TCP/print_both
        heartbeat += 1
        if heartbeat >= 20:  # ~ once per second (20 * 50ms)
            heartbeat = 0
            print_both("HB: main loop alive")

        # Accept new TCP clients
        if tcp_server:
            tcp_server.accept_clients()

        # Button handling (debounced)
        b_now = button.value()    # 1 = released, 0 = pressed
        t_now = time.ticks_ms()

        if b_now != button_last_state and time.ticks_diff(t_now, button_debounce_ms) > BUTTON_DEBOUNCE_MS:
            button_debounce_ms = t_now
            if b_now == 0:
                button_pressed_ms = t_now
                long_press_fired = False
            else:
                held_ms = time.ticks_diff(t_now, button_pressed_ms)
                if held_ms <= LONG_PRESS_MS:
                    system_enabled = not system_enabled
                    if not system_enabled:
                        FET_CHARGE.off(); FET_WELD1.off(); FET_WELD2.off()
                        led_orange()
                        print_both("System DISABLED (short press) — all FETs OFF")
                        next_weld_ok_at = time.ticks_add(t_now, WELD_LOCKOUT_MS)
                    else:
                        print_both("System ENABLED (short press)")
            button_last_state = b_now

        if b_now == 0 and button_pressed_ms != 0 and not long_press_fired:
            if time.ticks_diff(t_now, button_pressed_ms) > LONG_PRESS_MS:
                long_press_fired = True
                shutdown_procedure()

        # Weld pedal handling
        now = t_now
        ps = pedal.value()
        if ps != pedal_state:
            if time.ticks_diff(now, last_pedal_change) > PEDAL_DEBOUNCE_MS:
                last_pedal_change = now
                pedal_state = ps
                if ps == 0 and time.ticks_diff(now, next_weld_ok_at) >= 0:
                    trigger_weld()

        # Re-init if INA disappears
        if not (SAFE_MODE or ESCAPE_MODE) and not ina_present():
            recover_i2c_and_reinit()

        # Periodic CAL tracking
        if not (SAFE_MODE or ESCAPE_MODE) and time.ticks_diff(t_now, next_cal_check) >= 0:
            cal_now = i2c_read_u16(REG_CALIB)
            if cal_now is not None and cal_now != EXPECTED_CAL:
                print_both("WARN: CAL drifted to 0x%04X — restoring 0x%04X" % (cal_now, EXPECTED_CAL))
                try:
                    ina_write16(REG_CALIB, EXPECTED_CAL)
                    time.sleep(0.004)
                    cal_v = ina_read16_u(REG_CALIB)
                    update_I_SCALE_from_CAL(cal_v)
                except OSError:
                    pass
            next_cal_check = time.ticks_add(t_now, CAL_CHECK_MS)

        # Measurements
        v_pack = read_voltage_filtered()
        current = ina_current_avg(3, 0.005) - I_OFFSET
        power = (v_pack if (v_pack == v_pack) else 0.0) * (current if (current == current) else 0.0)
        temp = read_thermistor_temp()

        # Charger control (voltage window)
        if system_enabled and (v_pack == v_pack):
            if v_pack >= HARD_LIMIT:
                high_count += 1
            else:
                high_count = 0
            if high_count >= 2:
                FET_CHARGE.off(); led_red(); state = "!!! HARD OFF !!!"
            elif v_pack >= CHARGE_LIMIT:
                FET_CHARGE.off(); led_blue(); state = "OFF"
            elif v_pack < CHARGE_RESUME:
                FET_CHARGE.on(); led_green(); state = "ON"
            else:
                state = "ON" if FET_CHARGE.value() else "OFF"
                (led_green() if state == "ON" else led_blue())
        else:
            state = ("DISABLED" if not system_enabled else
                     ("SAFE" if SAFE_MODE else
                      ("ESCAPE" if ESCAPE_MODE else "INIT")))
            FET_CHARGE.off()

        # Throttled status prints
        if time.ticks_diff(t_now, _last_print) >= PRINT_MS:
            _last_print = t_now
            cells = read_cells_once()
            i_str = ("%.3f" % current) if (current == current) else "NaN"
            v_str = ("%.3f" % v_pack) if (v_pack == v_pack) else "NaN"
            t_str = ("%.1f" % temp) if (temp == temp) else "NaN"
            
            # Periodic debug snapshot
            if DEBUG:
                sh = i2c_read_u16(REG_SHUNT_VOLT)
                cur = i2c_read_u16(REG_CURRENT)
                if sh is not None and (sh & 0x8000): sh -= 0x10000
                if cur is not None and (cur & 0x8000): cur -= 0x10000
                vsh = (sh or 0) * VSHUNT_LSB
                i_phys = vsh / R_SHUNT if sh is not None else float('nan')
                i_inareg = (cur or 0) * CURRENT_LSB * I_SCALE * I_USER_SCALE
                cal_now = i2c_read_u16(REG_CALIB) or 0
                dbg("Vsh=%.6f V  I_phys=%.3f A  I_INA=%.3f A  CAL=0x%04X  I_SCALE=%.6f SCALE=%.3f" %
                    (vsh, i_phys, i_inareg, cal_now, I_SCALE, I_USER_SCALE))
            
            print_both("STATUS,enabled=%d,state=%s,vpack=%s,i=%s,temp=%s,cooldown_ms=%d,pulse_ms=%d" %
                       (1 if system_enabled else 0, state, v_str, i_str, t_str,
                        max(0, time.ticks_diff(next_weld_ok_at, time.ticks_ms())), PULSE_MS))
            if cells:
                V1, V2, V3, C1, C2, C3 = cells
                print_both("CELLS,V1=%.3f,V2=%.3f,V3=%.3f,C1=%.3f,C2=%.3f,C3=%.3f" %
                           (V1, V2, V3, C1, C2, C3))
                if not (V3 >= V2 >= V1 >= 0):
                    print_both("WARN: Vpack >= V2 >= V1 violated — check VBUS wiring/grounds.")
            else:
                print_both("CELLS,NONE")

        # COMMAND POLL from USB-CDC
        line_usb = usb_try_read_line()
        if line_usb is not None:
            cmd_handle(line_usb)

        # COMMAND POLL from UART1
        for _ in range(4):
            line_uart = uart_try_read_line()
            if line_uart is None:
                break
            cmd_handle(line_uart)


        # COMMAND POLL from TCP
        if tcp_server:
            for _ in range(4):
                line_tcp = tcp_server.try_read_line()
                if line_tcp is None:
                    break
                cmd_handle(line_tcp)
        time.sleep(0.05)

except KeyboardInterrupt:
    FET_CHARGE.off()
    led_off()
    print_both("\nManual stop — charger OFF")
