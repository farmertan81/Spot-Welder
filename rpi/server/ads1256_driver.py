#!/usr/bin/env python3
"""
ADS1256 Driver for Raspberry Pi
Differential AIN2-AIN3 input for AMC1311B current sense
Replaces ADS1263 driver
"""
import time
import spidev
import RPi.GPIO as GPIO

# BCM pins (Waveshare HAT compatible)
PIN_DRDY = 22
PIN_RST = 18
PIN_CS = 24

# ADS1256 commands
CMD_WAKEUP = 0x00
CMD_RDATA = 0x01
CMD_RDATAC = 0x03
CMD_SDATAC = 0x0F
CMD_RREG = 0x10
CMD_WREG = 0x50
CMD_SELFCAL = 0xF0
CMD_SYNC = 0xFC
CMD_STANDBY = 0xFD
CMD_RESET = 0xFE

# Registers
REG_STATUS = 0x00
REG_MUX = 0x01
REG_ADCON = 0x02
REG_DRATE = 0x03

# Data rates
DRATE_30000SPS = 0xF0
DRATE_15000SPS = 0xE0
DRATE_7500SPS = 0xD0
DRATE_3750SPS = 0xC0
DRATE_2000SPS = 0xB0
DRATE_1000SPS = 0xA1
DRATE_500SPS = 0x92
DRATE_100SPS = 0x82
DRATE_60SPS = 0x72
DRATE_50SPS = 0x63
DRATE_30SPS = 0x53
DRATE_25SPS = 0x43
DRATE_15SPS = 0x33
DRATE_10SPS = 0x23
DRATE_5SPS = 0x13
DRATE_2_5SPS = 0x03

# MUX settings for differential inputs
MUX_AIN2_AIN3 = 0x23  # P=AIN2, N=AIN3

class ADS1256:
    def __init__(self):
        self.spi = None
        self.initialized = False
        self.offset_voltage = 0.0  # Baseline voltage at zero current
        
        # Scaling constants
        self.Vref = 2.5  # Internal reference voltage
        self.gain_adc = 1.0  # PGA gain (1, 2, 4, 8, 16, 32, 64)
        self.FS = 8388607.0  # 24-bit signed max (2^23 - 1)
        self.Rsh = 50e-6  # 50 µΩ shunt
        self.G_AMC = 1.0  # AMC1311B gain is 1 V/V
        
        # Voltage per code
        self.volts_per_code = (2 * self.Vref) / (self.FS * 2 * self.gain_adc)
        
        # Current scale factor: I(A) = (V - V_offset) / Rsh
        print(f"ADS1256: Volts per code = {self.volts_per_code:.9e}")
        print(f"ADS1256: Shunt = {self.Rsh*1e6:.1f} µΩ")
    
    def init(self):
        """Initialize GPIO and SPI"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(PIN_DRDY, GPIO.IN, pull_up_down=GPIO.PUD_UP)            # GPIO.setup(PIN_RST, GPIO.OUT)  # PDWN tied to 5V
            GPIO.setup(PIN_CS, GPIO.OUT, initial=1)
            
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz = 1000000  # 1 MHz
            self.spi.mode = 0b01  # CPOL=0, CPHA=1
            self.spi.bits_per_word = 8
            
            # Hardware reset            # self._hw_reset()  # PDWN tied to 5V
            
            # Send RESET command
            self._send_cmd(CMD_RESET)
            time.sleep(0.1)
            
            # Stop continuous mode
            self._send_cmd(CMD_SDATAC)
            time.sleep(0.001)
            
            # Configure registers
            # STATUS: Auto-cal off, buffer enabled, order MSB
            self._wr_reg(REG_STATUS, 0x06)
            
            # MUX: AIN2-AIN3 differential
            self._wr_reg(REG_MUX, MUX_AIN2_AIN3)
            
            # ADCON: PGA=1, data rate control
            self._wr_reg(REG_ADCON, 0x00)  # PGA=1, CLK=internal
            
            # DRATE: 30 kSPS for fast weld capture
            self._wr_reg(REG_DRATE, DRATE_30000SPS)
            
            # Perform self-calibration
            print("ADS1256: Performing self-calibration...")
            self._send_cmd(CMD_SELFCAL)
            self._wait_drdy(timeout=2.0)
            
            # Measure baseline offset
            self._measure_offset()
            
            self.initialized = True
            print("ADS1256: Initialized successfully")
            return True
            
        except Exception as e:
            print(f"ADS1256: Init failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _cs_low(self):
        GPIO.output(PIN_CS, 0)
    
    def _cs_high(self):
        GPIO.output(PIN_CS, 1)
    
    def _send_cmd(self, cmd):
        self._cs_low()
        self.spi.xfer2([cmd])
        self._cs_high()
        time.sleep(0.001)
    
    def _wr_reg(self, addr, data):
        self._cs_low()
        self.spi.xfer2([CMD_WREG | (addr & 0x0F), 0x00, data & 0xFF])
        self._cs_high()
        time.sleep(0.001)
    
    def _rd_reg(self, addr):
        self._cs_low()
        resp = self.spi.xfer2([CMD_RREG | (addr & 0x0F), 0x00, 0x00])
        self._cs_high()
        return resp[2]
    
    def _wait_drdy(self, timeout=1.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if GPIO.input(PIN_DRDY) == 0:
                return True
            time.sleep(0.0001)
        return False
    
    def _hw_reset(self):        # GPIO.output(PIN_RST, 0)
        time.sleep(0.01)        # GPIO.output(PIN_RST, 1)
        time.sleep(0.1)
    
    def _read_counts(self):
        """Read 24-bit signed ADC value"""
        if not self._wait_drdy(0.1):
            return None
        
        self._cs_low()
        self.spi.xfer2([CMD_RDATA])
        time.sleep(0.00001)  # t6 delay
        resp = self.spi.xfer2([0x00, 0x00, 0x00])
        self._cs_high()
        
        raw24 = (resp[0] << 16) | (resp[1] << 8) | resp[2]
        if raw24 & 0x800000:
            raw = raw24 - (1 << 24)
        else:
            raw = raw24
        return raw
    
    def _read_voltage(self):
        """Read voltage from ADC"""
        counts = self._read_counts()
        if counts is None:
            return float('nan')
        return counts * self.volts_per_code
    
    def _measure_offset(self, n_samples=100):
        """Measure idle offset voltage"""
        print("ADS1256: Measuring offset...")
        voltages = []
        
        for _ in range(n_samples):
            v = self._read_voltage()
            if not (v != v):  # Check for NaN
                voltages.append(v)
        
        if voltages:
            self.offset_voltage = sum(voltages) / len(voltages)
            print(f"ADS1256: Offset = {self.offset_voltage:.6f} V ({len(voltages)} samples)")
        else:
            print("ADS1256: WARNING - Could not measure offset!")
            self.offset_voltage = 0.0
    
    def read_current(self):
        """Read single current sample (blocking)"""
        if not self.initialized:
            return float('nan')
        
        v = self._read_voltage()
        if v != v:  # NaN check
            return float('nan')
        
        # Subtract offset and convert to current
        v_shunt = v - self.offset_voltage
        I = -v_shunt / self.Rsh  # Inverted polarity
        return I
    
    def capture_waveform(self, duration_ms=30, pre_trigger_ms=5):
        """
        Capture waveform for specified duration
        Returns: (timestamps_us, currents_A)
        """
        if not self.initialized:
            return [], []
        
        timestamps = []
        currents = []
        
        t_start = time.time()
        t_end = t_start + (duration_ms / 1000.0)
        
        while time.time() < t_end:
            v = self._read_voltage()
            if v != v:  # Skip NaN
                continue
            
            t_sample = time.time()
            
            # Convert to current
            v_shunt = v - self.offset_voltage
            I = -v_shunt / self.Rsh  # Inverted polarity
            
            # Timestamp in microseconds relative to start
            t_us = int((t_sample - t_start) * 1e6)
            
            timestamps.append(t_us)
            currents.append(I)
        
        print(f"ADS1256: Captured {len(currents)} samples in {duration_ms} ms")
        return timestamps, currents
    
    def cleanup(self):
        """Cleanup GPIO and SPI"""
        try:
            if self.spi:
                self._send_cmd(CMD_STANDBY)
                self.spi.close()
            GPIO.cleanup()
        except Exception as e:
            print(f"ADS1256: Cleanup error: {e}")


# Singleton instance
_adc = None

def get_adc():
    """Get or create ADS1256 singleton"""
    global _adc
    if _adc is None:
        _adc = ADS1256()
        _adc.init()
    return _adc
