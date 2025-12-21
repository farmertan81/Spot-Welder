#!/usr/bin/env python3
"""
ADS1263 Driver for Raspberry Pi
Single-ended IN6 input for AMC1311B current sense
"""
import time
import spidev
import RPi.GPIO as GPIO
from collections import deque

# BCM pins per Waveshare HAT
PIN_DRDY = 17
PIN_RST = 18
PIN_CS = 22

# ADS1263 commands
CMD_RESET = 0x06
CMD_START1 = 0x08
CMD_STOP1 = 0x0A
CMD_RREG = 0x20
CMD_WREG = 0x40
CMD_RDATA1 = 0x12

# Registers
REG_ID = 0x00
REG_POWER = 0x01
REG_INTERFACE = 0x02
REG_MODE0 = 0x03
REG_MODE1 = 0x04
REG_MODE2 = 0x05
# REG_MODE3 removed (not used)
# REG_REF removed (not used)
REG_REFMUX = 0x0F
REG_INPMUX = 0x06
# REG_PGA removed (not used)

class ADS1263:
    def __init__(self):
        self.spi = None
        self.initialized = False
        self.offset_counts = 0.0
        
        # Scaling constants
        self.Vref = 5.0  # VDD reference (REFMUX = 0x24)
        self.gain_adc = 1.0
        self.FS = 8388607.0  # 24-bit signed max
        self.Rsh = 50e-6  # 50 µΩ
        self.G_AMC = 1.0  # AMC1311B gain
        
        # Quick scale factor: I(A) = code × scale_factor
        # scale_factor = (Vref / FS) / G_AMC / Rsh
        self.scale_factor = self.Vref / (self.FS * self.G_AMC * self.Rsh)
        print(f"ADS1263: Scale factor = {self.scale_factor:.6e} A/code")
        
    def init(self):
        """Initialize GPIO and SPI"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(PIN_DRDY, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(PIN_RST, GPIO.OUT)
            GPIO.setup(PIN_CS, GPIO.OUT, initial=1)
            
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz = 500000
            self.spi.mode = 0b01
            self.spi.bits_per_word = 8
            
            self._hw_reset()
            self._send_cmd(CMD_RESET)
            time.sleep(0.005)
            
            chip_id = self._rd_reg(REG_ID)
            print(f"ADS1263: Chip ID = 0x{chip_id:02X}")
            
            # Configure for fast single-ended capture
            # MODE0: 38.4 kSPS (0x00 for fastest, adjust if needed)
            self._wr_reg(REG_POWER, 0x11)  # Enable internal reference
            self._wr_reg(REG_MODE0, 0x04)  # DELAY=35us for stability
            
            # MODE1: Enable input buffer
            self._wr_reg(REG_MODE1, 0x84)  # FIR filter
            
            self._wr_reg(REG_MODE2, 0x8F)  # PGA bypassed
            
            # VDD/VSS as reference (5V)
            self._wr_reg(REG_REFMUX, 0x24)
            
            # Single-ended IN6 (P=IN6, N=AINCOM)
            self._wr_reg(REG_INPMUX, (0x0 << 4) | 0x1)  # IN0-IN1 differential
            
            # PGA disabled (gain=1)
            
            # Read back config
            m0 = self._rd_reg(REG_MODE0)
            im = self._rd_reg(REG_INPMUX)
            print(f"ADS1263: MODE0=0x{m0:02X} INPMUX=0x{im:02X}")
            
            # Start conversions
            self._send_cmd(CMD_START1)
            time.sleep(0.002)
            
            # Measure offset
            self._measure_offset()
            
            self.initialized = True
            print("ADS1263: Initialized successfully")
            return True
            
        except Exception as e:
            print(f"ADS1263: Init failed: {e}")
            return False
    
    def _cs_low(self):
        GPIO.output(PIN_CS, 0)
    
    def _cs_high(self):
        GPIO.output(PIN_CS, 1)
    
    def _send_cmd(self, cmd):
        self._cs_low()
        self.spi.xfer2([cmd])
        self._cs_high()
    
    def _wr_reg(self, addr, data):
        self._cs_low()
        self.spi.xfer2([CMD_WREG | (addr & 0x1F), 0x00, data & 0xFF])
        self._cs_high()
        time.sleep(0.0002)
    
    def _rd_reg(self, addr):
        self._cs_low()
        self.spi.xfer2([CMD_RREG | (addr & 0x1F), 0x00])
        val = self.spi.readbytes(1)[0]
        self._cs_high()
        return val
    
    def _wait_drdy(self, timeout=1.0):
        t0 = time.time()
        checks = 0
        while time.time() - t0 < timeout:
            checks += 1
            if GPIO.input(PIN_DRDY) == 0:
                return True
            time.sleep(0.0001)
        print(f"ADS1263: DRDY timeout after {checks} checks, DRDY={GPIO.input(PIN_DRDY)}")
        return False

    def _hw_reset(self):
        GPIO.output(PIN_RST, 1)
        time.sleep(0.001)
        GPIO.output(PIN_RST, 0)
        time.sleep(0.002)
        GPIO.output(PIN_RST, 1)
        time.sleep(0.003)
    
    def _read_counts(self):
        """Read 24-bit signed ADC value"""
        self._cs_low()
        resp = self.spi.xfer2([CMD_RDATA1, 0, 0, 0, 0])
        self._cs_high()
        
        raw24 = (resp[1] << 16) | (resp[2] << 8) | resp[3]
        if raw24 & 0x800000:
            raw = raw24 - (1 << 24)
        else:
            raw = raw24
        # Debug: check status and checksum
        status = resp[0]
        checksum = resp[4]
        if status != 0:
#             print(f"ADS1263: Status byte = 0x{status:02x}")
        # Debug: check status and checksum
            pass
        status = resp[0]
        checksum = resp[4]
        if status != 0:
#             print(f"ADS1263: Status byte = 0x{status:02x}")
            pass
        return raw
    
    def _measure_offset(self, n_samples=50):
        """Measure idle offset"""
        print("ADS1263: Waiting for AMC1311 to settle...")
        time.sleep(2.0)  # Let AMC1311 stabilize
        print("ADS1263: Measuring offset...")
        # Discard first 5 samples
        for _ in range(5):
            if self._wait_drdy(0.5):
                self._read_counts()
        acc = 0
        got = 0
        for _ in range(n_samples):
            if not self._wait_drdy(0.5):
                continue
            acc += self._read_counts()
            got += 1
        
        self.offset_counts = acc / max(1, got)
        print(f"ADS1263: Offset = {self.offset_counts:.1f} counts ({got} samples)")
    
    def read_current(self):
        """Read single current sample (blocking)"""
        if not self.initialized:
            return float('nan')
        
        if not self._wait_drdy(0.02):
            return float('nan')
        
        raw = self._read_counts()
        raw_corr = raw - self.offset_counts
        
        # Convert to amps
        I = raw_corr * self.scale_factor
        # Debug (idle): comment left in place; keep quiet here to reduce spam
        # print(f"ADC: counts={int(raw_corr)} I={I:.3f} A")
        return I
    
    def capture_waveform(self, duration_ms=30, pre_trigger_ms=5):
        """
        Capture waveform for specified duration
        Returns: (timestamps_us, currents_A)
        """
        if not self.initialized:
            return [], []
        
        # Estimate samples (38.4 kSPS = 38.4 samples/ms)
        est_samples = int(duration_ms * 40)  # slight overestimate
        
        timestamps = []
        currents = []
        
        t_start = time.time()
        t_end = t_start + (duration_ms / 1000.0)
        
        while time.time() < t_end:
            if not self._wait_drdy(0.001):
                continue
            
            t_sample = time.time()
            raw = self._read_counts()
            raw_corr = raw - self.offset_counts
            I = raw_corr * self.scale_factor
            
            # Timestamp in microseconds relative to start
            t_us = int((t_sample - t_start) * 1e6)
            
            timestamps.append(t_us)
            currents.append(I)
            
            if len(timestamps) >= est_samples * 2:
                break  # safety limit
        
        print(f"ADS1263: Captured {len(currents)} samples in {duration_ms} ms")
        return timestamps, currents
    
    def cleanup(self):
        """Cleanup GPIO and SPI"""
        try:
            if self.spi:
                self._send_cmd(CMD_STOP1)
                self.spi.close()
            GPIO.cleanup()
        except Exception as e:
            print(f"ADS1263: Cleanup error: {e}")


# Singleton instance
_adc = None

def get_adc():
    """Get or create ADS1263 singleton"""
    global _adc
    if _adc is None:
        _adc = ADS1263()
        _adc.init()
    return _adc