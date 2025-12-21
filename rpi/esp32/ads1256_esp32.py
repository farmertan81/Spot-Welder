# ADS1256 Driver for ESP32 MicroPython
# Ported from working Pi driver
# Differential AIN2-AIN3 for AMC1311B current sense

from machine import Pin, SPI
import time

# ADS1256 Commands
CMD_WAKEUP   = 0x00
CMD_RDATA    = 0x01
CMD_RDATAC   = 0x03
CMD_SDATAC   = 0x0F
CMD_RREG     = 0x10
CMD_WREG     = 0x50
CMD_SELFCAL  = 0xF0
CMD_SYNC     = 0xFC
CMD_RESET    = 0xFE

# ADS1256 Registers
REG_STATUS   = 0x00
REG_MUX      = 0x01
REG_ADCON    = 0x02
REG_DRATE    = 0x03

# Data rates
DRATE_30000SPS = 0xC0
DRATE_15000SPS = 0xE0
DRATE_7500SPS  = 0xD0
DRATE_3750SPS  = 0xC0
DRATE_2000SPS  = 0xB0
DRATE_1000SPS  = 0xA1

# MUX settings
MUX_AIN2_AIN3 = 0x23  # P=AIN2, N=AIN3 (your shunt via AMC1311)

class ADS1256:
    def __init__(self, spi, cs_pin, drdy_pin):
        self.spi = spi
        self.cs = Pin(cs_pin, Pin.OUT, value=1)
        self.drdy = Pin(drdy_pin, Pin.IN, Pin.PULL_UP)
        
        # Scaling constants (from your Pi driver)
        self.Vref = 2.5  # Internal reference
        self.gain_adc = 16.0  # PGA gain = 16
        self.FS = 8388607.0  # 24-bit signed max
        self.Rsh = 50e-6  # 50 µΩ shunt
        self.G_AMC = 1.0  # AMC1311B gain
        
        # Voltage per code
        self.volts_per_code = (2 * self.Vref) / (self.FS * 2 * self.gain_adc)
        
        self.offset_voltage = 0.0  # Baseline at zero current
        self.initialized = False
        
        print(f"ADS1256: Volts per code = {self.volts_per_code:.9e}")
        print(f"ADS1256: Shunt = {self.Rsh*1e6:.1f} µΩ")
        
        # Initialize
        time.sleep_ms(100)
        self.reset()
        self.configure()
        
    def wait_drdy(self, timeout_ms=1000):
        """Wait for DRDY to go low (data ready)"""
        start = time.ticks_ms()
        while self.drdy.value() == 1:
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                return False
        return True
    
    def reset(self):
        """Software reset"""
        self.cs.value(0)
        self.spi.write(bytes([CMD_RESET]))
        self.cs.value(1)
        time.sleep_ms(100)
        
        # Stop continuous mode
        self.cs.value(0)
        self.spi.write(bytes([CMD_SDATAC]))
        self.cs.value(1)
        time.sleep_ms(1)
        
    def write_reg(self, reg, value):
        """Write to a register"""
        self.cs.value(0)
        self.spi.write(bytes([CMD_WREG | (reg & 0x0F), 0x00, value & 0xFF]))
        self.cs.value(1)
        time.sleep_ms(1)
        
    def read_reg(self, reg):
        """Read from a register"""
        self.cs.value(0)
        self.spi.write(bytes([CMD_RREG | (reg & 0x0F), 0x00]))
        result = self.spi.read(1)
        self.cs.value(1)
        return result[0]
    
    def configure(self):
        """Configure ADS1256 with proven Pi settings"""
        # STATUS: Auto-cal off, buffer enabled, order MSB
        self.write_reg(REG_STATUS, 0x06)
        
        # MUX: AIN2-AIN3 differential (your shunt)
        self.write_reg(REG_MUX, MUX_AIN2_AIN3)
        
        # ADCON: PGA=1, CLK=internal
        self.write_reg(REG_ADCON, 0x05)  # PGA=32
        
        # DRATE: 2 kSPS (realistic for SPI speed + interrupt overhead)
        self.write_reg(REG_DRATE, DRATE_30000SPS)
        
        # Perform self-calibration
        print("ADS1256: Calibrating...")
        self.cs.value(0)
        self.spi.write(bytes([CMD_SELFCAL]))
        self.cs.value(1)
        self.wait_drdy(timeout_ms=2000)
        print("ADS1256: Calibration done")
        
        # Measure offset
        self.measure_offset()
        
        self.initialized = True
        
    def measure_offset(self, samples=50):
        """Measure baseline voltage offset at zero current"""
        print("ADS1256: Measuring offset...")
        voltages = []
        for _ in range(samples):
            v = self.read_voltage_raw()
            if v is not None:
                voltages.append(v)
        
        if voltages:
            self.offset_voltage = sum(voltages) / len(voltages)
            print(f"ADS1256: Offset = {self.offset_voltage:.6f} V")
        
    def read_counts(self):
        """Read 24-bit signed ADC value"""
        if not self.wait_drdy():
            return None
            
        self.cs.value(0)
        self.spi.write(bytes([CMD_RDATA]))
        time.sleep_us(7)  # t6 delay
        
        data = self.spi.read(3)
        self.cs.value(1)
        
        # Convert 24-bit two's complement to signed integer
        raw = (data[0] << 16) | (data[1] << 8) | data[2]
        if raw & 0x800000:
            raw -= 0x1000000
        
        return raw
    
    def read_voltage_raw(self):
        """Read raw voltage (no offset correction)"""
        counts = self.read_counts()
        if counts is None:
            return None
        
        voltage = counts * self.volts_per_code
        return voltage
    
    def read_voltage(self):
        """Read voltage with offset correction"""
        v = self.read_voltage_raw()
        if v is None:
            return None
        return v - self.offset_voltage
    
    def start_continuous(self):
        """Start continuous conversion mode"""
        self.cs.value(0)
        self.spi.write(bytes([CMD_RDATAC]))
        self.cs.value(1)
        time.sleep_us(10)

    def stop_continuous(self):
        """Stop continuous conversion mode"""
        self.cs.value(0)
        self.spi.write(bytes([CMD_SDATAC]))
        self.cs.value(1)
        time.sleep_us(10)

    def read_counts_fast(self):
        """Read 24-bit value in continuous mode (no DRDY wait, non-blocking)"""
        # Check if data is ready (non-blocking)
        if self.drdy.value() == 1:
            return None  # Not ready yet

        # Data is ready, read it immediately
        self.cs.value(0)
        data = self.spi.read(3)
        self.cs.value(1)

        # Convert 24-bit two's complement to signed integer
        raw = (data[0] << 16) | (data[1] << 8) | data[2]
        if raw & 0x800000:
            raw -= 0x1000000

        return raw

    def read_current_fast(self):
        """Read current in continuous mode (non-blocking)"""
        counts = self.read_counts_fast()
        if counts is None:
            return None

        voltage = counts * self.volts_per_code
        voltage_corrected = voltage - self.offset_voltage
        current = -voltage_corrected / self.Rsh
        return current

    def read_current(self):
        """Read current in Amps: I = V / Rsh"""
        v = self.read_voltage()
        if v is None:
            return None
        
        current = -v / self.Rsh
        return current


    def read_counts_isr(self):
        """Read 24-bit value in ISR - assumes DRDY triggered, no check"""
        # Data is ready (guaranteed by ISR trigger), read immediately
        self.cs.value(0)
        data = self.spi.read(3)
        self.cs.value(1)

        # Convert 24-bit two's complement to signed integer
        raw = (data[0] << 16) | (data[1] << 8) | data[2]
        if raw & 0x800000:
            raw -= 0x1000000

        return raw

    def read_current_isr(self):
        """Read current in ISR mode (no DRDY check)"""
        counts = self.read_counts_isr()
        voltage = counts * self.volts_per_code
        voltage_corrected = voltage - self.offset_voltage
        current = -voltage_corrected / self.Rsh
        return current

    def read_current_fast(self):
        """Fast polling read - check DRDY and read immediately"""
        if self.drdy.value() == 0:  # Data ready
            self.cs.value(0)
            data = self.spi.read(3)
            self.cs.value(1)
            
            raw = (data[0] << 16) | (data[1] << 8) | data[2]
            if raw & 0x800000:
                raw -= 0x1000000
            
            voltage = raw * self.volts_per_code - self.offset_voltage
            return -voltage / self.Rsh
        return None
