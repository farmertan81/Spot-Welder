import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.ads1x15 import Mode

class ADS1115Driver:
    def __init__(self):
        # I2C setup
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(i2c, address=0x4A)
        
        # Set gain to ±4.096V range
        self.ads.gain = 1
        
        # Set to CONTINUOUS mode at max speed (860 SPS)
        self.ads.mode = Mode.CONTINUOUS
        self.ads.data_rate = 860
        
        # Scaling constants
        self.Rsh = 50e-6  # 50 µΩ shunt
        self.G_AMC = 1.0  # AMC1311B gain
        self.scale_factor = 1.0 / (self.G_AMC * self.Rsh)
        
        # Voltage per LSB at gain=1
        self.volts_per_bit = 4.096 / 32768
        
        print(f"ADS1115: Scale factor = {self.scale_factor:.6e} A/V")
        print(f"ADS1115: Max sample rate = 860 SPS")
        
        # Measure offset
        print("ADS1115: Measuring offset...")
        import time
        offset_samples = []
        for _ in range(100):
            v0 = self.ads.read(0) * self.volts_per_bit
            v1 = self.ads.read(1) * self.volts_per_bit
            offset_samples.append(v1 - v0)
            time.sleep(0.001)
        
        self.offset_voltage = sum(offset_samples) / len(offset_samples)
        self.offset_current = self.offset_voltage * self.scale_factor
        print(f"ADS1115: Offset = {self.offset_voltage:.6f} V ({self.offset_current:.2f} A)")
        print("ADS1115: Initialized successfully")
        self.initialized = True
    
    def read_current(self):
        """Read current in Amps"""
        v0 = self.ads.read(0) * self.volts_per_bit
        v1 = self.ads.read(1) * self.volts_per_bit
        voltage = v1 - v0
        current = (voltage - self.offset_voltage) * self.scale_factor
        return current
    
    def read_raw(self):
        """Read raw voltage for debugging"""
        v0 = self.ads.read(0) * self.volts_per_bit
        v1 = self.ads.read(1) * self.volts_per_bit
        return v1 - v0

# Singleton instance
_adc = None

def get_adc():
    """Get or create ADS1115 singleton"""
    global _adc
    if _adc is None:
        _adc = ADS1115Driver()
    return _adc
