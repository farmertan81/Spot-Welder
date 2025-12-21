# Add these constants after the existing register definitions (around line 30)
REG_INPMUX2 = 0x07  # ADC2 input multiplexer
CMD_START2 = 0x0C   # Start ADC2
CMD_STOP2 = 0x0E    # Stop ADC2
CMD_RDATA2 = 0x14   # Read ADC2 data

# Add this method to the ADS1263 class (after read_current method)
def read_voltage(self):
    """Read voltage from ADC2 (AIN0 single-ended)"""
    if not self.initialized:
        return float('nan')
    
    try:
        # Read ADC2 data (24-bit)
        self._cs_low()
        self._send_cmd(CMD_RDATA2)
        time.sleep(0.0001)
        
        buf = self.spi.readbytes(4)  # 24-bit data + checksum
        self._cs_high()
        
        # Convert to signed 24-bit
        adc_code = (buf[0] << 16) | (buf[1] << 8) | buf[2]
        if adc_code & 0x800000:
            adc_code -= 0x1000000
        
        # Convert to voltage
        # Using 5V reference (AVDD-AVSS), 24-bit ADC2
        FS_ADC2 = 8388607.0  # 2^23 - 1
        voltage = (adc_code / FS_ADC2) * 5.0
        
        return voltage
        
    except Exception as e:
        print(f"ADS1263: Voltage read error: {e}")
        return float('nan')

# Add ADC2 initialization in the init() method (after ADC1 config, around line 100)
def _init_adc2(self):
    """Initialize ADC2 for voltage measurement"""
    # Configure ADC2 input: AIN0 single-ended (P=AIN0, N=AINCOM)
    self._wr_reg(REG_INPMUX2, (0x0 << 4) | 0xA)  # AIN0-AINCOM
    
    # Start ADC2 conversions
    self._send_cmd(CMD_START2)
    time.sleep(0.002)
    
    print("ADS1263: ADC2 initialized for voltage (AIN0)")
