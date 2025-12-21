
# ===== Cap Voltage Sensing (AMC1311 + Divider) =====
V_CAP_CH = 0  # AIN0–AIN1 (cap voltage via AMC1311 + 68k/10k divider)

# Divider scaling (measured values)
DIVIDER_TOP = 67_200.0      # 67.2k (measured)
DIVIDER_BOTTOM = 9_940.0    # 9.94k (measured)
DIVIDER_SCALE = (DIVIDER_TOP + DIVIDER_BOTTOM) / DIVIDER_BOTTOM  # ~7.76

def counts_to_cap_volts(counts: int, vref: float = 2.5) -> float:
    """Convert ADS1256 counts to cap voltage (accounting for divider)"""
    # Convert counts to ADC voltage
    v_adc = (counts / 8388607.0) * vref  # 24-bit signed, ±VREF
    # Scale by divider ratio
    return v_adc * DIVIDER_SCALE

# ===== Cap Voltage Sensing (AMC1311 + Divider) =====
V_CAP_CH = 0  # AIN0–AIN1 (cap voltage via AMC1311 + 68k/10k divider)

# Divider scaling (measured values)
DIVIDER_TOP = 67_200.0      # 67.2k (measured)
DIVIDER_BOTTOM = 9_940.0    # 9.94k (measured)
DIVIDER_SCALE = (DIVIDER_TOP + DIVIDER_BOTTOM) / DIVIDER_BOTTOM  # ~7.76

def counts_to_cap_volts(counts: int, vref: float = 2.5) -> float:
    """Convert ADS1256 counts to cap voltage (accounting for divider)"""
    # Convert counts to ADC voltage
    v_adc = (counts / 8388607.0) * vref  # 24-bit signed, ±VREF
    # Scale by divider ratio
    return v_adc * DIVIDER_SCALE
