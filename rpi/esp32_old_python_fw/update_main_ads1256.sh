#!/bin/bash
cd ~/weldctl/esp32

echo "=== Updating main.py with ADS1256 integration ==="

# Backup
cp main.py main.py.pre_ads1256

# Insert ADS1256 import after line 13
sed -i '13a\\n# ---- ADS1256 Current Sensor Import ----\ntry:\n    import ads1256_esp32\n    from machine import SPI\n    ADS1256_AVAILABLE = True\nexcept:\n    ADS1256_AVAILABLE = False\n    print("ADS1256 driver not found - current measurement disabled")' main.py

# Insert ADS1256 initialization after line 100 (accounting for added lines)
sed -i '108a\\n# ---- ADS1256 Current Sensor ----\nif ADS1256_AVAILABLE:\n    try:\n        spi_ads = SPI(1, baudrate=1000000, polarity=0, phase=1,\n                      sck=Pin(40), mosi=Pin(39), miso=Pin(38))\n        adc_current = ads1256_esp32.ADS1256(spi_ads, cs_pin=17, drdy_pin=18)\n        print("✓ ADS1256 current sensor initialized")\n    except Exception as e:\n        print(f"ADS1256 init failed: {e}")\n        ADS1256_AVAILABLE = False\n        adc_current = None\nelse:\n    adc_current = None\n\nweld_samples = []\nMAX_WELD_SAMPLES = 1000' main.py

echo "✓ Added ADS1256 initialization"
echo ""
echo "Now manually replace the do_weld_ms function using the new version"
echo "The new function is in: weld_function_new.txt"

