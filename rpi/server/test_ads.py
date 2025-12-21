from ads1115_driver import get_adc
import time

adc = get_adc()
print("\nReading idle current for 5 seconds...")
for i in range(10):
    raw_v = adc.read_raw()
    current = adc.read_current()
    print(f"Raw voltage: {raw_v:.6f} V, Current: {current:.2f} A")
    time.sleep(0.5)
