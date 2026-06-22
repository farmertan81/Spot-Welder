"""
PlatformIO extra script: Auto-copy and rename STM32 firmware.bin

This script runs POST-build and copies the compiled firmware from:
    .pio/build/<env>/firmware.bin
to the project root as:
    stm32_firmware.bin

This allows the ESP32-P4 to flash the STM32 from SD card without manual renaming.
The ESP32 looks for /stm32_firmware.bin on the SD card root.

Usage: Add this line to platformio.ini under [env:xxx]:
    extra_scripts = post:extra_script.py
"""

Import("env")
import shutil
import os

def post_build(source, target, env):
    """
    Called after successful build. Copies firmware.bin to project root as stm32_firmware.bin.
    """
    # Source: .pio/build/<env>/firmware.bin
    firmware_bin = str(target[0])
    
    # Destination: project_root/stm32_firmware.bin
    project_dir = env.get("PROJECT_DIR")
    dest_file = os.path.join(project_dir, "stm32_firmware.bin")
    
    try:
        shutil.copy2(firmware_bin, dest_file)
        print(f"✓ Copied firmware to: {dest_file}")
        
        # Show size for verification
        size_bytes = os.path.getsize(dest_file)
        size_kb = size_bytes / 1024.0
        print(f"  Size: {size_bytes} bytes ({size_kb:.2f} KB)")
        
    except Exception as e:
        print(f"✗ Failed to copy firmware: {e}")

# Register the post-build callback
env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", post_build)
