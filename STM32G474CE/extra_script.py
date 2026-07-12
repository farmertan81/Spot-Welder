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
    Called after the RAW .bin is produced. Copies it to project root as
    stm32_firmware.bin.

    IMPORTANT: We must copy the RAW binary (firmware.bin), NOT the ELF.
    Katapult flashes a raw image straight to app_start (0x08002000). If an ELF
    (with its headers/symbol tables) is flashed instead, the wrong bytes land at
    the vector table and the STM32 gets stuck in the bootloader (bricked until
    re-flashed with a debugger). So we explicitly resolve the .bin path from
    $BUILD_DIR/$PROGNAME.bin instead of trusting target[0].
    """
    build_dir = env.subst("$BUILD_DIR")
    progname  = env.subst("$PROGNAME")           # normally "firmware"
    firmware_bin = os.path.join(build_dir, progname + ".bin")

    # Destination: project_root/stm32_firmware.bin
    project_dir = env.subst("$PROJECT_DIR")
    dest_file = os.path.join(project_dir, "stm32_firmware.bin")

    # Safety: make sure we are copying a real raw binary, never an ELF.
    if not os.path.isfile(firmware_bin):
        print(f"✗ extra_script: raw binary not found at {firmware_bin} — "
              f"stm32_firmware.bin NOT updated")
        return
    with open(firmware_bin, "rb") as fh:
        head = fh.read(4)
    if head[:4] == b"\x7fELF":
        print(f"✗ extra_script: {firmware_bin} is an ELF, refusing to copy "
              f"(would brick the STM32 via Katapult)")
        return

    try:
        shutil.copy2(firmware_bin, dest_file)
        size_bytes = os.path.getsize(dest_file)
        print(f"✓ Copied RAW firmware to: {dest_file}")
        print(f"  Source: {firmware_bin}")
        print(f"  Size: {size_bytes} bytes ({size_bytes / 1024.0:.2f} KB)")
    except Exception as e:
        print(f"✗ Failed to copy firmware: {e}")

# Register the post-build callback ON THE RAW .bin (NOT the .elf) so it only
# fires once the raw binary exists, and target/order can't feed us the ELF.
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", post_build)
