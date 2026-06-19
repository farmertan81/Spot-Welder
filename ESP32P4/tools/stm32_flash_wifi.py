#!/usr/bin/env python3
# ============================================================
#  stm32_flash_wifi.py — flash the STM32 weld controller over WiFi
# ============================================================
# Sends a freshly built STM32G474 firmware image wirelessly to the ESP32-P4,
# which then drives the STM32 into its ROM bootloader and programs it over the
# existing UART link (AN3155). This mirrors the OLD board's espota + STM32-flash
# workflow, but for the STM32 instead of the ESP32.
#
# The STM32 firmware is built by PlatformIO, so its .bin lives under
#   STM32G474CE/.pio/build/<env>/firmware.bin
# The <env> name depends on your platformio.ini; pass --bin to point at it
# directly, or let this script auto-discover the most recent firmware.bin.
#
# USAGE (from the ESP32P4 project folder):
#   python tools/stm32_flash_wifi.py                       # auto-find bin + default hosts
#   python tools/stm32_flash_wifi.py 192.168.1.77          # override host
#   python tools/stm32_flash_wifi.py --bin C:\path\firmware.bin
#   python tools/stm32_flash_wifi.py spotwelder.local --bin ..\STM32G474CE\.pio\build\genericSTM32G474CE\firmware.bin
#
# It is normally launched by the VS Code task "Flash STM32 over WiFi"
# (see .vscode/tasks.json).
#
# Only the Python standard library is used (works with ESP-IDF's bundled Python).

import glob
import os
import sys
import time
import urllib.request
import urllib.error

# ---- Defaults (edit to taste) -------------------------------------------
DEFAULT_HOSTS = ["spotwelder.local", "192.168.1.77"]  # tried in order
TIMEOUT_S     = 180   # mass-erase + write + reboot can take a while
# -------------------------------------------------------------------------


def project_root():
    # tools/stm32_flash_wifi.py -> ESP32P4 project root is one level up.
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def repo_root():
    # ESP32P4 sits inside the Spot-Welder repo; STM32G474CE is a sibling.
    return os.path.dirname(project_root())


def find_firmware():
    # 1) Explicit --bin <path>
    for i, a in enumerate(sys.argv):
        if a == "--bin" and i + 1 < len(sys.argv):
            fw = sys.argv[i + 1]
            if not os.path.isfile(fw):
                print("ERROR: --bin path not found:\n  %s" % fw)
                sys.exit(1)
            return fw

    # 2) Auto-discover the newest firmware.bin under STM32G474CE/.pio/build/*
    pattern = os.path.join(repo_root(), "STM32G474CE", ".pio", "build",
                           "*", "firmware.bin")
    matches = glob.glob(pattern)
    if not matches:
        print("ERROR: no STM32 firmware.bin found.")
        print("Looked for: %s" % pattern)
        print("Build the STM32 project in PlatformIO first, or pass --bin <path>.")
        sys.exit(1)
    matches.sort(key=os.path.getmtime, reverse=True)
    if len(matches) > 1:
        print("Note: multiple firmware.bin found; using the newest:")
        for m in matches:
            print("   %s  (%s)" % (m, time.ctime(os.path.getmtime(m))))
    return matches[0]


def hosts_from_args():
    # positional args that are not flags or the --bin value
    args = []
    skip = False
    for a in sys.argv[1:]:
        if skip:
            skip = False
            continue
        if a == "--bin":
            skip = True
            continue
        if a.startswith("-"):
            continue
        args.append(a)
    return args if args else DEFAULT_HOSTS


def upload(host, data):
    url = "http://%s/stm32" % host
    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Content-Type", "application/octet-stream")
    req.add_header("Content-Length", str(len(data)))
    print("-> POST %s  (%d bytes)" % (url, len(data)))
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=TIMEOUT_S) as resp:
        body = resp.read().decode(errors="replace").strip()
        dt = time.time() - t0
        print("<- HTTP %d  (%.1fs)  %s" % (resp.status, dt, body))
        return resp.status == 200


def main():
    fw = find_firmware()
    with open(fw, "rb") as f:
        data = f.read()
    print("STM32 firmware: %s  (%d bytes)" % (fw, len(data)))
    if len(data) < 256 or len(data) > 512 * 1024:
        print("ERROR: image size out of range (256 B .. 512 KB for G474CE).")
        return 1

    last_err = None
    for host in hosts_from_args():
        try:
            print("\n=== Trying %s ===" % host)
            if upload(host, data):
                print("\nDONE. The ESP32 received the image and is flashing the")
                print("STM32 now. Watch the device screen for the STM32 progress")
                print("bar; the ESP32 reboots automatically when finished.")
                return 0
        except urllib.error.HTTPError as e:
            last_err = "HTTP %d %s" % (e.code, e.reason)
            print("   failed: %s" % last_err)
        except Exception as e:  # noqa: BLE001 (CLI tool: report anything)
            last_err = str(e)
            print("   failed: %s" % last_err)

    print("\nERROR: could not reach any host (%s)." % ", ".join(hosts_from_args()))
    print("Last error: %s" % last_err)
    print("Checklist:")
    print("  - ESP32 powered on and joined to your WiFi (check the screen IP).")
    print("  - The ESP32 is running firmware that includes the /stm32 endpoint.")
    print("  - The STM32 BOOT0 line is wired to ESP32 GPIO31.")
    print("  - You're on the same network/subnet as the device.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
