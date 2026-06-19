#!/usr/bin/env python3
# ============================================================
#  ota_flash.py — flash ESP32-P4 firmware over WiFi (no USB)
# ============================================================
# This is the ESP-IDF equivalent of PlatformIO's `upload_protocol = espota`.
# It POSTs the built firmware image straight to the device's /ota endpoint,
# which writes it to the inactive OTA partition and reboots into it.
#
# USAGE (from the ESP32P4 project folder):
#   python tools/ota_flash.py                 # uses defaults below
#   python tools/ota_flash.py 192.168.1.77    # override host
#   python tools/ota_flash.py spotwelder.local
#
# It is normally launched by the VS Code task "Flash over WiFi (OTA)"
# (see .vscode/tasks.json) AFTER you build with the ESP-IDF "Build" button.
#
# Only the Python standard library is used, so it works with the same Python
# that ships with ESP-IDF — no `pip install` required.

import base64
import os
import sys
import time
import urllib.request
import urllib.error

# ---- Defaults (edit to taste) -------------------------------------------
DEFAULT_HOSTS = ["spotwelder.local", "192.168.1.77"]  # tried in order
USERNAME      = "admin"
PASSWORD      = "spotwelder2024"
FIRMWARE_REL  = os.path.join("build", "hello_world.bin")  # project name = hello_world
TIMEOUT_S     = 120
# -------------------------------------------------------------------------


def project_root():
    # tools/ota_flash.py  ->  project root is one level up from tools/
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def find_firmware():
    fw = os.path.join(project_root(), FIRMWARE_REL)
    if not os.path.isfile(fw):
        print("ERROR: firmware not found:\n  %s" % fw)
        print("Build first (ESP-IDF 'Build' button or `idf.py build`).")
        sys.exit(1)
    return fw


def hosts_from_args():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    return args if args else DEFAULT_HOSTS


def upload(host, data):
    url = "http://%s/ota" % host
    auth = base64.b64encode(("%s:%s" % (USERNAME, PASSWORD)).encode()).decode()
    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Authorization", "Basic " + auth)
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
    print("Firmware: %s  (%d bytes)" % (fw, len(data)))

    last_err = None
    for host in hosts_from_args():
        try:
            print("\n=== Trying %s ===" % host)
            if upload(host, data):
                print("\nDONE. Device is rebooting into the new firmware.")
                print("Watch the serial monitor for the boot banner to confirm.")
                return 0
        except urllib.error.HTTPError as e:
            last_err = "HTTP %d %s" % (e.code, e.reason)
            print("   failed: %s" % last_err)
        except Exception as e:  # noqa: BLE001 (CLI tool: report anything)
            last_err = str(e)
            print("   failed: %s" % last_err)

    print("\nERROR: could not flash any host (%s)." % ", ".join(hosts_from_args()))
    print("Last error: %s" % last_err)
    print("Checklist:")
    print("  - Device powered on and joined to your WiFi (check the screen IP).")
    print("  - You ran the crash-fix build over USB at least once.")
    print("  - You're on the same network/subnet as the device.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
