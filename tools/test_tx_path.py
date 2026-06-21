#!/usr/bin/env python3
"""
TX-path isolation test.

Sends a HARMLESS command (READY,1) to the STM32 app at 1Mbaud and checks
whether the app echoes the matching ACK. This proves, beyond doubt, whether
the PC's TX line actually reaches the STM32 RX pin (PA10).

  - See "ACK,READY,1"  -> TX path WORKS. The BOOTLOADER problem is elsewhere.
  - No ACK (only STATUS) -> TX path is DEAD. Adapter/BMP TX is not wired to PA10,
                            or the baud rate is wrong for the TX direction.

Usage:  python test_tx_path.py COM4
"""
import serial
import sys
import time

def main():
    if len(sys.argv) < 2:
        print("Usage: python test_tx_path.py <COM_PORT>")
        sys.exit(1)
    port = sys.argv[1]

    print("="*60)
    print("TX-PATH ISOLATION TEST (1,000,000 baud)")
    print("="*60)

    try:
        ser = serial.Serial(port, 1000000, timeout=0.3)
    except serial.SerialException as e:
        print(f"❌ Could not open {port}: {e}")
        print("   Close MobaXterm / any other serial monitor first.")
        sys.exit(1)

    time.sleep(0.2)

    # We will test several commands the app definitely answers, in order.
    # Each one elicits a distinct ACK the app sends ONLY in direct reply.
    tests = [
        ("READY,1",  b"ACK,READY,1"),
        ("READY,0",  b"ACK,READY,0"),
    ]

    overall_ok = False

    for cmd, expect in tests:
        ser.reset_input_buffer()
        print(f"\n>> Sending: {cmd!r}")
        ser.write((cmd + "\n").encode("ascii"))
        ser.flush()

        # Read for ~400ms and scan for the expected ACK
        deadline = time.time() + 0.4
        buf = b""
        while time.time() < deadline:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
            if expect in buf:
                break

        if expect in buf:
            print(f"   ✅ Got {expect.decode()}  -> command was RECEIVED and ACTED ON")
            overall_ok = True
        else:
            txt = buf.decode("ascii", errors="replace")
            print(f"   ❌ No {expect.decode()} seen.")
            print(f"      Buffer snippet: {txt[:100]!r}")

    ser.close()

    print("\n" + "="*60)
    if overall_ok:
        print("✅ TX PATH WORKS — the PC can send commands to the STM32.")
        print("   So the BOOTLOADER issue is NOT a wiring problem.")
        print("   Next: investigate the Katapult-request / reset path.")
    else:
        print("❌ TX PATH DEAD — the STM32 never acted on any command.")
        print("   The app keeps streaming STATUS but ignores input. Causes:")
        print("     1. Adapter/Black-Magic TX is NOT wired to STM32 PA10 (RX).")
        print("     2. Black Magic passthrough TX doesn't run at 1Mbaud.")
        print("     3. Wrong COM port for the TX direction.")
        print("   Fix the wiring/baud, then BOOTLOADER will work too.")
    print("="*60)

if __name__ == "__main__":
    main()
