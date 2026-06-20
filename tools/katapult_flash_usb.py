#!/usr/bin/env python3
"""
Katapult USB-to-Serial Flasher for STM32G474

This script flashes firmware to the STM32 via Katapult bootloader using a USB-to-serial adapter.
It's a PC-side alternative to the ESP32 wireless flash path.

Hardware Setup:
  - USB-to-serial adapter connected to USART1:
    * Adapter TX → STM32 PA10 (RX)
    * Adapter RX → STM32 PA9 (TX)
    * Adapter GND → STM32 GND
  - Optional: Adapter GPIO → STM32 NRST for automated double-tap reset

Usage:
  1. Manual reset entry (if no GPIO control):
     python3 katapult_flash_usb.py /dev/ttyUSB0 firmware.bin
     → Script will wait for you to manually double-tap NRST button

  2. Automated reset entry (if GPIO wired):
     python3 katapult_flash_usb.py /dev/ttyUSB0 firmware.bin --reset-gpio <gpio_number>
     → Script will drive the GPIO to perform hardware double-tap

Protocol:
  - Katapult bootloader runs at 250000 baud, 8N1
  - CRC-16-CCITT checked frames
  - Firmware must be built for 0x08002000 (Katapult relocates app)
"""

import serial
import sys
import time
import struct
import argparse

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

# Katapult protocol constants
KATAPULT_BAUD = 250000
KATAPULT_HEADER = [0x01, 0x88]
KATAPULT_TRAILER = [0x99, 0x03]

CMD_CONNECT = 0x11
CMD_SEND_BLOCK = 0x12
CMD_SEND_EOF = 0x13
CMD_COMPLETE = 0x15

RESP_ACK = 0xf0
RESP_NACK = 0xf1
RESP_CMD_ERROR = 0xf2
RESP_CMD_BUSY = 0xf3

APP_START = 0x08002000  # Katapult app offset

def crc16_ccitt(data):
    """Calculate CRC-16-CCITT (polynomial 0x1021, init 0xFFFF)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def build_frame(cmd, payload=b''):
    """Build Katapult protocol frame"""
    word_len = len(payload) // 4
    frame = bytes(KATAPULT_HEADER)
    frame += struct.pack('BB', cmd, word_len)
    frame += payload
    crc = crc16_ccitt(frame[2:])
    frame += struct.pack('>H', crc)  # Big-endian CRC
    frame += bytes(KATAPULT_TRAILER)
    return frame

def parse_response(ser, timeout=1.0):
    """Parse Katapult response frame"""
    start = time.time()
    buf = b''
    
    # Find header
    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            buf += byte
            if len(buf) >= 2 and buf[-2:] == bytes(KATAPULT_HEADER):
                break
    else:
        return None, None  # Timeout
    
    # Read cmd + word_len
    header = ser.read(2)
    if len(header) < 2:
        return None, None
    
    cmd = header[0]
    word_len = header[1]
    
    # Read payload
    payload_len = word_len * 4
    payload = ser.read(payload_len) if payload_len > 0 else b''
    
    # Read CRC
    crc_bytes = ser.read(2)
    if len(crc_bytes) < 2:
        return None, None
    
    # Read trailer
    trailer = ser.read(2)
    if trailer != bytes(KATAPULT_TRAILER):
        print(f"Warning: Invalid trailer: {trailer.hex()}")
    
    # Verify CRC
    frame_data = header + payload
    expected_crc = crc16_ccitt(frame_data)
    received_crc = struct.unpack('>H', crc_bytes)[0]
    
    if expected_crc != received_crc:
        print(f"Warning: CRC mismatch (expected {expected_crc:04x}, got {received_crc:04x})")
    
    return cmd, payload

def hardware_double_tap(gpio_pin):
    """Perform hardware double-tap reset on NRST via GPIO"""
    if not GPIO_AVAILABLE:
        print("ERROR: RPi.GPIO not available. Install with: pip3 install RPi.GPIO")
        return False
    
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(gpio_pin, GPIO.OUT, initial=GPIO.HIGH)
        
        print(f"Performing hardware double-tap on GPIO {gpio_pin}...")
        
        # First tap
        GPIO.output(gpio_pin, GPIO.LOW)
        time.sleep(0.010)  # 10ms
        GPIO.output(gpio_pin, GPIO.HIGH)
        
        # Wait 200ms
        time.sleep(0.200)
        
        # Second tap
        GPIO.output(gpio_pin, GPIO.LOW)
        time.sleep(0.010)  # 10ms
        GPIO.output(gpio_pin, GPIO.HIGH)
        
        # Wait for bootloader to initialize
        time.sleep(0.300)
        
        print("Double-tap complete!")
        return True
        
    except Exception as e:
        print(f"GPIO error: {e}")
        return False
    finally:
        GPIO.cleanup()

def flash_firmware(port, firmware_path, reset_gpio=None):
    """Flash firmware via Katapult bootloader"""
    
    # Load firmware
    print(f"Loading firmware from {firmware_path}...")
    try:
        with open(firmware_path, 'rb') as f:
            firmware = f.read()
    except Exception as e:
        print(f"ERROR: Failed to load firmware: {e}")
        return False
    
    print(f"Firmware size: {len(firmware)} bytes")
    
    # Open serial port
    print(f"Opening {port} at {KATAPULT_BAUD} baud...")
    try:
        ser = serial.Serial(port, KATAPULT_BAUD, timeout=1.0)
    except Exception as e:
        print(f"ERROR: Failed to open serial port: {e}")
        return False
    
    time.sleep(0.1)
    
    # Enter bootloader
    if reset_gpio is not None:
        # Automated entry via GPIO
        if not hardware_double_tap(reset_gpio):
            return False
    else:
        # Manual entry - wait for user
        print("\n" + "="*60)
        print("MANUAL RESET REQUIRED")
        print("="*60)
        print("Please double-tap the NRST button on the STM32:")
        print("  1. Press and release NRST")
        print("  2. Wait ~200ms")
        print("  3. Press and release NRST again")
        print("The bootloader should enter and hold.")
        print("="*60)
        input("\nPress Enter after double-tapping NRST...")
        print()
    
    ser.reset_input_buffer()
    
    # Send CONNECT
    print("Sending CONNECT...")
    connect_frame = build_frame(CMD_CONNECT)
    ser.write(connect_frame)
    
    resp_cmd, resp_payload = parse_response(ser, timeout=2.0)
    
    if resp_cmd != CMD_CONNECT:
        if resp_cmd == RESP_NACK:
            print("ERROR: Bootloader sent NACK to CONNECT")
        elif resp_cmd is None:
            print("ERROR: No response from bootloader (timeout)")
            print("  - Is STM32 powered?")
            print("  - Is USART1 (PA9/PA10) connected correctly?")
            print("  - Did the double-tap reset work?")
        else:
            print(f"ERROR: Unexpected response to CONNECT: 0x{resp_cmd:02x}")
        ser.close()
        return False
    
    # Parse CONNECT response
    if len(resp_payload) < 9:
        print(f"ERROR: CONNECT response too short: {len(resp_payload)} bytes")
        ser.close()
        return False
    
    proto_ver = resp_payload[0]
    app_start = struct.unpack('>I', resp_payload[1:5])[0]
    block_size = struct.unpack('>I', resp_payload[5:9])[0]
    
    print(f"✓ CONNECT OK:")
    print(f"  Protocol version: {proto_ver}")
    print(f"  App start: 0x{app_start:08x}")
    print(f"  Block size: {block_size} bytes")
    
    if app_start != APP_START:
        print(f"WARNING: Expected app_start 0x{APP_START:08x}, got 0x{app_start:08x}")
    
    # Flash firmware in blocks
    print(f"\nFlashing {len(firmware)} bytes...")
    offset = 0
    block_count = 0
    
    while offset < len(firmware):
        remain = len(firmware) - offset
        chunk_size = min(block_size, remain)
        chunk = firmware[offset:offset+chunk_size]
        
        # Pad to 4-byte boundary
        while len(chunk) % 4 != 0:
            chunk += b'\xff'
        
        # Build SEND_BLOCK payload: address (4 bytes) + data
        block_addr = app_start + offset
        payload = struct.pack('>I', block_addr) + chunk
        
        # Send block
        block_frame = build_frame(CMD_SEND_BLOCK, payload)
        ser.write(block_frame)
        
        # Wait for ACK
        resp_cmd, _ = parse_response(ser, timeout=2.0)
        
        if resp_cmd != RESP_ACK:
            if resp_cmd == RESP_NACK:
                print(f"\nERROR: NACK at offset {offset}")
            elif resp_cmd == RESP_CMD_ERROR:
                print(f"\nERROR: CMD_ERROR at offset {offset}")
            else:
                print(f"\nERROR: Unexpected response 0x{resp_cmd:02x} at offset {offset}")
            ser.close()
            return False
        
        offset += chunk_size
        block_count += 1
        
        # Progress
        pct = (offset * 100) // len(firmware)
        print(f"\r  Progress: {pct}% ({offset}/{len(firmware)} bytes, {block_count} blocks)", end='')
    
    print()  # Newline after progress
    
    # Send EOF
    print("Sending EOF...")
    eof_frame = build_frame(CMD_SEND_EOF)
    ser.write(eof_frame)
    
    resp_cmd, _ = parse_response(ser, timeout=2.0)
    if resp_cmd != RESP_ACK:
        print(f"ERROR: EOF response: 0x{resp_cmd:02x}")
        ser.close()
        return False
    
    print("✓ EOF acknowledged")
    
    # Send COMPLETE
    print("Sending COMPLETE...")
    complete_frame = build_frame(CMD_COMPLETE)
    ser.write(complete_frame)
    
    resp_cmd, _ = parse_response(ser, timeout=2.0)
    if resp_cmd != RESP_ACK:
        print(f"ERROR: COMPLETE response: 0x{resp_cmd:02x}")
        ser.close()
        return False
    
    print("✓ COMPLETE acknowledged")
    print("\n" + "="*60)
    print("✅ FLASH SUCCESSFUL!")
    print("="*60)
    print("The STM32 should now boot into the new firmware.")
    
    ser.close()
    return True

def main():
    parser = argparse.ArgumentParser(
        description='Flash STM32G474 firmware via Katapult bootloader over USB-to-serial',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Manual reset (user double-taps NRST button)
  python3 katapult_flash_usb.py /dev/ttyUSB0 firmware.bin

  # Automated reset (GPIO 17 wired to NRST, Raspberry Pi)
  python3 katapult_flash_usb.py /dev/ttyUSB0 firmware.bin --reset-gpio 17

Hardware Wiring:
  USB-to-Serial → STM32 USART1
    TX → PA10 (STM32 RX)
    RX → PA9  (STM32 TX)
    GND → GND

  Optional for automated reset (Raspberry Pi):
    GPIO pin → STM32 NRST
        """
    )
    
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('firmware', help='Firmware binary file (.bin)')
    parser.add_argument('--reset-gpio', type=int, metavar='N',
                       help='GPIO pin number for automated NRST double-tap (BCM numbering)')
    
    args = parser.parse_args()
    
    success = flash_firmware(args.port, args.firmware, args.reset_gpio)
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
