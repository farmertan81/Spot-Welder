#!/usr/bin/env python3
"""
Test script to verify Katapult bootloader entry methods.

Usage:
  1. Connect USB-to-serial adapter to STM32 USART1 (PA9=TX, PA10=RX)
  2. Run: python3 test_katapult_entry.py /dev/ttyUSB0
  3. Script will try to detect if Katapult is responding

If Katapult is active, it will respond to CONNECT command.
"""

import serial
import sys
import time
import struct

def crc16_ccitt(data):
    """Calculate CRC-16-CCITT (poly 0x1021, init 0xFFFF)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def build_katapult_frame(cmd, payload=b''):
    """Build Katapult protocol frame"""
    word_len = len(payload) // 4
    frame = struct.pack('<BB', 0x01, 0x88)  # Header
    frame += struct.pack('<BB', cmd, word_len)
    frame += payload
    crc = crc16_ccitt(frame[2:])  # CRC of cmd+word_len+payload
    frame += struct.pack('>H', crc)  # CRC big-endian
    frame += struct.pack('<BB', 0x99, 0x03)  # Trailer
    return frame

def test_katapult_response(port_name):
    """Test if Katapult bootloader is responding"""
    print(f"Testing Katapult on {port_name}...")
    print("Bootloader should be at 250000 baud, 8N1")
    print()
    
    try:
        # Open serial at Katapult baud rate
        ser = serial.Serial(port_name, 250000, timeout=1.0)
        time.sleep(0.1)
        
        # Send CONNECT command (0x11)
        connect_frame = build_katapult_frame(0x11)
        print(f"Sending CONNECT: {connect_frame.hex()}")
        ser.write(connect_frame)
        
        # Read response
        response = ser.read(100)
        if len(response) > 0:
            print(f"✅ Got response: {response.hex()}")
            print()
            
            # Parse response
            if len(response) >= 4 and response[0:2] == b'\x01\x88':
                resp_code = response[2]
                if resp_code == 0xf0:
                    print("✅ ACK received - Katapult is ACTIVE!")
                    print()
                    print("Bootloader is ready for flashing.")
                    return True
                elif resp_code == 0xf1:
                    print("⚠️  NACK received")
                elif resp_code == 0xf2:
                    print("⚠️  CMD_ERROR received")
                elif resp_code == 0xf3:
                    print("⚠️  CMD_BUSY received")
                else:
                    print(f"⚠️  Unknown response code: 0x{resp_code:02x}")
            else:
                print("⚠️  Invalid frame format")
        else:
            print("❌ No response - Katapult NOT active")
            print()
            print("Possible reasons:")
            print("  1. STM32 is running the application (not bootloader)")
            print("  2. Wrong serial port or not connected")
            print("  3. Need to enter bootloader via double-tap NRST")
            print()
            print("To enter bootloader:")
            print("  - Method 1: Send 'BOOTLOADER' command from ESP32")
            print("  - Method 2: Double-tap NRST button (< 500ms apart)")
            print("  - Method 3: Re-flash via ST-Link")
            
        ser.close()
        return False
        
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        return False

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_katapult_entry.py <serial_port>")
        print("Example: python3 test_katapult_entry.py /dev/ttyUSB0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print("=" * 60)
    print("Katapult Bootloader Entry Test")
    print("=" * 60)
    print()
    print("Instructions:")
    print("  1. Ensure USB-to-serial is connected to USART1")
    print("     PA9 (STM32 TX) → RX on adapter")
    print("     PA10 (STM32 RX) → TX on adapter")
    print("  2. If STM32 is running the app, try double-tap NRST")
    print("  3. Run this script")
    print()
    
    input("Press Enter to test...")
    print()
    
    success = test_katapult_response(port)
    
    print()
    print("=" * 60)
    if success:
        print("✅ TEST PASSED - Bootloader is active and responding")
    else:
        print("❌ TEST FAILED - Bootloader not detected")
    print("=" * 60)

if __name__ == "__main__":
    main()
