#!/usr/bin/env python3
"""
Automatic Katapult Bootloader Test
- Sends BOOTLOADER command to enter bootloader
- Tests if Katapult is responding
- All in one script, no manual double-tap needed
"""

import serial
import time
import struct
import sys

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

def main():
    if len(sys.argv) < 2:
        print("Usage: python test_katapult_auto.py <COM_PORT>")
        print("Example: python test_katapult_auto.py COM4")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print("="*60)
    print("Automatic Katapult Bootloader Entry Test")
    print("="*60)
    print()
    
    try:
        # Step 1: Open at app baud (1Mbaud) and send BOOTLOADER command
        print(f"Step 1: Opening {port} @ 1,000,000 baud (app mode)...")
        ser = serial.Serial(port, 1000000, timeout=0.5)
        time.sleep(0.1)
        
        # First, drain any STATUS messages the app is streaming so we have a clean view
        ser.reset_input_buffer()
        time.sleep(0.05)

        print("Step 2: Sending BOOTLOADER command...")
        ser.write(b"BOOTLOADER\n")
        ser.flush()

        # CRITICAL DIAGNOSTIC: read the app's reply at 1Mbaud BEFORE the reset.
        # If the command reached the STM32 RX pin (PA10), the app prints
        # "ACK,BOOTLOADER" then resets. If we see NO ack and STATUS keeps
        # streaming, the TX->PA10 wire is not connected (monitor-only link).
        print("        Listening at 1Mbaud for the app's reply (250ms)...")
        time.sleep(0.05)
        reply = ser.read(400)
        try:
            reply_txt = reply.decode('ascii', errors='replace')
        except Exception:
            reply_txt = repr(reply)

        if b"ACK,BOOTLOADER" in reply or b"KATAPULT" in reply.upper():
            print("        ✅ App acknowledged the command (ACK,BOOTLOADER seen).")
            print("           -> Command path OK, STM32 is resetting into Katapult.")
        elif b"STATUS" in reply:
            print("        ⚠️  App is STILL streaming STATUS - command was NOT acted on.")
            print("           -> Most likely the serial TX line is NOT wired to STM32 PA10 (RX).")
            print("           -> Your link is probably monitor-only (STM32 TX -> adapter RX only).")
            print(f"           Raw 1Mbaud reply snippet: {reply_txt[:120]!r}")
        elif len(reply) == 0:
            print("        ⚠️  Silence at 1Mbaud after the command.")
            print("           -> STM32 may have reset; continuing to Katapult check.")
        else:
            print(f"        Reply ({len(reply)} bytes): {reply_txt[:120]!r}")

        print("         (STM32 should reset into Katapult now)")
        time.sleep(0.4)  # Wait for reset + Katapult init
        
        # Step 3: Switch to Katapult baud (250k)
        print("Step 3: Switching to 250,000 baud (Katapult mode)...")
        ser.close()
        ser = serial.Serial(port, 250000, timeout=1.5)
        time.sleep(0.2)
        
        # Flush any leftover data
        ser.reset_input_buffer()
        
        # Step 4: Send CONNECT frame
        print("Step 4: Sending CONNECT frame...")
        frame_data = bytes([0x11, 0x00])  # cmd=0x11 (CONNECT), word_len=0
        crc = crc16_ccitt(frame_data)
        connect = bytes([0x01, 0x88]) + frame_data + struct.pack('>H', crc) + bytes([0x99, 0x03])
        print(f"        Frame: {connect.hex()}")
        ser.write(connect)
        
        # Step 5: Read response
        print("Step 5: Waiting for response...")
        response = ser.read(100)
        
        print()
        print("="*60)
        
        if len(response) == 0:
            print("❌ TEST FAILED - No response from bootloader")
            print()
            print("Possible issues:")
            print("  - BOOTLOADER command didn't trigger reset")
            print("  - Katapult isn't installed at 0x08000000")
            print("  - Serial connection issue")
            ser.close()
            sys.exit(1)
        
        print(f"✅ Got response: {len(response)} bytes")
        print(f"   Raw data: {response.hex()}")
        print()
        
        # Parse response
        if len(response) >= 4 and response[0:2] == bytes([0x01, 0x88]):
            resp_cmd = response[2]
            resp_word_len = response[3]
            
            if resp_cmd == 0x11:  # CONNECT response
                print("✅✅ SUCCESS - Katapult is ACTIVE!")
                print(f"    Response: CONNECT (0x{resp_cmd:02x})")
                print(f"    Payload: {resp_word_len * 4} bytes")
                
                if len(response) >= 9:
                    proto_ver = response[4]
                    app_start = struct.unpack('>I', response[5:9])[0]
                    if len(response) >= 13:
                        block_size = struct.unpack('>I', response[9:13])[0]
                        print(f"    Protocol version: {proto_ver}")
                        print(f"    App start: 0x{app_start:08x}")
                        print(f"    Block size: {block_size} bytes")
                
                print()
                print("Bootloader is ready for flashing!")
                ser.close()
                sys.exit(0)
            
            elif resp_cmd == 0xf0:  # ACK
                print("⚠️  Got ACK (0xf0) instead of CONNECT response")
                print("    This is unusual - Katapult may be in a weird state")
            
            elif resp_cmd == 0xf1:  # NACK
                print("❌ Got NACK (0xf1) - Katapult rejected CONNECT")
            
            else:
                print(f"⚠️  Unexpected response command: 0x{resp_cmd:02x}")
        
        else:
            print("⚠️  Response doesn't match Katapult frame format")
            print("    Expected: 01 88 <cmd> <len> ...")
            print(f"    Got:      {response[:4].hex() if len(response) >= 4 else response.hex()}")
        
        print()
        print("="*60)
        print("❌ TEST FAILED - Unexpected response format")
        print("="*60)
        ser.close()
        sys.exit(1)
        
    except serial.SerialException as e:
        print()
        print("="*60)
        print(f"❌ SERIAL ERROR: {e}")
        print("="*60)
        print()
        print("Common issues:")
        print(f"  - {port} is already open in another program (MobaXterm, etc.)")
        print(f"  - {port} doesn't exist or isn't connected")
        print("  - Check Device Manager → Ports (COM & LPT)")
        sys.exit(1)
    
    except KeyboardInterrupt:
        print("\n\nTest cancelled by user.")
        sys.exit(1)

if __name__ == '__main__':
    main()
