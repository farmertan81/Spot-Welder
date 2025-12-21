#!/usr/bin/env python3
"""
TCP Weld Data Receiver
Connects to ESP32 TCP server to receive high-speed weld data
"""

import socket
import threading
import time

class TCPWeldReceiver:
    def __init__(self, esp32_ip='192.168.68.71', port=8081, data_callback=None, log_callback=None):
        self.esp32_ip = esp32_ip
        self.port = port
        self.data_callback = data_callback
        self.log_callback = log_callback
        self.sock = None
        self.connected = False
        self.running = False
        self.thread = None
        
    def log(self, msg):
        if self.log_callback:
            self.log_callback(f"[TCP] {msg}")
        else:
            print(f"[TCP] {msg}")
    
    def connect(self):
        """Connect to ESP32 TCP server"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.esp32_ip, self.port))
            self.sock.settimeout(0.1)
            self.connected = True
            self.log(f"Connected to {self.esp32_ip}:{self.port}")
            return True
        except Exception as e:
            self.log(f"Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close connection"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.connected = False
        self.log("Disconnected")
    
    def start(self):
        """Start background receive thread"""
        if not self.connected:
            if not self.connect():
                return False
        
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        self.log("Receive thread started")
        return True
    
    def stop(self):
        """Stop background thread"""
        self.disconnect()
    
    def _receive_loop(self):
        """Background thread to receive TCP data"""
        line_buffer = ""
        
        while self.running:
            try:
                if not self.sock:
                    time.sleep(0.1)
                    continue
                
                # Receive data
                try:
                    chunk = self.sock.recv(4096).decode('utf-8', errors='ignore')
                    if not chunk:
                        self.log("Connection closed by ESP32")
                        self.connected = False
                        break
                    
                    line_buffer += chunk
                    
                    # Process complete lines
                    while '\n' in line_buffer:
                        line, line_buffer = line_buffer.split('\n', 1)
                        line = line.strip()
                        if line and self.data_callback:
                            self.data_callback(line)
                
                except socket.timeout:
                    time.sleep(0.01)
                except Exception as e:
                    self.log(f"Receive error: {e}")
                    self.connected = False
                    break
            
            except Exception as e:
                self.log(f"Loop error: {e}")
                time.sleep(1.0)

# Test code
if __name__ == '__main__':
    def on_data(line):
        print(f"RX: {line}")
    
    receiver = TCPWeldReceiver(data_callback=on_data)
    receiver.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        receiver.stop()
