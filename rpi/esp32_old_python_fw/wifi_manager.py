# WiFi Manager for ESP32
import network
import time

def connect_wifi(ssid, password, timeout=10):
    """Connect to WiFi and return IP address"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if wlan.isconnected():
        print(f"Already connected to WiFi: {wlan.ifconfig()[0]}")
        return wlan.ifconfig()[0]
    
    print(f"Connecting to WiFi: {ssid}...", end="")
    wlan.connect(ssid, password)
    
    start = time.time()
    while not wlan.isconnected():
        if time.time() - start > timeout:
            print(" TIMEOUT")
            return None
        print(".", end="")
        time.sleep(0.5)
    
    ip = wlan.ifconfig()[0]
    print(f" Connected!")
    print(f"IP Address: {ip}")
    return ip

def start_ftp_server():
    """Start FTP server"""
    try:
        import uftpd
        print("FTP server started on port 21")
        print("User: esp32 / Pass: welder123")
        return True
    except Exception as e:
        print(f"FTP server failed: {e}")
        return False
