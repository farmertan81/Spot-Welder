# ESP32-P4 SPI WiFi Integration Checklist

## ✅ What's Been Done

### Stage 1: XIAO C6 Slave Firmware (Complete)
- ✅ Seeed XIAO ESP32-C6 flashed with esp-hosted SPI slave firmware v2.12.9
- ✅ SPI transport initialized: MOSI=18, MISO=20, CLK=19, CS=2, HS=1, DR=17
- ✅ XIAO boots successfully and waits for P4 host

### Stage 2: P4 Host Firmware Reconfigured (Complete — This Commit)
- ✅ `sdkconfig.defaults` switched from SDIO → SPI transport
- ✅ SPI master GPIOs mapped to J7 wireless module header
- ✅ Pin configuration matches XIAO slave exactly
- ✅ Committed to Dev branch (`b399380`)

---

## 🔧 Next Steps: Hardware + Testing

### Step 1: Physical Wiring (J7 Header → XIAO C6)

**CRITICAL:** Wire the XIAO C6 to the CrowPanel J7 wireless module header:

| Signal      | P4 GPIO | J7 Pin | XIAO Pin | XIAO GPIO |
|-------------|---------|--------|----------|-----------|
| MOSI        | 48      | —      | D10      | 18        |
| MISO        | 47      | —      | D9       | 20        |
| CLK         | 26      | —      | D8       | 19        |
| CS          | 29      | —      | D2       | 2         |
| Handshake   | 30      | —      | D1       | 1         |
| Data Ready  | 31      | —      | D7       | 17        |
| Power       | —       | 3V3    | 3V3      | —         |
| Ground      | —       | GND    | GND      | —         |

⚠️ **DO NOT connect XIAO D4/D5** (GPIO22/23) — they're physically wired to the P4's I2C bus (GT911 touch + STC8 backlight controller).

**Verify wiring:**
- Use short jumper wires (< 10 cm) to minimize signal noise
- Check continuity with a multimeter if available
- Ensure XIAO is powered (3.3V on J7 header)

---

### Step 2: Rebuild & Flash P4 Firmware

The P4 firmware **MUST** be rebuilt with the new SPI config:

```bash
# On your Windows machine (ESP-IDF PowerShell):
cd C:\path\to\Spot-Welder\ESP32P4
git pull origin Dev  # Get the latest SPI config

# Clean old SDIO config artifacts
idf.py fullclean

# Rebuild with new SPI config
idf.py build

# Flash to P4
idf.py -p COM6 flash monitor
```

---

### Step 3: Verify Boot Sequence

Watch the serial monitor output after flashing. You should see:

#### ✅ Expected Success Output:
```
I (xxxx) wifi_stack_init: Initializing ESP-Hosted...
I (xxxx) transport: SPI transport initialized
I (xxxx) transport: MOSI[48] MISO[47] CLK[26] CS[29] HS[30] DR[31]
I (xxxx) transport: Waiting for slave...
I (xxxx) transport: Slave ready, ESP-Hosted link UP
I (xxxx) wifi_bridge: WiFi stack initialized successfully
```

#### ❌ If You See Errors:

**"Slave not responding" / "ESP-Hosted link timeout":**
- Check XIAO is powered and running (use a USB cable to XIAO to verify it boots)
- Check XIAO serial monitor shows "SPI slave initialized, waiting for host..."
- Verify all 8 signal wires (6 SPI + 3V3 + GND) are connected correctly
- Check for loose connections on J7 header

**"SPI transaction failed" / "Checksum error":**
- SPI clock may be too fast (20 MHz default)
- Lower it in `sdkconfig.defaults`: `CONFIG_ESP_HOSTED_SPI_FREQ_ESP32C6=10`
- Rebuild and reflash P4

**"SDIO timeout" (old message):**
- The build used the old SDIO config
- Run `idf.py fullclean` to wipe stale sdkconfig
- Rebuild from scratch

---

### Step 4: Test WiFi Functionality

Once the P4 boots with "ESP-Hosted link UP":

1. **WiFi provisioning:** Open the welder UI, navigate to WiFi settings
2. **Connect to your network:** Enter SSID + password
3. **Verify connection:** Check the WiFi status icon on the main screen
4. **Test OTA:** Go to Settings → OTA Update, verify the firmware update flow works

---

## 🔍 Troubleshooting

### P4 boots but "WiFi disabled" (link failed)
- This is **expected** if XIAO isn't connected or running
- The welder UI will work normally (WiFi just disabled)
- Fix: Wire the XIAO, verify it's flashed and running, reboot P4

### WiFi connects but SD card doesn't work
- ✅ **This is exactly what we fixed!** The SDMMC controller is now free.
- Test the SD card separately (file browser, data logging, etc.)
- If SD card still fails, check the SD card slot wiring (separate issue)

### XIAO slave stops responding after P4 reboot
- The XIAO might need a power cycle when P4 reboots
- Add a reset pulse: Connect XIAO EN pin to a P4 GPIO and toggle it in `wifi_stack_init()`
- For now: manually power-cycle the XIAO after P4 reboots

---

## 📊 Performance Tuning (Optional)

Once WiFi is stable, you can increase SPI clock speed for better throughput:

In `ESP32P4/sdkconfig.defaults`, change:
```
CONFIG_ESP_HOSTED_SPI_FREQ_ESP32C6=20  # Start here (conservative)
# Try increasing to 30, then 40 (max for C6)
```

Rebuild, flash, and test WiFi stability after each increase.

---

## 🎯 Success Criteria

You're done when:
- ✅ P4 boots with "ESP-Hosted link UP"
- ✅ WiFi connects to your network
- ✅ SD card mounts successfully
- ✅ Welder UI fully functional with WiFi + SD card both working

---

## 📝 Notes

- **Stable baseline preserved:** Your current working firmware is now in `main` (commit `567d197`)
- **Dev branch is safe to experiment:** Any issues, just revert to main
- **XIAO firmware unchanged:** If you need to reflash XIAO, use `/home/ubuntu/xiao-c6-slave.tar.gz`

---

Need help? Paste the P4 serial monitor output showing the WiFi init sequence.
