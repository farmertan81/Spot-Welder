# Spot-Welder

A capacitor-bank battery spot welder built around a 3-tier architecture: an
**STM32G474CE** real-time controller, an **ESP32-S3** touchscreen display /
bridge, and an optional **Raspberry Pi / Flask** web dashboard for remote
monitoring and control.

---

## System Overview (3-Tier Architecture)

```
   ┌──────────────────────┐        ┌──────────────────────┐        ┌──────────────────────┐
   │   TIER 1: STM32G474   │  UART  │   TIER 2: ESP32-S3    │  WiFi  │  TIER 3: Raspberry Pi │
   │   (real-time control) │◄──────►│  (display + bridge)   │◄──────►│   / Flask web server  │
   │                       │ 2 Mbd  │                       │  TCP   │      (optional)       │
   └──────────────────────┘        └──────────────────────┘        └──────────────────────┘
        weld timing,                  4.3" 800×480 touch UI,            remote dashboard,
        ADC/INA226 sensing,           local control, SD card,           logging, graphing
        PWM, safety, Flash            UART↔TCP forwarding
```

### Tier 1 — STM32G474CE (authoritative controller)
- Owns **all** weld-control state and safety logic: pulse timing (TIM1/TIM2 +
  DWT cycle counter), PWM output, ADC + INA226 current/voltage sensing,
  lead-resistance calibration, watchdog, and persistent settings in Flash.
- Communicates with the ESP32 over **USART1 (PA9 = TX, PA10 = RX)** at
  **2 000 000 baud, 8N1**.
- The STM32 is the single source of truth — the ESP32 never overrides its
  safety decisions.

### Tier 2 — ESP32-S3 (display + bridge)
- Board: **Sunton ESP32-8048S043C** (4.3" 800×480 IPS RGB panel, GT911
  capacitive touch, PSRAM, microSD).
- Runs the LVGL touchscreen UI (Status / Pulse / Telemetry / Config / Logs
  tabs), drives the local control flow, and **bridges** the STM32 UART link to
  a TCP/WiFi connection for the optional web tier.
- Hosts the over-the-air and SD-card firmware-update flows for **both** chips.

### Tier 3 — Raspberry Pi / Flask (optional remote dashboard)
- Connects to the ESP32 over TCP (port 8888) for a remote web dashboard:
  live telemetry, voltage graphing, logging, and remote control.
- Entirely optional — the welder is fully functional with Tiers 1 + 2 alone.

---

## ✅ What's Working Perfectly

- **Welding & control** — pulse sequencing, power/preheat control, pedal and
  contact triggering, and all safety gating run rock-solid on the STM32.
- **Sensing & telemetry** — INA226 pack/cell monitoring, ADC current/voltage
  capture, and waveform capture stream cleanly to the UI at 2 Mbaud.
- **Lead-resistance calibration** — full UI → STM32 → Flash flow with
  pedal/contact trigger logic mirroring a real weld.
- **Touchscreen UI** — responsive LVGL 5-tab interface with anti-shudder
  handling and persistent NVS settings/recipes.
- **WiFi / web bridge** — STM32↔ESP32↔Flask forwarding, dual-path voltage
  reporting, mDNS (`spotwelder.local`), and ArduinoOTA.
- **ESP32 self-update** — the ESP32 reflashes its **own** firmware from the SD
  card (`/esp32_firmware.bin`) reliably.
- **STM32 wireless/SD update via Katapult bootloader** — STM32 firmware can be
  updated wirelessly (WiFi), from SD card, or via USB-to-serial, with full
  CRC validation. No ST-LINK required for routine updates.

---

## ✅ Wireless & SD Firmware Updates (ESP32-P4 + STM32)

Both the **ESP32-P4** and the **STM32G474** support **self-contained firmware updates**
with no programmer or PC required:

### ESP32-P4 Update
- Drop `/esp32_firmware.bin` on the SD card and tap "Update ESP32" on the touchscreen,
  OR upload via the web interface at `http://<esp-ip>/update`
- Uses dual OTA partitions for safe rollback

### STM32 Update (via Katapult Bootloader)

The STM32 uses a **persistent 8 KiB Katapult bootloader** at flash address `0x08000000`.
Katapult handles firmware updates with CRC validation over UART, with no BOOT0/NRST
pins required.

**Three update methods:**
1. **Wireless (WiFi)** — upload `.bin` to `http://<esp-ip>/stm32` or run
   `python3 tools/stm32_flash_wifi.py`
2. **SD card** — copy firmware to SD, tap "Update STM32" on touchscreen
3. **USB-to-serial (PC)** — `python3 tools/katapult_flash_usb.py /dev/ttyUSB0 firmware.bin`

All three paths work reliably in production and include read-back verification.

**ST-LINK is optional** — only needed for:
- Initial Katapult installation (one-time)
- Deep debugging with breakpoints
- Emergency recovery (rare)

➡️ **Full setup guide:** [docs/firmware/KATAPULT_BOOTLOADER_SETUP.md](docs/firmware/KATAPULT_BOOTLOADER_SETUP.md)

---

## Hardware Requirements

| Component | Part | Notes |
|---|---|---|
| Controller MCU | **STM32G474CEU6** | 512 KB Flash; Katapult bootloader @ `0x08000000`, app @ `0x08002000` |
| Display / bridge | **Elecrow CrowPanel ESP32-P4** | ESP32-P4, 5" 800×480 RGB, capacitive touch, 16 MB PSRAM, SD card |
| STM32↔ESP32 link | 2 wires | ESP32-P4 GPIO 27/28 (UART3-IN) ↔ STM32 PA9/PA10 (USART1) @ 576k baud |
| STM32 programmer (optional) | ST-LINK V2/V3 | Optional; only needed for initial Katapult install or deep debugging |
| Remote tier (optional) | Raspberry Pi / PC | Runs the Flask dashboard, connects over WiFi/TCP |
| Power stage | Supercap bank + INA226 sensors | 4S pack monitoring, weld/balance/charger control |

> **Legacy hardware:** The original build used a Sunton ESP32-8048S043C (ESP32-S3)
> with GPIO 17/18 @ 2 Mbaud. Current production uses ESP32-P4 with GPIO 27/28 @ 576k baud.

---

## Build Instructions

Both firmwares use **PlatformIO**.

### STM32 firmware
```bash
cd STM32G474CE
pio run -e g474ceu6_stlink            # build
pio run -e g474ceu6_stlink -t upload  # flash via ST-LINK (required)
```
- Output binary: `STM32G474CE/.pio/build/g474ceu6_stlink/firmware.bin`
- Requires the ARM GCC toolchain (PlatformIO installs it automatically).

### ESP32 firmware
```bash
cd ESP32_8048S043C
pio run                # build
pio run -t upload      # flash over USB
```
- Combined image and `firmware.bin` are produced under
  `ESP32_8048S043C/.pio/build/esp32-8048S043C/`.
- The ESP32 can also self-update from the SD card: copy `firmware.bin` to the
  card as `/esp32_firmware.bin` and trigger the update from the Setup tab.

### Putting firmware on the SD card
- `/esp32_firmware.bin` — ESP32 update image (SD update **works**).
- `/stm32_firmware.bin` — STM32 update image (SD update **unreliable**; use
  ST-LINK — see [KNOWN_ISSUES.md](docs/issues/KNOWN_ISSUES.md)).

---

## Repository Layout

| Path | Contents |
|---|---|
| `STM32G474CE/` | STM32 controller firmware (PlatformIO) |
| `ESP32_8048S043C/` | ESP32-S3 display / bridge firmware (PlatformIO) |
| `Spotwelder Full/` | Earlier combined ESP32 project (reference) |
| `docs/` | All documentation — see [docs/README.md](docs/README.md) (hardware, firmware, features, issues) |
| `PROTOCOL.md` | Serial/TCP telemetry protocol reference |
| `AGENTS.md` | Build commands + workspace map for contributors/agents |
| `Datasheets/` | Component datasheets |
| `archive/old_firmware/` | Archived known-good firmware binaries |
| `docs/issues/KNOWN_ISSUES.md` | **STM32 SD-flash limitation — read this** |

---

## Further Reading

- **[KNOWN_ISSUES.md](docs/issues/KNOWN_ISSUES.md)** — STM32 SD-flash limitation in full.
- `docs/firmware/ESP32_ARCHITECTURE_ANALYSIS.md` — deep dive on the ESP32 firmware.
- `docs/hardware/PIN_ASSIGNMENTS.md` — authoritative STM32 pin map.
- `docs/hardware/FIRMWARE_MEASUREMENT_ANALYSIS.md` — measurement / ADC architecture.
- `docs/features/SD_AND_STM32_FLASHING_FEASIBILITY.md` — original feasibility study for the
  SD-flash feature.
- **[PROTOCOL.md](PROTOCOL.md)** — serial/TCP telemetry protocol reference.
- **[AGENTS.md](AGENTS.md)** — build commands and workspace map.
