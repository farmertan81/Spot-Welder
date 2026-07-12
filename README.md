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
- **STM32 update via ST-LINK** — flashing the STM32 with an ST-LINK programmer
  works every time and is the supported path (see below).

---

## ⚠️ Known Limitation: STM32 SD-Card Flashing Requires ST-LINK

Updating the **STM32** firmware from the SD card (driving the chip into its ROM
bootloader purely in software, over the existing UART link) is **not reliable**
on this hardware. **To update the STM32 firmware, use an ST-LINK programmer.**

**Root cause (short version):** the Sunton ESP32-8048S043C board exposes **no
free GPIO** that can be wired to the STM32 `BOOT0` pin, and the STM32G4 ROM
bootloader does not come up USART-responsive after a software-only jump. Without
a `BOOT0` control line, there is no fully reliable wire-free way to force the
ROM bootloader.

➡️ **Full details, analysis, and the future fix are in
[KNOWN_ISSUES.md](docs/issues/KNOWN_ISSUES.md).**

### Workaround — flashing the STM32 with an ST-LINK

1. Connect an **ST-LINK V2/V3** to the STM32 SWD header (SWDIO, SWCLK, GND, and
   3V3 reference).
2. Build the STM32 firmware (see below) — the binary is produced at
   `STM32G474CE/.pio/build/g474ceu6_stlink/firmware.bin`.
3. Flash with PlatformIO:
   ```bash
   cd STM32G474CE
   pio run -e g474ceu6_stlink -t upload
   ```
   …or with STM32CubeProgrammer (load `firmware.bin` at address `0x08000000`).
4. The **ESP32** still updates from SD normally — only the STM32 needs the
   ST-LINK.

> An **experimental option-byte** SD-flash path exists in the firmware (the
> STM32 reprograms its boot option bytes to force the ROM bootloader, then
> restores them on the next boot). It is **unverified on hardware** — keep an
> ST-LINK handy as the guaranteed recovery path. See docs/issues/KNOWN_ISSUES.md.

---

## Hardware Requirements

| Component | Part | Notes |
|---|---|---|
| Controller MCU | **STM32G474CEU6** | 512 KB Flash, single bank; ROM bootloader at `0x1FFF0000` |
| Display / bridge | **Sunton ESP32-8048S043C** | ESP32-S3, 4.3" 800×480 RGB, GT911 touch, PSRAM, microSD |
| STM32↔ESP32 link | 2 wires | ESP32 GPIO18 → STM32 PA10 (RX); STM32 PA9 → ESP32 GPIO17 (RX) |
| **STM32 programmer** | **ST-LINK V2/V3** | **Required** to flash the STM32 firmware (see limitation above) |
| Remote tier (optional) | Raspberry Pi | Runs the Flask dashboard, connects over WiFi/TCP |
| Power stage | Supercap bank + INA226 sensors | 4S pack monitoring, weld/balance/charger control |

> **No BOOT0 / NRST control wires** exist between the ESP32 and STM32 — every
> exposed ESP32 header pin is already consumed by the RGB LCD, GT911 touch I²C,
> the STM32 UART, the microSD SPI bus, or the USB debug UART.

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
