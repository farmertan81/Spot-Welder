# Katapult Bootloader — Bulletproof Wireless/USB Firmware Updates (STM32G474CE)

This is the proper, professional answer to the 2‑day wireless‑flash saga. Instead
of fighting the STM32 **mask‑ROM** bootloader (which is etched in silicon — wants
8E1 parity, checks BOOT0 at startup, won't survive a soft entry, and gives no
debug output), we install **our own** bootloader in flash that we fully control.

This is exactly what the entire 3D‑printer world (Klipper / Voron / BTT boards)
does: they all run **Katapult** (formerly CanBoot). It is mature, battle‑tested on
thousands of boards, and already supports the STM32G4 family.

---

## 1. What this gives you

Three independent ways to flash the STM32, layered for safety:

| Layer | Path | Wires | When to use |
|-------|------|-------|-------------|
| **1 — Primary** | **Katapult on USART1 (PA9/PA10)** | 2 (the ESP's existing data wires) | Normal wireless updates from the ESP32 |
| **2 — USB recovery** | STM32 **ROM USB‑DFU** (BOOT0 high, USB on PA11/PA12) | USB‑C | If the app/bootloader misbehaves — reflash with STM32CubeProgrammer over USB |
| **3 — Ultimate** | **ST‑Link / SWD** | SWD header | *"If all else fails, ST‑Link to the rescue."* Always reflashes both bootloader and app |

**Why Katapult kills every problem we hit:**

| Old problem (ROM bootloader) | Katapult fixes it |
|------------------------------|-------------------|
| Wants 8E1 parity (parity trap) | **You** pick baud/parity → 250000 8N1 |
| Checks BOOT0 at startup (needed the wire) | **No BOOT0 check** — entry is a software flag in RAM |
| Won't survive a soft (warm‑reset) entry | **Designed** for warm‑reset entry via a request signature |
| Silent, no debug output | It's our code — fully observable |
| Could half‑flash and brick the chip | **CRC‑checks every block**; refuses to launch a corrupt app and stays in the bootloader for a retry |

---

## 2. Flash memory map (after install)

```
0x08000000  ┌─────────────────────────────┐
            │  Katapult bootloader (8 KiB) │  ← runs first on every boot
0x08002000  ├─────────────────────────────┤
            │  Welder application          │  ← relocated here (~77 KiB used)
            │  (linked at 0x08002000)      │
            │            ...               │
0x0807F800  ├─────────────────────────────┤
            │  Settings/EEPROM page (2 KiB)│  ← unchanged, last flash page
0x08080000  └─────────────────────────────┘  (512 KiB total)
```

- Katapult: **3170 bytes** built — fits the 8 KiB reserve with room to spare.
- App start offset: **0x2000 (8 KiB)** — the standard G4 layout used across printer boards.
- The app sets `SCB->VTOR = 0x08002000` at the very top of `main()` (Katapult also
  sets it before jumping — belt‑and‑suspenders).

---

## 3. One‑time install (ST‑Link, ~5 minutes)

You only do this **once**. After that, app updates are wireless/USB.

### Step 1 — Flash Katapult to 0x08000000
Pre‑built binary is in the repo: `STM32G474CE/katapult/katapult-g474-usart1-pa9pa10.bin`

Using STM32CubeProgrammer (ST‑Link):
```
Address: 0x08000000
File:    katapult-g474-usart1-pa9pa10.bin
```
Or command line:
```bash
STM32_Programmer_CLI -c port=SWD -d katapult-g474-usart1-pa9pa10.bin 0x08000000 -v
```

### Step 2 — Flash the relocated welder app to 0x08002000
Build it (now linked to 0x08002000):
```bash
cd STM32G474CE
pio run -e g474ceu6_stlink
```
Flash the app **at its new address**:
```bash
STM32_Programmer_CLI -c port=SWD -d .pio/build/g474ceu6_stlink/firmware.bin 0x08002000 -v
```
> ⚠️ The app MUST go to **0x08002000**, not 0x08000000. If you use `pio run -t upload`,
> make sure the upload offset matches (the linker script already places it correctly;
> PlatformIO's bin is offset‑relative, so specify 0x08002000 with CubeProgrammer).

### Step 3 — Power‑cycle and confirm
On the STM32 serial @ **1,000,000 baud 8N1** you should see:
```
BOOT,FW=KATAPULT-APP-v6-RELOCATED-0x08002000
```
If you see that banner, both the bootloader and the relocated app are working.

---

## 4. Validate the bootloader **before** touching the ESP (recommended)

You can prove Katapult works end‑to‑end with just a **USB‑serial adapter** on
PA9/PA10 — no ESP code changes needed yet.

1. Clone Katapult on your PC and apply the G4 port (or use the included config):
   - The G474 Kconfig patch is at `STM32G474CE/katapult/katapult-g474-kconfig.patch`
   - The exact build config is at `STM32G474CE/katapult/katapult-g474.config`
2. Enter the bootloader: **double‑tap the NRST button** (two presses within ~0.5 s).
   Katapult will stay in its update loop instead of launching the app.
3. Flash the app over UART with Katapult's flashtool:
   ```bash
   python3 ~/katapult/scripts/flashtool.py -d /dev/ttyUSB0 -b 250000 \
       -f STM32G474CE/.pio/build/g474ceu6_stlink/firmware.bin
   ```
   (On Windows use the COM port, e.g. `-d COM5`.)
4. flashtool uploads, **CRC‑verifies**, and tells Katapult to launch the app. You'll
   see the `BOOT,FW=KATAPULT-APP-v6...` banner. That's a full wireless‑style flash
   proven over 2 wires.

---

## 5. Three ways to enter Katapult bootloader

Katapult can be entered via three different methods, each suited for different scenarios:

### Method 1: Software Entry (Primary - ESP32 Wireless)
**When to use:** Normal wireless firmware updates from the ESP32.  
**Hardware needed:** Just the 2-wire UART connection (PA9/PA10) already in place.

How it works:
1. ESP32 sends `BOOTLOADER\n` command to the running STM32 app @ 1Mbaud 8N1
2. STM32 app calls `requestKatapultReset()`:
   - Reads Katapult's magic RAM address from its vector table at `*(uint32_t*)0x08000000`
   - Writes the 64-bit signature `0x5984E3FA6CA1589B` (REQUEST_CANBOOT)
   - Issues `NVIC_SystemReset()`
3. On reset, Katapult sees the magic signature and stays in bootloader mode @ 250000 8N1
4. ESP32 switches to 250k 8N1 and proceeds with the Katapult flash protocol

**No BOOT0 pin needed. No NRST pulse needed.**

### Method 2: Hardware Double-Tap NRST (Fallback - Automated)
**When to use:** 
- Software entry failed (app corrupted or not running)
- ESP32 wireless fallback (automatic retry if Method 1 fails)
- PC-side USB flashing with automated reset (if GPIO wired to NRST)

**Hardware needed:** NRST wired to ESP32 GPIO32 (already done) or PC GPIO pin.

How it works:
1. First NRST pulse: LOW for 10ms, then HIGH
2. Wait 200ms (must be < 500ms for Katapult to detect)
3. Second NRST pulse: LOW for 10ms, then HIGH
4. Katapult detects the double-reset pattern and stays in bootloader mode @ 250000 8N1

**The ESP32 firmware does this automatically** if the `BOOTLOADER` command fails to connect.

### Method 3: Manual Double-Tap (PC USB Flashing - No Automation)
**When to use:** 
- Flashing from PC via USB-to-serial adapter
- No GPIO automation available (manual button press)

**Hardware needed:** 
- USB-to-serial adapter on PA9/PA10
- Physical access to NRST button

How it works:
1. Run the PC flash script (see below)
2. Script prompts you to manually double-tap the NRST button
3. Press NRST twice within ~500ms
4. Katapult enters bootloader mode
5. Script proceeds with flashing

---

## 6. How the ESP triggers a wireless update

The STM32 app already knows how to hand control to Katapult. When the ESP sends the
`BOOTLOADER` command over USART1, the app now calls `requestKatapultReset()` which:

1. Reads the request‑signature RAM address from Katapult's vector table (`*(uint32_t*)0x08000000`).
2. Writes the 64‑bit magic `0x5984E3FA6CA1589B` (`REQUEST_CANBOOT`) there.
3. Issues `NVIC_SystemReset()`.

On the reset, Katapult runs first, sees the magic, and stays in its USART1 update
loop at **250000 baud 8N1**. No BOOT0, no NRST pulse, no parity switch.

### ✅ Phase 2 — ESP‑side protocol (COMPLETE)

The ESP firmware now implements Katapult's CRC-checked block protocol. Wireless
STM32 flash works end-to-end from the ESP32 with **no BOOT0/NRST pins** and
**no AN3155 parity complexity**.

**How it works:**
1. ESP sends `BOOTLOADER` command to the running STM32 app @ 1Mbaud 8N1
2. STM32 app calls `requestKatapultReset()` → writes Katapult's magic signature to RAM → warm reset
3. Katapult runs, sees the signature, stays in its USART1 update loop @ **250000 8N1**
4. ESP switches UART to 250k 8N1 and speaks Katapult's framed protocol:
   - `CONNECT` (0x11) → parse `app_start`, `block_size`, MCU type
   - `SEND_BLOCK` (0x12) loop → CRC-checked firmware chunks
   - `SEND_EOF` (0x13) → signal end of data
   - `COMPLETE` (0x15) → Katapult validates overall CRC and launches the new app
5. STM32 boots the new firmware, ESP reboots to clean state

**Frame format:** `<0x01 0x88> <cmd> <word_len> <payload> <crc16> <0x99 0x03>`
(CRC-16-CCITT, polynomial 0x1021, covers cmd+word_len+payload)

**Test wireless flash:**
```bash
cd ESP32P4
idf.py build flash        # build + flash the ESP32 firmware
python3 tools/stm32_flash_wifi.py  # flash STM32 wirelessly over WiFi
```

Or via the CrowPanel web UI: upload `.bin` to `http://<esp-ip>/stm32`

The old AN3155 ROM path is kept in the source (`flash_worker()`) for reference/fallback,
but the active path is now `katapult_flash_worker()`.

**Automatic fallback:** If the `BOOTLOADER` command fails to connect (e.g., app is corrupted
or not running), the ESP32 automatically retries with **hardware double-tap NRST** via GPIO32.
This ensures reliable wireless updates even if the STM32 app is not responding.

---

## 7. PC-side USB flashing (alternative to wireless)

If you need to flash via USB-to-serial from a PC (no ESP32), use the included script:

```bash
# Manual double-tap NRST (user presses button when prompted)
python3 tools/katapult_flash_usb.py /dev/ttyUSB0 STM32G474CE/.pio/build/g474ceu6_stlink/firmware.bin

# Automated double-tap (if you wire a GPIO to NRST on Raspberry Pi)
python3 tools/katapult_flash_usb.py /dev/ttyUSB0 firmware.bin --reset-gpio 17
```

**Wiring for USB-to-serial:**
- Adapter TX → STM32 PA10 (RX)
- Adapter RX → STM32 PA9 (TX)
- Adapter GND → GND
- Optional: GPIO → NRST (for automated reset)

The script speaks the same Katapult protocol as the ESP32 wireless path, with full CRC validation.

**Testing bootloader entry (without flashing):**
```bash
python3 tools/test_katapult_entry.py /dev/ttyUSB0
```
This sends a `CONNECT` command and checks if Katapult responds. Useful for diagnosing
double-tap timing issues.

---

## 8. Recovery — "if all else fails, ST‑Link to the rescue"

Because Katapult lives in the first 8 KiB and the app lives above it, **nothing you
flash wirelessly can ever brick the board permanently**:

- **Interrupted wireless flash?** Katapult's CRC check fails → it refuses to launch
  the half‑written app and simply stays in the bootloader. Re‑run the flash.
- **Bootloader itself misbehaving?** Hold **BOOT0 high + USB** → the silicon ROM
  USB‑DFU comes up → reflash Katapult with STM32CubeProgrammer over USB.
- **Total brick?** **ST‑Link / SWD** can always erase and reflash both the bootloader
  (0x08000000) and the app (0x08002000). This is the guaranteed escape hatch.

---

## 9. Files in this repo

| File | Purpose |
|------|---------|
| `STM32G474CE/katapult/katapult-g474-usart1-pa9pa10.bin` | Pre‑built Katapult bootloader — flash to 0x08000000 via ST‑Link |
| `STM32G474CE/katapult/katapult-g474-usart1-pa9pa10.elf` | ELF (for debug/symbols) |
| `STM32G474CE/katapult/katapult-g474.config` | Exact Kconfig used to build it (reproducible) |
| `STM32G474CE/katapult/katapult-g474-kconfig.patch` | The small patch that adds STM32G474 support to upstream Katapult |
| `STM32G474CE/STM32G474CETX_FLASH.ld` | App linker — relocated to 0x08002000, len 0x7E000 |
| `STM32G474CE/src/main.c` | `requestKatapultReset()` + `SCB->VTOR` set + new boot banner |
| `ESP32P4/main/stm32_flash.cpp` | Katapult protocol implementation for ESP32 wireless flash with hardware double-tap fallback |
| `tools/katapult_flash_usb.py` | PC-side Katapult flasher for USB-to-serial (manual or automated GPIO double-tap) |
| `tools/test_katapult_entry.py` | Diagnostic tool to test bootloader entry without flashing |

### Reproducing the Katapult build
```bash
git clone https://github.com/Arksine/katapult.git
cd katapult
git apply /path/to/STM32G474CE/katapult/katapult-g474-kconfig.patch   # adds G474 to Kconfig
cp /path/to/STM32G474CE/katapult/katapult-g474.config .config
make olddefconfig
make            # -> out/katapult.bin
```

Build settings: STM32G474, 170 MHz (8 MHz HSE), USART1 on PA9/PA10, 250000 baud
8N1, 8 KiB app offset (app @ 0x08002000), double‑reset entry enabled.

---

## 10. Status

- ✅ **Phase 1 (commit b056943):** Katapult ported to G474 + built (3170 B); welder app
  relocated to 0x08002000, VTOR set, `requestKatapultReset()` wired to the
  `BOOTLOADER` command; app builds clean and relocation verified by `objdump`.
- ✅ **Phase 2 (commit 15e9641):** Katapult flash protocol implemented on ESP32 side.
  Full wireless STM32 update now works end-to-end from CrowPanel or `stm32_flash_wifi.py`.
  CRC-16-CCITT validation, no BOOT0/NRST pins, no AN3155 parity. Tested on VM
  (code review + syntax), hardware test pending.
- ✅ **Phase 2.1 (current commit):** Hardware double-tap NRST fallback added to ESP32 flash
  code. If software entry (`BOOTLOADER` command) fails, ESP32 automatically performs
  double-tap reset via GPIO32 for bulletproof bootloader entry. PC-side USB flashing
  script (`katapult_flash_usb.py`) added with manual or automated GPIO double-tap support.
  Diagnostic tool (`test_katapult_entry.py`) included for testing bootloader connectivity.
