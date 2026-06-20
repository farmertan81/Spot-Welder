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

## 5. How the ESP triggers a wireless update

The STM32 app already knows how to hand control to Katapult. When the ESP sends the
`BOOTLOADER` command over USART1, the app now calls `requestKatapultReset()` which:

1. Reads the request‑signature RAM address from Katapult's vector table (`*(uint32_t*)0x08000000`).
2. Writes the 64‑bit magic `0x5984E3FA6CA1589B` (`REQUEST_CANBOOT`) there.
3. Issues `NVIC_SystemReset()`.

On the reset, Katapult runs first, sees the magic, and stays in its USART1 update
loop at **250000 baud 8N1**. No BOOT0, no NRST pulse, no parity switch.

### ⚠️ Phase 2 — ESP‑side protocol (still to do)

The ESP firmware currently speaks the **ROM's AN3155 protocol** (0x7F sync, 8E1,
etc.). Katapult uses its **own protocol** (see `katapult/protocol.md`). To flash
wirelessly from the ESP, the ESP side must implement Katapult's protocol (or the
simplest path: have the ESP shell out to / port `flashtool.py`'s logic):

- Open USART1 at **250000 8N1**
- Send the `BOOTLOADER` command so the app resets into Katapult
- Speak Katapult's framed protocol: `CONNECT` → `REQUEST_BLOCK` writes → `EOF` → CRC verify → `COMPLETE`
- Katapult validates CRC and launches the app

This Phase‑2 work is **not yet implemented**. Until then, use the USB‑serial +
`flashtool.py` path in section 4 (which is the reference implementation of that
exact protocol) to flash, and ST‑Link as the ultimate fallback.

---

## 6. Recovery — "if all else fails, ST‑Link to the rescue"

Because Katapult lives in the first 8 KiB and the app lives above it, **nothing you
flash wirelessly can ever brick the board permanently**:

- **Interrupted wireless flash?** Katapult's CRC check fails → it refuses to launch
  the half‑written app and simply stays in the bootloader. Re‑run the flash.
- **Bootloader itself misbehaving?** Hold **BOOT0 high + USB** → the silicon ROM
  USB‑DFU comes up → reflash Katapult with STM32CubeProgrammer over USB.
- **Total brick?** **ST‑Link / SWD** can always erase and reflash both the bootloader
  (0x08000000) and the app (0x08002000). This is the guaranteed escape hatch.

---

## 7. Files in this repo

| File | Purpose |
|------|---------|
| `STM32G474CE/katapult/katapult-g474-usart1-pa9pa10.bin` | Pre‑built Katapult bootloader — flash to 0x08000000 via ST‑Link |
| `STM32G474CE/katapult/katapult-g474-usart1-pa9pa10.elf` | ELF (for debug/symbols) |
| `STM32G474CE/katapult/katapult-g474.config` | Exact Kconfig used to build it (reproducible) |
| `STM32G474CE/katapult/katapult-g474-kconfig.patch` | The small patch that adds STM32G474 support to upstream Katapult |
| `STM32G474CE/STM32G474CETX_FLASH.ld` | App linker — relocated to 0x08002000, len 0x7E000 |
| `STM32G474CE/src/main.c` | `requestKatapultReset()` + `SCB->VTOR` set + new boot banner |

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

## 8. Status

- ✅ **Phase 1 (this commit):** Katapult ported to G474 + built (3170 B); welder app
  relocated to 0x08002000, VTOR set, `requestKatapultReset()` wired to the
  `BOOTLOADER` command; app builds clean and relocation verified by `objdump`.
- ⏳ **Phase 2 (next):** Implement Katapult's flash protocol on the ESP32 so the
  full wireless update runs from the CrowPanel. Until then, validate with
  `flashtool.py` over USB‑serial (section 4).
