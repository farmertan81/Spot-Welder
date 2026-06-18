# ESP32-P4 UI Stable Baseline (v1.0-ui-stable)

**Date:** 2026-06-18  
**Tag:** `v1.0-ui-stable`  
**Branch:** Dev

## Summary
CrowPanel ESP32-P4 5" spot-welder firmware now has a **fully working UI** ported from the OLD ESP32_8048S043C board, with all critical bugs fixed. The UI correctly syncs with the STM32 controller, displays accurate values, and applies settings without flicker or mapping errors.

---

## What's Working ✅
- **Display:** RGB LCD rendering stable (fixed timing issues, bounce buffer)
- **Touch:** GT911 I2C touch fully operational
- **Dashboard:** Live voltage, current, temperature, energy readings
- **Pulse Tab:**
  - Single/Double/Triple mode selection
  - Duration/gap spinboxes with hold-to-repeat (+/- buttons)
  - Power % slider (50-100%)
  - Preheat settings (enable/duration/power/gap)
  - **Apply button** with correct visual feedback (RED=applied, GREEN=pending)
- **Joule Tab:** Target energy and max-time controls (defaults to 50J)
- **Calibrate Tab:** Lead resistance measurement
- **Settings Tab:** Pedal/contact trigger modes, system info
- **Recipe Sync:** ESP32 ↔ STM32 bidirectional communication working
- **Defaults:** Sensible boot values (power 80%, joule 50J, preheat off)

---

## Critical Bugs Fixed This Session 🐛
1. **Stuck-green Apply button** (root cause: preheat_pct spinbox min=10 but STM32 had preheat_pct=3, causing permanent draft≠applied mismatch)
   - Fix: clamp all STM32 values to spinbox ranges before sync
2. **Power reverting to 80% after reset** (blocked by stuck-dirty loop preventing STM32 STATUS sync)
   - Fix: same clamp fix above broke the dirty loop
3. **Mode mapping scrambled** (SET_PULSE sent mode LAST but STM32 expected it FIRST, so d1=10 → mode=10 → triple)
   - Fix: reordered SET_PULSE fields to match STM32 parser
4. **Mode flicker on Apply** (Triple → instant applied_mode=3 → stale STM32 STATUS overwrites to 1 → back to 3)
   - Fix: removed instant sync, let STM32 STATUS echo be single source of truth

---

## Known Constraints
- **User builds on Windows** (ESP32: `idf.py -p COM6 flash monitor` / STM32: PlatformIO `g474ceu6_stlink`)
- **No VM compilation** for this embedded project (ARM toolchains, flash hardware required)
- Git version control enforced; all changes committed to Dev branch

---

## Architecture
- **ESP32-P4:** UI render (LVGL 8.x), USB serial ↔ STM32, display/touch drivers (ESP-IDF v6.0.1)
- **STM32G474CEU6:** Weld control loop, current sensing (INA226), flash persistence, UART protocol (PlatformIO/HAL)
- **Protocol:** Plain-text UART commands (SET_PULSE, SET_POWER, SET_PREHEAT, STATUS, etc.)
- **Repo:** `farmertan81/Spot-Welder`, branch `Dev`

---

## Next Steps (Roadmap)
1. **Flask web UI integration** — remote control/monitoring via WiFi
2. **OTA updates** — ESP32-P4 and STM32 firmware update over-the-air
3. **Field testing** — verify weld quality/repeatability with production hardware

---

## Tag Purpose
This tag marks a **clean baseline** before adding Flask/OTA complexity. If those features introduce regressions, revert to this tag for a known-good UI-only build.

**Commit:** `faf3ca0` — "ESP32P4: fix mode-select flicker on Apply (revert to STM32-echo-only pattern)"
