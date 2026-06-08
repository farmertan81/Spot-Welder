# STATUS Tab UI Improvements

**Files changed:** `Spotwelder Full/src/ui.cpp`, `Spotwelder Full/src/main.cpp`,
`Spotwelder Full/include/ui.h`
**Status:** Compile-verified only (PlatformIO build SUCCESS). **NOT hardware-tested.**

Four improvements requested after confirming Joule mode works (99.66% accuracy).

---

## 1. Moved "Joules" to the Last Weld section (STATUS tab)

The live workpiece energy is now shown in the **Last Weld** box on the STATUS
tab, so you can see it while welding without switching to the Joule tab.

- The Last Weld box now shows **Duration | Peak | Avg | Joules** (was
  Target | Actual | Duration | Peak).
- "Joules" displays the latched workpiece energy from the last weld
  (`last_weld_energy_j`), formatted as `49.8 J`.
- The live Joules readout on the Joule tab was **kept** (no harm; useful when
  tuning on that tab).

## 2. Fixed the trigger display

`paint_dash_trigger()` previously showed a delay time for *both* modes
(e.g. `Pedal  (1.0s)`). Now:

- **Pedal mode** → shows just `Pedal` (pedal has no delay).
- **Contact mode** → shows `Contact 1.0s` (with the configured probe delay).

## 3. Weld counter now persists across restarts

The counter used to reset to 0 on every ESP32 reboot. It is now stored in NVS
(ESP32 `Preferences`) in a dedicated `weldstats` namespace:

- **On boot:** `load_weld_count_from_nvs()` restores the saved value.
- **On each weld** (`EVENT,WELD_DONE`): incremented, then saved.
- **On tap-to-reset** (`RESET_WELD_COUNT`): set to 0, then saved.

So the lifetime weld count survives power cycles and only changes on a real
weld or a user reset.

## 4. Added Average Current to Last Weld

The STM32 already reports `avg_a=` in the `EVENT,WELD_DONE` frame, so **no
STM32 firmware change was needed** — the ESP32 just parses and displays it.

- New field `last_weld_avg_a` in `WelderDisplayState` (`ui.h`).
- `main.cpp` parses `avg_a=` from `WELD_DONE` and forwards it to the UI state.
- New **Avg** cell shows it as `3819 A`.

---

## Final Last Weld layout

```
LAST WELD
Duration: 15 ms   Peak: 3980 A   Avg: 3819 A   Joules: 49.8 J
```

---

## Build & flash

ESP32 display firmware (PlatformIO):

```bash
cd "Spotwelder Full"
pio run                 # build (verified SUCCESS)
pio run -t upload       # flash over USB (set upload_port if needed)
```

No STM32 firmware change in this task — only the ESP32 display firmware needs
re-flashing.

## Test checklist (on hardware)

1. Flash ESP32. STATUS tab Last Weld box should read `--` for all four cells
   until the first weld.
2. Fire a weld → Duration, Peak, Avg, and Joules should populate.
3. Trigger line: in Pedal mode shows `Pedal`; long-press to switch to Contact →
   shows `Contact X.Xs`.
4. Note the weld count, power-cycle the ESP32 → count should be restored (not 0).
5. Tap the weld counter to reset → reads 0; power-cycle → still 0.
