# MODE Switcher (Status tab) + Joule Tab Cleanup

**Scope:** ESP32 display firmware only (`Spotwelder Full/src/ui.cpp`).
No STM32 firmware changes were required — the STM32 already handles
`SET_MODE,<0|1>` (see `STM32G474CE/src/main.c`, `SET_MODE` command handler)
and forwards `control_mode` in every `STATUS` packet.

**Status:** Compile-verified only (RAM 55.1%, Flash 24.2%). NOT hardware-tested.

---

## What changed

### 1. MODE switcher on the STATUS tab
The "MODE" box on the Status dashboard (top-left info strip) is now an
interactive **long-press switcher**, mirroring the existing TRIGGER switcher
right next to it:

- **Display:** shows `TIME` (accent color) or `JOULE` (green).
- **Short tap:** nothing (prevents accidental mode flips).
- **Long-press (~600 ms, same as TRIGGER):** toggles TIME ⇄ JOULE.
- **Visual feedback:** while held, the label turns yellow and the box
  background darkens — identical to the TRIGGER switcher.
- **Auto-apply (no dialog):** on toggle the change is applied immediately.

Implementation:
- New `btn_dash_mode` clickable object (replaces the old static MODE panel),
  built exactly like `btn_dash_trigger` and made touch-safe via
  `make_interaction_safe()`.
- New `paint_dash_mode(bool holding)` and `on_dash_mode_event()` handlers,
  modeled on `paint_dash_trigger()` / `on_dash_trigger_event()`.
- Reuses `TRIGGER_LONGPRESS_MS` (600 ms) for a consistent feel.

### 2. Auto-apply path (STM32 + Flask sync)
On a MODE toggle the handler reuses the existing Joule apply callback:

```
on_dash_mode_event()  ->  _joule_apply_cb(mode, target, max_ms)
                          (main.cpp: onJouleApply)
                            -> SET_MODE,<0|1>      (UART -> STM32)
                            -> SET_JOULE_TARGET    (UART -> STM32)
                            -> SET_JOULE_MAX       (UART -> STM32)
                            -> control_mode mirrored in ESP32 state
```

- **STM32** receives `SET_MODE`, switches mode, persists to flash, and echoes
  the new `control_mode` in the next `STATUS`.
- **Flask** sync is automatic: `control_mode` already rides the `STATUS`
  packet that `buildStatus()` sends over TCP, so the web dashboard reflects
  the new mode with no extra message type.
- **No fighting back:** the Status-tab mode-line sync now keeps the local
  `_joule_draft_mode` mirrored to the STM32's authoritative `control_mode`,
  but is gated by `_mode_longpress_fired` so an in-flight STATUS can't stomp a
  just-toggled value (same guard pattern the TRIGGER switcher uses).

### 3. Joule tab — APPLY button removed
- Deleted the green `APPLY` button, its `lbl_joule_apply` "APPLIED" label, and
  the `on_joule_apply()` handler. This removes the **"stuck on APPLIED" bug**.
- Settings now commit automatically: when the user lifts their finger off a
  +/- stepper (`LV_EVENT_RELEASED` / `LV_EVENT_PRESS_LOST`), a single
  `on_joule_setting_released()` fires and sends the current target + max-ms to
  the STM32. Sending on release (not per-step) means a hold-to-repeat ramp
  results in one commit, avoiding UART/flash churn.

### 4. Joule tab — TIME/JOULE toggle buttons removed
- Deleted the `TIME (ms)` / `JOULE (energy)` toggle buttons
  (`btn_joule_mode_time` / `btn_joule_mode_joule`) and their
  `on_joule_mode_time` / `on_joule_mode_joule` handlers.
- Mode is now controlled **only** from the Status-tab MODE switcher, so there
  is a single source of truth for the welding mode.

### 5. Joule tab — layout cleanup
With the mode toggles and APPLY button gone, the tab was reorganized:
- New **"ENERGY SETTINGS"** header + a sub-line:
  *"Switch TIME / JOULE on the STATUS tab (long-press the MODE box)."*
- **Target Energy** and **Max Duration** steppers now use the freed vertical
  space: larger value readouts (48 px font), wider value field (220 px), and
  taller +/- buttons (72 px).
- The explanatory note moved to a full-width line below the steppers and now
  mentions that changes apply automatically.

---

## Hold-to-repeat interaction (unchanged, still works)
The +/- steppers keep their hold-to-repeat behavior (gated by the Config
tab's "Hold to Repeat" option). The new release handler simply commits the
final value once the ramp ends.

---

## Hardware test checklist (pending — needs real hardware)
1. STATUS tab: MODE box shows the current mode correctly on boot.
2. Long-press MODE box ~0.6 s → toggles TIME ⇄ JOULE; label highlights yellow
   while held; flips on release.
3. STM32 receives the mode change (mode actually switches; persists across a
   power cycle).
4. Flask dashboard reflects the new mode within ~1 STATUS interval.
5. Joule tab: APPLY button is gone; no "stuck APPLIED" state.
6. Joule tab: TIME/JOULE toggle buttons are gone; layout looks clean.
7. Adjust Target Energy / Max Duration, lift finger → values commit to STM32
   (verify on Flask and/or STM32 logs); a hold-ramp commits once on release.
