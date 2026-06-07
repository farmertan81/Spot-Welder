# Joule Mode Fix — Deliver Target Energy to the WORKPIECE

**File changed:** `STM32G474CE/src/main.c`
**Status:** Compile-verified only (PlatformIO build SUCCESS). **NOT hardware-tested** — bench-verify on real hardware.

---

## TL;DR

Your 50 J target only delivered ~34 J to the workpiece. The energy *math* on
the STM32 was already correct — it was **the pulse duration** that was wrong.
In Joule mode the weld was firing for the **Pulse-tab time durations**
(`d1/d2/d3`), not for `joule_max_ms`. The energy check could only *stop a pulse
early*, never *extend* it — so with your heavy lead losses the short pulse
ended before the workpiece ever reached target.

---

## Answering your question: "Was Flask doing the lead-loss compensation?"

**No.** Flask never did (and physically cannot do) energy control.

- Flask only **sends the target** (`SET_JOULE_TARGET`) and **displays** the
  energy numbers it receives back in the STM32 STATUS frame
  (`joule_workpiece_j`, `joule_total_j`, `joule_loss_j`, etc.).
- Energy integration happens on a **per-sample (100 µs)** basis during the
  weld. That control loop *must* live on the STM32 — there is no way to do
  millisecond-level pulse energy control over WiFi/TCP from Flask.
- "Flask working" earlier was about **settings sync** (the dirty-group fix from
  the previous task), **not** energy control. The two are unrelated.

So the STM32 is, and must remain, the single source of truth for energy. The
fix belongs entirely on the STM32 — which is where it now is.

---

## The STM32 was already integrating WORKPIECE energy correctly

The integration loop (inside `capturePulseAmpsForDurationUs`) already does the
right thing — it subtracts the lead loss every sample:

```c
total_power_w     = v_cap_live * amps;                 // power leaving the caps
lead_loss_power_w = amps * amps * lead_resistance_ohms; // I²R lost in the leads
workpiece_power_w = total_power_w - lead_loss_power_w;  // = V_tips × I  ✅
joule_accumulated += workpiece_power_w * joule_dt_sec;  // WORKPIECE energy
// stop when joule_accumulated >= joule_target_j - JOULE_OVERSHOOT_COMP
```

`(V_cap·I − I²·R_lead)` is mathematically identical to `V_tips·I` — i.e. the
power actually dissipated in the workpiece. This is correct and was **not**
changed.

---

## The actual bug: pulse duration

`fireRecipe()` fires the main weld as phases driven by the **time-mode**
durations:

```c
doPulseMsPwm(weld_d1_ms, ...);   // phase 1  ← time-mode setting!
doPulseMsPwm(weld_d2_ms, ...);   // phase 2
doPulseMsPwm(weld_d3_ms, ...);   // phase 3
```

The Joule energy check runs *inside* the pulse and can only **break early**
when the target is hit. It can never make the pulse longer than the configured
`weld_d1_ms`. So when your leads burn ~90 % of the energy, the workpiece never
reaches 50 J before the short configured pulse simply runs out → you get 34 J.

`joule_max_ms` (the field you set in the UI) was only used as an in-loop safety
timeout and for status reporting — it **never actually governed the pulse
length**.

---

## The fix

In Joule mode the weld is now **one continuous pulse** whose length is governed
by `joule_max_ms` (capped at `MAX_WELD_MS = 200`), and the energy integrator
stops it early the moment the workpiece reaches target.

1. **`get_planned_active_pulse_ms()`** — in Joule mode the waveform-capture
   window is sized by `joule_max_ms` instead of `d1+d2+d3`.

2. **Phase 1 main pulse** — fires for `joule_max_ms` (not `weld_d1_ms`) when in
   Joule mode:
   ```c
   const bool joule_mode = (control_mode == 1U);
   uint16_t joule_pulse_ms = (uint16_t)joule_max_ms;
   if (joule_pulse_ms > MAX_WELD_MS) joule_pulse_ms = MAX_WELD_MS;
   const uint16_t main1_ms = joule_mode ? joule_pulse_ms : weld_d1_ms;
   ...
   doPulseMsPwm(main1_ms, ...);
   ```

3. **Phases 2 & 3 are skipped** in Joule mode (`&& !joule_mode` added to both
   conditions). Joule mode is a single pulse — the multi-phase d2/d3/gap
   structure does not apply, and this also closes a race where TIM2 kills the
   FET exactly at `joule_max_ms` before the in-loop timeout flag is set.

4. **Timeout reporting** — if the single pulse ends without reaching target
   (and without a bad-contact abort), `joule_timeout_abort` is set so STATUS
   correctly reports **TIMEOUT**, telling you to raise `joule_max_ms` or reduce
   lead losses.

---

## Expected behavior after the fix

- Set **50 J** → the pulse now runs (up to `joule_max_ms`) until the
  **workpiece** actually absorbs ~50 J, then stops.
- **Total energy drawn from the caps will be higher** than 50 J (your lead
  losses are added on top — at 90 % loss, total could be 400 J+). That is
  expected and correct: the target is workpiece energy, not cap energy.
- The pulse will be **longer** than the old time-mode duration.

### ⚠️ Important: set `joule_max_ms` high enough
With ~90 % lead loss, reaching 50 J at the workpiece needs a lot of total
energy and therefore time. The default `joule_max_ms` of 40 ms may still be too
short — if you see **TIMEOUT** with the workpiece below target, **raise
`joule_max_ms`** (e.g. 100–200 ms). 200 ms is the firmware hard cap
(`MAX_WELD_MS`).

### ⚠️ Advisory: 90 % lead loss is extreme
`lead_r = 2.055 mΩ` causing ~90 % loss is very high. Either the lead-resistance
calibration is inflated, or the leads are genuinely very lossy. The fix makes
the target *reachable*, but efficiency stays poor — consider shorter / thicker
leads or re-checking the lead-resistance calibration. (Advisory only — not a
code issue.)

---

## Testing instructions (on hardware)

1. Flash the built firmware (`.pio/build/g474ceu6_stlink/firmware.bin`).
2. Set Joule mode, target 50 J, `joule_max_ms` = 150 ms.
3. Fire a weld. Confirm the live **Joules** readout climbs to ~50 J at the
   workpiece and the pulse stops itself.
4. Check status: should report normal completion (not TIMEOUT) if target met.
5. If TIMEOUT with workpiece < 50 J → raise `joule_max_ms`.
6. Compare total-from-caps vs workpiece energy to gauge real lead efficiency.
