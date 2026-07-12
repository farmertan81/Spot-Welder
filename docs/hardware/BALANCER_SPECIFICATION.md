# 4-Cell Li-ion Balancer Specification

> **Project:** Capacitive Discharge Spot Welder  
> **MCU:** STM32G474CE  
> **Author:** JQ-DHJ  
> **Status:** Hardware Design Phase  
> **Last Updated:** 2026-05-10

---

## Overview

This document specifies the passive cell-balancing subsystem for the spot welder's
4S Li-ion battery pack. The balancer bleeds excess energy from higher-voltage cells
through resistive loads switched by optocoupler-isolated N-channel MOSFETs. Each cell
channel is independently controlled by a dedicated STM32G474CE GPIO pin (PB3–PB6),
enabling per-cell PWM or on/off balancing under firmware control.

**Design Goals:**
- Balance current ≈ 1.26 A per cell (fast passive balancing)
- Full galvanic isolation between MCU logic and cell potentials via optocouplers
- Thermal headroom with 3× paralleled 1 W sense resistors per channel
- Simple firmware interface — one GPIO per cell, active-high enable

---

## Hardware Components

### Bill of Materials

| Ref | Part Number | Description | Value / Rating | Qty per Cell | Qty × 4 Cells | Notes |
|-----|-------------|-------------|----------------|:------------:|:--------------:|-------|
| U1 | TLP291-4 | Quad optocoupler (SOP-16) | CTR ≥ 100% @ 5 mA | ¼ pkg | 1 pkg total | One package covers all 4 channels |
| Q1–Q4 | IRLML6244TRPBF | N-ch MOSFET SOT-23 | 20 V, 6.3 A, R_DS(on) 21 mΩ | 1 | 4 | Logic-level gate, V_GS(th) 0.8 V typ |
| R_BAL | CSRT2512FT10R0-UP | Current-sense resistor 2512 | 10 Ω, 1 W, 1% | 3 | 12 | 3 in parallel → 3.33 Ω effective |
| R_LED | — | LED current-limit resistor 0805 | 470 Ω, ¼ W | 1 | 4 | Sets I_F ≈ 5.6 mA @ 3.3 V logic |
| R_GS | — | Gate pull-down resistor 0805 | 100 kΩ, ¼ W | 1 | 4 | Ensures MOSFET off when opto inactive |
| R_COLL | — | Collector pull-up / gate-drive resistor 0805 | 100 Ω, ¼ W | 1 | 4 | Limits collector current, speeds turn-off |

### Circuit Design — Per-Cell Channel

```
 STM32 GPIOx ──[ 470 Ω ]──►|── TLP291 LED ──► GND (MCU side)
                                │
                          ─ ─ isolation barrier ─ ─
                                │
   Cell+ ──[ 100 Ω ]── TLP291 Collector ─┬─ IRLML6244 Gate
                        TLP291 Emitter ──GND│   │
                                            └─[100 kΩ]─ GND (cell-ref)
                                                │
   Cell+ ──┬──[ 10 Ω ]──┤
            ├──[ 10 Ω ]──┼── IRLML6244 Drain
            └──[ 10 Ω ]──┘
                              IRLML6244 Source ── Cell−
```

**Operating Principle:**
1. MCU drives GPIO HIGH → current flows through 470 Ω into the TLP291 IR LED (~5.6 mA).
2. The optocoupler's phototransistor saturates, pulling the MOSFET gate toward the cell
   positive rail through the 100 Ω collector resistor.
3. The IRLML6244 turns on (V_GS ≈ V_cell ≈ 4.2 V, well above 0.8 V threshold), 
   connecting the 3× paralleled 10 Ω resistors across the cell.
4. Balance current flows: I_bal = V_cell / R_eff ≈ 4.2 V / 3.33 Ω ≈ 1.26 A.
5. MCU drives GPIO LOW → LED off → phototransistor off → 100 kΩ pull-down
   discharges gate → MOSFET turns off → no balance current.

---

## GPIO Pin Assignment

### STM32G474CE Pins

| GPIO Pin | AF | Cell | Direction | Notes |
|----------|--------|------|-----------|-------|
| **PB3** | GPIO_Output | Cell 1 (bottom) | Push-pull, no pull | TLP291-4 channel 1 LED anode via 470 Ω |
| **PB4** | GPIO_Output | Cell 2 | Push-pull, no pull | TLP291-4 channel 2 LED anode via 470 Ω |
| **PB5** | GPIO_Output | Cell 3 | Push-pull, no pull | TLP291-4 channel 3 LED anode via 470 Ω |
| **PB6** | GPIO_Output | Cell 4 (top) | Push-pull, no pull | TLP291-4 channel 4 LED anode via 470 Ω |

### Pin Mapping Summary

```
PB3  →  TLP291 Ch.1  →  Q1 (IRLML6244)  →  Cell 1  (most negative cell)
PB4  →  TLP291 Ch.2  →  Q2 (IRLML6244)  →  Cell 2
PB5  →  TLP291 Ch.3  →  Q3 (IRLML6244)  →  Cell 3
PB6  →  TLP291 Ch.4  →  Q4 (IRLML6244)  →  Cell 4  (most positive cell)
```

> **PB3 Note:** PB3 defaults to JTDO/SWO after reset. Firmware must explicitly
> reconfigure it as GPIO_Output. This is safe because the project uses only SWD
> (SWCLK/SWDIO on PA13/PA14) — JTAG/SWO is not required.

---

## Electrical Specifications

### Balance Current

| Parameter | Formula | Value |
|-----------|---------|-------|
| Effective load resistance | 10 Ω ∥ 10 Ω ∥ 10 Ω | **3.33 Ω** |
| MOSFET R_DS(on) contribution | 21 mΩ @ V_GS = 4.5 V | Negligible (< 1%) |
| Total path resistance | R_eff + R_DS(on) | ≈ 3.35 Ω |
| Balance current @ 4.20 V | 4.20 V / 3.35 Ω | **1.254 A** |
| Balance current @ 3.60 V | 3.60 V / 3.35 Ω | **1.075 A** |
| Balance current @ 3.00 V | 3.00 V / 3.35 Ω | **0.896 A** |

### Power Dissipation

#### Per Resistor (CSRT2512FT10R0-UP)

| Parameter | Formula | Value | Rating | Margin |
|-----------|---------|-------|--------|--------|
| Current per resistor @ 4.2 V | 1.254 A / 3 | **0.418 A** | — | — |
| Power per resistor @ 4.2 V | 0.418² × 10 Ω | **1.748 W** | 3.5 W | **50% derating — OK** |
| Power per resistor @ 3.6 V | 0.358² × 10 Ω | **1.285 W** | 3.5 W | **63% derating — OK** |

> **Note on resistor rating:** The CSRT2512FT10R0-UP is rated 1 W at standard
> conditions per the original BOM. If using the Vishay CSRT2512 series rated at
> 3.5 W (with proper thermal management / copper pour), the margin is excellent.
> Verify the exact part procured against its datasheet. At 1 W rating, 1.748 W
> **exceeds** the limit — the 3.5 W variant or equivalent is **required**.

#### Per MOSFET (IRLML6244TRPBF)

| Parameter | Formula | Value | Rating | Margin |
|-----------|---------|-------|--------|--------|
| Conduction loss @ 4.2 V | 1.254² × 0.021 Ω | **33 mW** | 1.3 W (SOT-23) | 97% margin |
| Junction temp rise | 33 mW × 200 °C/W (θ_JA) | **+6.6 °C** | T_J max 150 °C | Negligible |

#### Total System Power (All 4 Cells Active)

| Parameter | Value |
|-----------|-------|
| Total balance power @ 4.2 V | 4 × (4.2 V × 1.254 A) = **21.1 W** |
| Total balance power @ 3.6 V | 4 × (3.6 V × 1.075 A) = **15.5 W** |

> ⚠️ **Thermal Warning:** 21 W of resistive dissipation requires adequate airflow
> or heatsinking. Do not run all 4 channels continuously without thermal validation.

### Performance Metrics — Balance Time Estimates

Assuming a 100 mV cell imbalance at top-of-charge (4.20 V vs. 4.10 V) with a
typical 18650 cell (≈ 2500 mAh):

| Parameter | Value |
|-----------|-------|
| Energy to remove (approx.) | ΔV × C_nom ≈ 0.1 V × 2.5 Ah = **0.25 Wh** |
| Balance power per cell | ≈ 5.27 W @ 4.2 V |
| Estimated balance time | 0.25 Wh / 5.27 W ≈ **2.8 minutes** |
| For 50 mV imbalance | ≈ **1.4 minutes** |
| For 200 mV imbalance | ≈ **5.7 minutes** |

> These are rough estimates. Actual balance time depends on cell capacity, ESR,
> temperature, and the balancing algorithm (continuous vs. pulsed).

---

## Firmware Implementation

### GPIO Initialization

```c
/* ---------- balancer GPIO initialization ---------- */
/* Call from main() after HAL_Init() and SystemClock_Config() */

static void Balancer_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIOB clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Pre-set pins LOW before configuring as output (balancers OFF) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 |
                              GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = GPIO_PIN_3 | GPIO_PIN_4 |
                            GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* Balancing is slow — low slew is fine */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
```

### Control Functions

```c
/* ---------- balancer control API ---------- */

#define BAL_CELL1_PIN  GPIO_PIN_3
#define BAL_CELL2_PIN  GPIO_PIN_4
#define BAL_CELL3_PIN  GPIO_PIN_5
#define BAL_CELL4_PIN  GPIO_PIN_6
#define BAL_PORT       GPIOB

/* Enable / disable a single cell balancer (cell = 1..4) */
void balancer_set(uint8_t cell, uint8_t enable)
{
    uint16_t pin;
    switch (cell) {
        case 1: pin = BAL_CELL1_PIN; break;
        case 2: pin = BAL_CELL2_PIN; break;
        case 3: pin = BAL_CELL3_PIN; break;
        case 4: pin = BAL_CELL4_PIN; break;
        default: return;  /* invalid cell */
    }
    HAL_GPIO_WritePin(BAL_PORT, pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Disable all balancers (emergency / init) */
void balancer_all_off(void)
{
    HAL_GPIO_WritePin(BAL_PORT,
        BAL_CELL1_PIN | BAL_CELL2_PIN | BAL_CELL3_PIN | BAL_CELL4_PIN,
        GPIO_PIN_RESET);
}

/* Read current balancer state bitmask (bit 0 = cell 1 … bit 3 = cell 4) */
uint8_t balancer_get_state(void)
{
    uint8_t state = 0;
    if (HAL_GPIO_ReadPin(BAL_PORT, BAL_CELL1_PIN)) state |= 0x01;
    if (HAL_GPIO_ReadPin(BAL_PORT, BAL_CELL2_PIN)) state |= 0x02;
    if (HAL_GPIO_ReadPin(BAL_PORT, BAL_CELL3_PIN)) state |= 0x04;
    if (HAL_GPIO_ReadPin(BAL_PORT, BAL_CELL4_PIN)) state |= 0x08;
    return state;
}
```

### Balancing Algorithm

> **TODO — Implement and document the chosen strategy.**

Candidate strategies:

| Strategy | Description | Pros | Cons |
|----------|-------------|------|------|
| **Top-balance (threshold)** | Bleed any cell > target (e.g., 4.18 V) when pack is charging | Simple, safe | Only active near top of charge |
| **Delta-balance** | Bleed cells whose voltage exceeds (V_min + threshold) | Balances at any SoC | Needs accurate per-cell voltage |
| **PWM duty-cycle** | Modulate balance current proportional to ΔV | Smoother thermal profile | More complex firmware |
| **Time-sliced** | Round-robin enable, one cell at a time | Limits total power | Slower balance |

**Recommended initial implementation:**
1. Start with simple **threshold top-balance** during charging.
2. Add **delta-balance** once per-cell INA226 readings are validated.
3. Consider **PWM** for thermal management if continuous balancing causes overheating.

**Safety rules (must implement):**
- Disable all balancers if any cell < 3.0 V (prevent over-discharge).
- Disable all balancers if any cell temperature > 60 °C (if temp sense available).
- Disable all balancers during weld pulse (avoid noise / current transients).
- Maximum continuous balance time per cell: configurable timeout (default 30 min).
- Firmware watchdog must disable balancers on MCU fault / hang.

---

## PCB Layout Guidelines

### Trace Requirements

| Parameter | Recommendation |
|-----------|---------------|
| Balance current traces (cell+ to resistors to MOSFET drain/source) | **≥ 40 mil (1.0 mm)** width, 1 oz Cu → ~1.5 A capacity |
| Optocoupler LED traces (MCU side) | 10 mil (0.25 mm) — low current (~6 mA) |
| Gate drive traces | 10 mil — low current, short runs |
| Ground plane | Solid pour on cell-side ground; **separate** from MCU-side ground (isolation!) |

### Thermal Management

| Technique | Details |
|-----------|---------|
| Copper pour under resistors | Fill zone on both top and bottom layers beneath each CSRT2512 trio |
| Thermal vias | Array of 0.3 mm vias (min 6 per resistor) connecting top and bottom copper |
| Resistor spacing | ≥ 2 mm between parallel resistors for airflow |
| Board edge clearance | Keep resistors ≥ 5 mm from board edge for heatsink contact option |

### Component Placement

```
  ┌──────────────────────────────────────────────┐
  │  [TLP291-4]                                  │
  │   (MCU side traces — isolated)               │
  │──────────── isolation gap (≥ 2mm) ──────────│
  │                                              │
  │  Q1 [IRLML6244]   R1a R1b R1c  ← Cell 1    │
  │  Q2 [IRLML6244]   R2a R2b R2c  ← Cell 2    │
  │  Q3 [IRLML6244]   R3a R3b R3c  ← Cell 3    │
  │  Q4 [IRLML6244]   R4a R4b R4c  ← Cell 4    │
  │                                              │
  │  [ Cell connections along board edge ]       │
  └──────────────────────────────────────────────┘
```

- Place the TLP291-4 with its internal isolation barrier aligned with the
  PCB isolation slot/gap.
- Keep MCU-side traces (LED drive) physically separated from cell-side traces.
- Route cell power traces as short and wide as possible.
- Place 100 kΩ gate pull-downs immediately adjacent to each MOSFET gate pin.

---

## Implementation Status

### Hardware
- [x] Schematic concept complete
- [x] Component selection finalized (BOM)
- [x] GPIO pins allocated (PB3–PB6 verified available)
- [ ] Schematic capture in EDA tool
- [ ] PCB layout complete
- [ ] Design rule check (DRC) passed
- [ ] Components ordered
- [ ] PCB fabricated
- [ ] Board assembled
- [ ] Hardware bench test — continuity & isolation
- [ ] Hardware bench test — balance current verification

### Firmware
- [ ] GPIO initialization code integrated into `main.c`
- [ ] Basic on/off control functions
- [ ] Balancer state reporting to ESP32 / UI
- [ ] Per-cell voltage monitoring integration (INA226 cross-check)
- [ ] Threshold top-balance algorithm
- [ ] Delta-balance algorithm
- [ ] Safety interlock — disable during weld pulse
- [ ] Safety interlock — under-voltage cutoff
- [ ] Safety interlock — thermal cutoff (if temp sense available)
- [ ] Safety interlock — watchdog timeout
- [ ] Full integration testing
- [ ] Long-duration balance test (thermal soak)

---

## Testing & Validation

### Test Procedures

> **TODO — Expand each step with specific pass/fail criteria.**

1. **Isolation Test**
   - Verify > 1 MΩ between MCU-side and cell-side traces with multimeter.

2. **GPIO Verification**
   - Toggle each PB3–PB6 pin; confirm optocoupler LED current ≈ 5.6 mA.

3. **MOSFET Switching**
   - With resistor load disconnected, measure gate voltage when opto is active.
   - Expect V_GS ≈ V_cell (3.0–4.2 V range).

4. **Balance Current**
   - Connect a bench supply set to 4.20 V per cell.
   - Enable one channel; measure current through load resistors.
   - Expected: **1.25 ± 0.05 A**.

5. **Thermal Validation**
   - Run one channel for 10 minutes at 4.20 V.
   - Measure resistor surface temperature with thermocouple.
   - Pass criteria: T_surface < 100 °C in still air.

6. **All-Channel Stress Test**
   - Enable all 4 channels simultaneously for 5 minutes.
   - Monitor for thermal runaway, solder joint integrity.
   - Verify MCU remains responsive.

7. **Safety Interlock Test**
   - Simulate low-voltage condition; confirm balancers disable.
   - Trigger weld pulse; confirm balancers disable during pulse.

### Test Results

| Test | Date | Result | Notes |
|------|------|--------|-------|
| — | — | — | _No tests performed yet_ |

---

## Future Enhancements

| Enhancement | Priority | Description |
|-------------|----------|-------------|
| PWM-modulated balancing | Medium | Use TIM2/TIM3 channels on PB3–PB6 for proportional current control |
| Temperature sensing | Medium | Add NTC per cell or IR sensor for thermal feedback |
| Active balancing | Low | Replace resistive bleed with charge-shuttle topology for higher efficiency |
| Balance current sensing | Low | Add low-side shunt per channel for closed-loop current measurement |
| BLE / Wi-Fi reporting | Low | Stream per-cell balance status to phone app |
| Auto-calibration | Low | Measure actual balance current via INA226 and adjust strategy |

---

## Key Datasheet References

| Component | Datasheet | Key Parameters |
|-----------|-----------|----------------|
| IRLML6244TRPBF | Infineon IRL40SC228 family / IRLML6244 | V_DS 20 V, I_D 6.3 A, R_DS(on) 21 mΩ, V_GS(th) 0.8 V typ |
| TLP291-4 | Toshiba TLP291-4 | 4-ch optocoupler, CTR ≥ 100%, I_F 5–20 mA, V_CE(sat) 0.2 V |
| CSRT2512FT10R0-UP | Vishay CSRT2512 series | 10 Ω, 1% tolerance, 3.5 W (with thermal management), 2512 package |
| STM32G474CE | ST DS12288 | PB3–PB6 GPIO, 3.3 V logic, push-pull output |

---

## Revision History

| Rev | Date | Author | Changes |
|-----|------|--------|---------|
| 0.1 | 2026-05-10 | — | Initial specification — hardware design phase. BOM finalized, GPIO assigned, electrical calculations complete. |

---

*This is a living document. Update as design progresses through schematic → layout → fabrication → firmware integration → validation.*
