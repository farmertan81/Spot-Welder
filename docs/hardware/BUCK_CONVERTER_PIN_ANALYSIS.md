# Synchronous Buck Converter — Pin & Timer Analysis

**Date:** 2025-05-10
**MCU:** STM32G474CE (48-pin package, UFQFPN48 or LQFP48)
**Firmware:** `STM32G474CE/src/main.c`

---

## 1. Current Pin Usage Summary

| Pin   | Function              | Peripheral   | AF   | Notes                              |
|-------|-----------------------|-------------|------|------------------------------------|
| PA0   | Analog input          | ADC          | —    | Shunt/analog sensing               |
| PA1   | Analog input          | ADC          | —    | Shunt/analog sensing               |
| PA2   | Analog input          | ADC1_CH2     | —    | AMC1302 current sense (P)          |
| PA3   | Analog input          | ADC1_CH3/CH4 | —    | AMC1302 current sense (N) + Vcap P |
| PA4   | Analog input          | ADC2_CH17    | —    | AMC1311B Vcap (N)                  |
| PA8   | **TIM1_CH1**          | TIM1         | AF6  | **Welding PWM — DO NOT TOUCH**     |
| PA9   | USART1_TX             | USART1       | AF7  | ESP32 communication — USED         |
| PA10  | USART1_RX             | USART1       | AF7  | ESP32 communication — USED         |
| PA13  | SWDIO                 | Debug        | AF0  | SWD programming — avoid            |
| PA14  | SWCLK                 | Debug        | AF0  | SWD programming — avoid            |
| PA15  | I2C1_SCL              | I2C1         | AF4  | INA226 sensors — USED              |
| PB2   | Charger enable        | GPIO output  | —    | Controls buck charger MOSFET       |
| PB7   | I2C1_SDA              | I2C1         | AF4  | INA226 sensors — USED              |
| PB12  | Pedal input           | GPIO input   | —    | Weld trigger — USED                |
| PC6   | Status LED            | GPIO output  | —    | UFQFPN48 only (not on LQFP48)     |

### Reserved for Balancer (Spec'd, not yet implemented in firmware)

| Pin   | Function              | Notes                              |
|-------|-----------------------|------------------------------------|
| PB3   | Balancer Cell 1       | TLP291 optocoupler drive           |
| PB4   | Balancer Cell 2       | TLP291 optocoupler drive           |
| PB5   | Balancer Cell 3       | TLP291 optocoupler drive           |
| PB6   | Balancer Cell 4       | TLP291 optocoupler drive           |

### Timer Usage

| Timer | Channel | Function             | Pin  | Status       |
|-------|---------|----------------------|------|--------------|
| TIM1  | CH1     | Welding PWM          | PA8  | **ACTIVE**   |
| TIM1  | CH2-CH4 | —                    | —    | Available    |
| TIM2  | —       | µs delay / FET-kill  | None | **ACTIVE** (internal, no output pin) |
| TIM8  | All     | —                    | —    | **UNUSED**   |

---

## 2. Requirements for Synchronous Buck Converter

A synchronous buck converter needs:
1. **High-side MOSFET gate:** PWM signal (active during t_ON)
2. **Low-side MOSFET gate:** Complementary PWM (active during t_OFF)
3. **Dead-time insertion:** Prevents shoot-through (both MOSFETs ON simultaneously)
4. **Break input (optional):** Hardware overcurrent shutdown

STM32G4 advanced timers (TIM1, TIM8) provide all of these via:
- CHx (main output) → High-side gate driver
- CHxN (complementary output) → Low-side gate driver
- BDTR register → Programmable dead-time
- BKINx → Hardware break/fault input

---

## 3. Available Options Analysis

### Constraints

- **TIM1_CH1 (PA8)** is dedicated to welding — cannot share timer period/frequency
- **PA9/PA10** are USART1 — TIM1_CH2/CH3 main outputs blocked
- **PA15** is I2C1_SCL — TIM8_CH1 (AF2) blocked
- **PB3-PB6** reserved for balancer — TIM8_CH1/CH1N (PB6/PB3) conflicted
- **PC6-PC9** not available on LQFP48 (only on UFQFPN48 for PC6)
- **PA13/PA14** are SWD debug — avoid

### Option Evaluation

#### ✅ Option 1: TIM8_CH3 (PB9) + TIM8_CH3N (PB1) — **RECOMMENDED**

```
High-Side MOSFET: TIM8_CH3  on PB9 (AF4)
Low-Side MOSFET:  TIM8_CH3N on PB1 (AF4)

Timer: TIM8
Channel: CH3 + CH3N (complementary pair)
Alternate Function: AF4 for both pins
```

| Criteria                    | Rating    | Notes                                           |
|-----------------------------|-----------|------------------------------------------------|
| Pin availability            | ✅ FREE    | Neither pin used in firmware or balancer spec   |
| Timer independence          | ✅ Excellent| TIM8 completely separate from TIM1 (welding)   |
| Dead-time support           | ✅ Yes     | TIM8 BDTR register, same as TIM1               |
| Break input available       | ✅ Yes     | TIM8_BKIN on PA6 (AF4) — also free             |
| Package availability        | ✅ Both    | PB9 and PB1 on both LQFP48 and UFQFPN48       |
| Same AF number              | ✅ AF4     | Both pins use AF4 — clean configuration        |
| No balancer conflict        | ✅ None    | PB3-PB6 untouched                              |
| No debug conflict           | ✅ None    | SWD pins untouched                             |
| Physical proximity          | ⚠️ Moderate| PB1=pin 17/18, PB9=pin 46/47 on package       |

**Why TIM8 over TIM1?**
- TIM1 is the welding timer — its period (ARR=16999, ≈10 kHz) is optimized for MOSFET gate driving at high current. The buck converter needs a completely different frequency (50-200 kHz).
- Using a separate timer avoids any interaction between welding and charging.
- TIM8 has identical advanced timer capabilities (dead-time, break, complementary outputs).

#### ⚠️ Option 2: TIM8_CH1 (PB6) + TIM8_CH1N (PB3) — Conflicts with Balancer

```
High-Side MOSFET: TIM8_CH1  on PB6 (AF4)
Low-Side MOSFET:  TIM8_CH1N on PB3 (AF4)
```

| Criteria                    | Rating    | Notes                                           |
|-----------------------------|-----------|------------------------------------------------|
| Pin availability            | ⚠️ Planned | PB3/PB6 reserved for balancer in spec          |
| Timer independence          | ✅ Excellent| TIM8 separate from TIM1                        |
| Physical proximity          | ✅ Good    | PB3 and PB6 close on package                   |
| Balancer conflict           | ❌ Yes     | Would require balancer pin reassignment         |

*Only viable if balancer is relocated to other pins (e.g., PC13-PC15 as GPIO or TIM3/TIM4 channels).*

#### ❌ Option 3: TIM1_CH2 (PA9) + TIM1_CH2N (PA12) — UART Conflict

```
High-Side MOSFET: TIM1_CH2  on PA9 (AF6)
Low-Side MOSFET:  TIM1_CH2N on PA12 (AF6)
```

| Criteria                    | Rating    | Notes                                           |
|-----------------------------|-----------|------------------------------------------------|
| Pin availability            | ❌ Blocked | PA9 = USART1_TX (ESP32 link)                   |
| Timer shared with welding   | ❌ Bad     | Same TIM1, different frequency requirements     |

*Would require relocating USART1 AND sharing TIM1 (different ARR impossible). Not recommended.*

---

## 4. Recommended Configuration Detail

### Pin Configuration Code

```c
/* ============ Buck Converter Pins ============ */
#define BUCK_TIM          TIM8
#define BUCK_TIM_CH       TIM_CHANNEL_3
#define BUCK_HS_PORT      GPIOB
#define BUCK_HS_PIN       GPIO_PIN_9    /* TIM8_CH3  = High-side gate */
#define BUCK_LS_PORT      GPIOB
#define BUCK_LS_PIN       GPIO_PIN_1    /* TIM8_CH3N = Low-side gate  */

/* Optional: Overcurrent break input */
#define BUCK_BKIN_PORT    GPIOA
#define BUCK_BKIN_PIN     GPIO_PIN_6    /* TIM8_BKIN (AF4)            */
```

### GPIO Initialization

```c
static void MX_BUCK_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PB9 = TIM8_CH3 (High-side gate driver) */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;       /* Safe default: gate LOW */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB1 = TIM8_CH3N (Low-side gate driver) */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
```

### TIM8 Initialization (100 kHz example)

```c
TIM_HandleTypeDef htim8;

static void MX_TIM8_Buck_Init(void) {
    __HAL_RCC_TIM8_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /*
     * Buck converter frequency calculation:
     * f_PWM = f_TIM / (PSC+1) / (ARR+1)
     *
     * For 100 kHz @ 170 MHz:
     *   PSC = 0, ARR = 1699  →  170MHz / 1 / 1700 = 100.00 kHz
     *
     * For 200 kHz @ 170 MHz:
     *   PSC = 0, ARR = 849   →  170MHz / 1 / 850  = 200.00 kHz
     *
     * For 50 kHz @ 170 MHz:
     *   PSC = 0, ARR = 3399  →  170MHz / 1 / 3400 = 50.00 kHz
     */
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;  /* Center-aligned recommended for buck */
    htim8.Init.Period = 849;                   /* 200 kHz center-aligned (effective switching freq) */
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
        /* Handle error */
    }

    /* Channel 3: PWM Mode 1 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;                       /* Start at 0% duty */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;    /* Safe: gate LOW when idle */
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;  /* Safe: gate LOW when idle */

    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        /* Handle error */
    }

    /*
     * Dead-time calculation for IRL40SC228:
     *
     * Dead-time = DTG × t_DTS
     * t_DTS = 1 / f_TIMER_CLK = 1/170MHz ≈ 5.88 ns
     *
     * IRL40SC228 typical switching times:
     *   t_r (rise)  ≈ 27 ns
     *   t_f (fall)  ≈ 14 ns
     *   t_d(on)     ≈ 12 ns
     *   t_d(off)    ≈ 42 ns
     *
     * Conservative dead-time = t_d(off) + t_f + margin
     *                        ≈ 42 + 14 + 44 = 100 ns
     *
     * DTG encoding (for DTG[7:5] = 0xx):
     *   Dead-time = DTG[7:0] × t_DTS
     *   For 100ns: DTG = 100 / 5.88 ≈ 17 → use DTG = 0x11 (17)
     *   Actual dead-time = 17 × 5.88 = 100.0 ns
     *
     * For 200ns (more conservative):
     *   DTG = 200 / 5.88 ≈ 34 → use DTG = 0x22 (34)
     *   Actual dead-time = 34 × 5.88 = 200.0 ns
     */
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0x22;              /* ~200ns dead-time (conservative) */
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;  /* Enable if using BKIN */
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
        /* Handle error */
    }
}
```

### Start/Stop Functions

```c
/* Start buck converter PWM with complementary outputs */
static void buckStart(uint16_t duty) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);  /* Start complementary output */
}

/* Stop buck converter - both gates LOW (safe) */
static void buckStop(void) {
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
}

/* Update duty cycle (0 to htim8.Init.Period) */
static inline void buckSetDuty(uint16_t duty) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty);
}
```

---

## 5. Frequency & Duty Cycle Quick Reference

| Target f_sw | ARR  | Max Duty | Dead-time (200ns) | Effective Duty Range |
|-------------|------|----------|-------------------|---------------------|
| 50 kHz      | 3399 | 3399     | 34 counts         | 34–3365 (1%–99%)   |
| 100 kHz     | 1699 | 1699     | 34 counts         | 34–1665 (2%–98%)   |
| 150 kHz     | 1132 | 1132     | 34 counts         | 34–1098 (3%–97%)   |
| 200 kHz     | 849  | 849      | 34 counts         | 34–815 (4%–96%)    |

> **Note:** Center-aligned mode doubles the effective switching frequency for the same ARR value, halving inductor ripple. For 100 kHz effective switching with center-aligned: ARR = 1699.

---

## 6. Package Pin Locations

### UFQFPN48 (if using QFN package)

```
PB1  = Pin 18  (bottom edge)
PB9  = Pin 47  (left edge)
PA6  = Pin 14  (bottom edge, BKIN option)
```

### LQFP48 (if using LQFP package)

```
PB1  = Pin 17  (bottom-left area)
PB9  = Pin 46  (top-right area)
PA6  = Pin 13  (bottom-left area)
```

> ⚠️ PB1 and PB9 are on opposite sides of the package. Route the gate driver (Si827x) close to the MOSFETs and run short traces from the MCU to the driver inputs. The Si827x has high dV/dt immunity so trace length to the driver is less critical than driver-to-gate distance.

---

## 7. Break Input for Overcurrent Protection (Optional)

If hardware overcurrent shutdown is needed:

```c
/* PA6 = TIM8_BKIN (AF4) — unused pin, available */

/* GPIO config */
GPIO_InitStruct.Pin = GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_PULLDOWN;  /* Default: no fault */
GPIO_InitStruct.Alternate = GPIO_AF4_TIM8;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* Enable break in TIM8 BDTR */
sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;  /* Active high = fault */
sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;  /* Auto-restart after fault clears */
```

When BKIN goes HIGH, TIM8 immediately forces both outputs to their idle state (both gates LOW), providing <100ns hardware overcurrent response — much faster than software ADC polling.

---

## 8. Bonus: HRTIM Alternative (Future Consideration)

The STM32G474 has a **High-Resolution Timer (HRTIM)** with 184 ps resolution — ideal for power conversion. Key advantages:
- 184 ps dead-time resolution (vs 5.88 ns for TIM8)
- Up to 6 complementary pairs
- Built-in cycle-by-cycle current limiting
- ADC trigger synchronization

HRTIM pins on PB12-PB15 could be explored if higher precision is needed, but TIM8 is more than sufficient for a basic synchronous buck at 50-200 kHz.

---

## 9. Final Recommendation

```
┌─────────────────────────────────────────────────────┐
│  RECOMMENDED PIN ASSIGNMENT                          │
│                                                      │
│  High-Side MOSFET gate: PB9  (TIM8_CH3,  AF4)      │
│  Low-Side MOSFET gate:  PB1  (TIM8_CH3N, AF4)      │
│  Break/Fault input:     PA6  (TIM8_BKIN, AF4)      │
│                                                      │
│  Timer: TIM8 (independent from welding TIM1)        │
│  Dead-time: ~200 ns (DTG=0x22, adjustable)          │
│  Frequency: 100-200 kHz (ARR=1699-849)              │
│  Mode: Center-aligned PWM1                           │
│                                                      │
│  Zero conflicts with existing firmware.              │
│  Zero conflicts with planned balancer (PB3-PB6).    │
│  SWD debug fully preserved (PA13/PA14).             │
└─────────────────────────────────────────────────────┘
```
