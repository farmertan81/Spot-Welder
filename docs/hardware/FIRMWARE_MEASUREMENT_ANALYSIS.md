# STM32G474CE Firmware — Measurement Architecture Analysis

> **Date:** 2026-06-02
>
> **Source:** `STM32G474CE/src/main.c` (4336 lines, Dev branch)
>
> **Purpose:** Read-only analysis for informed hardware/calibration discussion

---

## 1) Project Structure

```
Spot-Welder/
├── STM32G474CE/
│   ├── src/
│   │   ├── main.c                    ← All firmware logic (4336 lines, monolith)
│   │   └── stm32_settings_flash.c    ← Flash persistence with CRC32
│   └── include/
│       └── stm32_settings_flash.h    ← PersistentSettings struct definition
├── Spotwelder Full/                  ← ESP32 display firmware (PlatformIO)
│   └── src/main.cpp
├── docs/
│   ├── PIN_ASSIGNMENTS.md            ← Master pin map (25 pins allocated)
│   ├── BALANCER_SPECIFICATION.md
│   ├── BUCK_CONVERTER_PIN_ANALYSIS.md
│   └── README.md
└── Datasheets/                       ← 21 component PDFs
```

All STM32 firmware is in a **single monolithic `main.c`** — no HAL CubeMX `.ioc` file, no separate peripheral drivers. Everything is hand-coded bare-metal with HAL calls.

---

## 2) Analog Front-End: Two Isolated Amplifiers

### 2.1 Current Sensing — U3 (AMC1301/AMC1311 — label ambiguity)

**Schematic:** U3 "U_AMP_WELD" with net label "BN&_AMC1311_B0"

**Code defines (lines 120–123):**
```c
/* Physical chain: Amps → µV across shunt → ×41 (AMC1302) → ADC counts */
#define SHUNT_GAIN 8.2f          /* AMC1301 fixed gain                  */
#define SHUNT_EFF_OHMS 0.000050f /* 50 µΩ effective shunt (calibrated) */
```

**⚠️ Naming inconsistency in code comments:**
| Location | Chip referenced |
|---|---|
| Line 121 comment | "AMC1302" (gain ×41) |
| Line 122 `#define` comment | "AMC1301" (gain ×41) |
| Schematic net label | "AMC1311" |
| Actual `SHUNT_GAIN` value | **8.2** (matches neither ×41 nor ×2) |

**The `SHUNT_GAIN = 8.2` doesn't match any TI AMC part's native gain.** This is likely an empirically tuned value that absorbs the actual amplifier gain, voltage divider ratios, and PCB parasitics into a single calibration constant. The CURRENT_CAL_FACTOR (1.76×) further corrects the end-to-end chain.

**Signal path:**
```
Weld current → 50µΩ shunt → U3 isolated amp → differential output
  SHUNT_P → PA1 (ADC1_IN2)
  SHUNT_N → PA2 (ADC1_IN3)
```

**Current calculation (line 1210–1212):**
```c
float v_adc = (float)diff * v_per_count;      // diff = ADC_P - ADC_N
float v_shunt = v_adc / SHUNT_GAIN;           // Back-calculate shunt voltage
float amps = (v_shunt / SHUNT_EFF_OHMS) * CURRENT_CAL_FACTOR;  // 1.76x correction
```

**Full conversion chain:**
```
ADC counts → differential voltage → ÷ SHUNT_GAIN(8.2) → shunt µV
→ ÷ SHUNT_EFF_OHMS(50µΩ) → raw amps → × CAL_FACTOR(1.76) → calibrated amps
```

### 2.2 Capacitor Voltage Sensing — U2 (AMC1311B)

**Schematic:** U2 "U_AMP_VCAP" — clearly labeled AMC1311B

**Code references (line 114, 3534–3538):**
```c
/* Contact detection uses AMC1311B capacitor voltage sensing on PA3/PA4. */

/* AMC1311B OUTP (Pin 7) → PA3 → ADC1_IN4  (VOLT_AIN0)
 * AMC1311B OUTN (Pin 6) → PA4 → ADC2_IN17 (VOLT_AIN1) */
```

**Signal path:**
```
Cap bank voltage → external resistor divider (÷6) → U2 AMC1311B
  VCAP_OUTP → PA3 (ADC1_IN4)
  VCAP_OUTN → PA4 (ADC2_IN17)   ← NOTE: PA4 is ADC2 only!
```

**Voltage calculation (line 3550–3577):**
```c
float vP = ((float)sumP / valid) * (vdda / 4095.0f);
float vN = ((float)sumN / valid) * (vdda / 4095.0f);
float v = (vP - vN) * V_CAP_DIVIDER;   // V_CAP_DIVIDER = 6.0
```

**Key design detail:** The code uses **16-sample averaging** for stable cap voltage reads (`readCapVoltage()`), but during fast weld sampling uses single-shot reads via `adcReadFastTriplet()` for real-time Vcap tracking.

**Bug fix history:** PA4 was originally incorrectly mapped as ADC1_IN5 (which is PB14). Fixed to use ADC2_IN17.

---

## 3) ADC Architecture

### 3.1 ADC1 (Primary — current + voltage positive leg)

**Init:** `MX_ADC1_Init()` at line 3253
- 12-bit, PCLK/4, software trigger, single-ended
- Factory calibration run at init (`HAL_ADCEx_Calibration_Start`)

**Channels used:**
| Channel | Pin | Signal | Function |
|---|---|---|---|
| ADC1_IN1 | PA0 | THERM_IN | NTC thermistor |
| ADC1_IN2 | PA1 | SHUNT_P_ADC | Current sense + (AMC1301 OUTP) |
| ADC1_IN3 | PA2 | SHUNT_N_ADC | Current sense − (AMC1301 OUTN) |
| ADC1_IN4 | PA3 | VCAP_OUTP | Cap voltage + (AMC1311B OUTP) |
| VREFINT | internal | — | VDDA measurement (16× averaged) |

**Two operating modes:**
1. **Normal mode:** Single-channel sequential reads via `adcReadChannel()` — long sampling (247.5 cycles)
2. **Fast weld mode:** 3-channel scan (IN2 + IN3 + IN4) via `adcPrepareFastCurrentChannels()` — short sampling (47.5 cycles) for ~100µs sample rate during pulses

### 3.2 ADC2 (Secondary — voltage negative leg only)

**Init:** `MX_ADC2_Init()` at line 3287

**Single channel:**
| Channel | Pin | Signal | Function |
|---|---|---|---|
| ADC2_IN17 | PA4 | VCAP_OUTN | Cap voltage − (AMC1311B OUTN) |

**Why ADC2?** PA4 maps to ADC2_IN17 on STM32G474, not available on ADC1. This was a hardware constraint that required a dedicated ADC2 instance.

### 3.3 VDDA Calibration

`measureVDDA()` (line 3336) uses VREFINT factory calibration:
- 16× averaged VREFINT reads
- Factory cal value at `0x1FFF75AA` (measured at VDDA = 3.000V)
- Formula: `VDDA = (3.0V × factory_cal) / measured_raw`
- Sanity clamped to 2.5–3.6V range
- **Refreshed every 1 second** in main loop

---

## 4) INA226 Sensor Network (I2C1)

### 4.1 Hardware Configuration

**Bus:** I2C1 on PA15 (SCL) / PB7 (SDA), AF4
- Bus recovery (9× SCL toggle) runs before I2C init
- All sensors configured: AVG=64, VBUS_CT=1.1ms, VSH_CT=1.1ms, continuous mode
- Update period: ~141ms (64 × 1.1ms × 2)
- Read interval in main loop: 150ms

### 4.2 Sensors Currently Active in Firmware (3 of 5)

| Device | Address | Firmware Variable | Measures |
|---|---|---|---|
| U8 | **0x40** | `ina_vpack`, `ina_ichg` | Pack voltage + charge current (shunt) |
| U9 | **0x41** | `vlow` | Cell 1 tap (bottom of stack) |
| U10 | **0x44** | `vmid` | Cell 4 tap (top of stack, below pack+) |

### 4.3 Sensors in Schematic But NOT in Firmware

| Device | Address | Purpose |
|---|---|---|
| U13 | **0x42** | Cell 2 tap |
| U12 | **0x43** | Cell 3 tap |

**Firmware only initializes and reads 0x40, 0x41, 0x44.** The health check (`ina226_health_check`) only tests these three. The cell voltage math uses a 3S model:
```c
ina_cell1 = vlow;                  // Bottom cell
ina_cell2 = vmid - vlow;           // Middle cell
ina_cell3 = ina_vpack - vmid;      // Top cell
```

### 4.4 INA226 Calibration Scales

```c
#define V_NODE1_SCALE 1.001327f   // vlow (0x41) — calibrated 2025-05-08
#define V_NODE2_SCALE 0.986048f   // vmid (0x44) — calibrated 2025-05-08
#define VPACK_SCALE   1.001149f   // vpack (0x40) — calibrated 2025-05-08
```
Calibrated against Fluke 87V at vlow=2.904V, vmid=5.805V, vpack=8.71V.

### 4.5 INA226 Shunt Current

```c
#define INA226_SHUNT_R 0.0002f              // 200 µΩ charge shunt
#define CHARGE_CURRENT_CORRECTION 1.0f      // No correction applied
```
- Shunt voltage resolution: 2.5 µV/bit
- Current = (shunt_V / 200µΩ), sign-inverted, noise-floored at ±10mA

---

## 5) Charger Control

**Pin:** PB2 = `CHARGER_EN` (GPIO output, push-pull)

**⚠️ Documentation mismatch:** PB2 was recently removed from `PIN_ASSIGNMENTS.md` (marked as "available") but is **actively used in firmware** as `CHARGER_EN_PORT/CHARGER_EN_PIN`. The GPIO init at line 3168 explicitly configures PB2 as output and the charger state machine toggles it throughout the code (11 references).

**Charger state machine** (`chargerStateMachine()`, line 532):
- Hysteresis: ON below 8.7V, OFF at/above 9.11V
- Safety interlocks: OFF during welding, OFF if INA226 fails, OFF if disarmed
- Post-weld lockout period

---

## 6) Current Calibration Chain

The weld current measurement has a **two-factor calibration:**

### Factor 1: SHUNT_GAIN (8.2)
Absorbs the analog front-end gain. This appears to be an empirical value that doesn't match any AMC datasheet gain directly.

### Factor 2: CURRENT_CAL_FACTOR (1.76)
```c
/* Calibrated against 400F capacitance discharge test (2025-04-21):
 *   Discharge test: 8.74V -> 7.70V @30s -> 6.81V @60s, R=0.6Ω
 *   Result: C = 400F (confirmed via exponential decay)
 *   Weld test: ΔV=0.103V over 10ms on 400F caps => I_real = 4,120A
 *   Reported I = 2,339A => error = 1.76x too low */
#define CURRENT_CAL_FACTOR 1.76f
```

### Capacitor bank
```c
#define CAP_FARADS 413.0f   // Measured via drawdown: 8.98V→4.49V, R=6Ω, t=172.08s
```

---

## 7) Lead Resistance Handling

### 7.1 Runtime Variable
```c
static float lead_resistance_ohms = 0.0011f;  // 1.1 mΩ measured (4-wire Kelvin)
static const float LEAD_RESISTANCE_MIN_OHMS = 0.0001f;  // 0.1 mΩ
static const float LEAD_RESISTANCE_MAX_OHMS = 0.0100f;  // 10.0 mΩ
```

### 7.2 Flash Persistence
`PersistentSettings.lead_resistance_ohms` — saved/loaded via CRC-protected Flash page 255.

### 7.3 Joule Mode Usage (Real-Time Integration)
During weld pulses, the sampling loop (line 1241–1262) uses lead resistance for **workpiece energy** calculation:
```c
float total_power_w = v_cap_live * amps;                    // Vcap × I
float lead_loss_power_w = amps * amps * lead_resistance_ohms; // I²R lead
float workpiece_power_w = total_power_w - lead_loss_power_w;  // Net to workpiece

joule_total_accumulated += total_power_w * dt;
joule_lead_loss_accumulated += lead_loss_power_w * dt;
joule_accumulated += workpiece_power_w * dt;               // ← Controls cutoff
```

The Joule cutoff fires on `joule_accumulated` (workpiece energy), NOT total energy. This means lead resistance directly affects when the weld terminates.

### 7.4 UART Command Interface
Lead resistance can be set via UART command from ESP32 (parsed in `parseCommand()`).

---

## 8) Weld Pulse Timing Architecture

**TIM1:** PWM generation for MOSFET gate drive (PA8, TIM1_CH1)
- 170MHz / (0+1) / (16999+1) = **10 kHz PWM**
- Duty controlled via CCR1 register (0–1023 effective range mapped to 0–16999 ARR)

**TIM2:** Precision pulse duration timer (1 µs resolution)
- 170MHz / 170 = 1 MHz tick
- One-shot mode for pulse duration
- **Highest-priority ISR (priority 0,0)** — kills FET within ~70ns of timer overflow
- Also used for gap delay timing (without ISR)

**DWT CYCCNT:** Microsecond timestamps for waveform capture
- `micros_now() = DWT->CYCCNT / 170` (hardcoded for 170MHz)
- Rolls over every ~25 seconds

---

## 9) Waveform Capture System

4096-sample ring buffer capturing current + voltage at 100µs intervals:
```c
typedef struct {
    float current_amps;
    float voltage_volts;
    uint32_t timestamp_us;
} WaveformSample;
```

Features:
- PWM phase-sweep sampling (6 sub-samples per slot when duty < 100%)
- Phase boundary detection (preheat start/end, gap, main start/end)
- Chunked UART transmission (1024-byte chunks at 2 Mbps)
- Real-time Joule integration during capture
- Vcap tracking via simultaneous ADC1 (PA3) + ADC2 (PA4) fast reads

---

## 10) Key Discrepancies Found

| Item | Code Says | Schematic/Docs Say | Impact |
|---|---|---|---|
| Current sense chip | AMC1301/AMC1302 (comments) | AMC1311 (net label) | Comment-only; `SHUNT_GAIN=8.2` is empirical regardless |
| SHUNT_GAIN value | 8.2 | AMC1301=41×, AMC1311B=2× | Neither matches — value is calibration artifact |
| PB2 (CHARGER_EN) | Active in firmware (11 references) | Removed from PIN_ASSIGNMENTS.md | **Doc is wrong** — PB2 is in active use |
| INA226 sensors | 3 used (0x40, 0x41, 0x44) | 5 in schematic (+ 0x42, 0x43) | Firmware needs update for 4S cell monitoring |
| Cell math | 3S model (cell1/cell2/cell3) | 4S pack architecture | Missing cell4 in firmware |

---

## 11) Summary of Pin-to-Function Mapping (Analog)

| Pin | ADC | Channel | Signal | Chip | Function |
|---|---|---|---|---|---|
| PA0 | ADC1 | IN1 | THERM_IN | — | NTC thermistor |
| PA1 | ADC1 | IN2 | SHUNT_P_ADC | U3 (current amp) OUTP | Weld current + |
| PA2 | ADC1 | IN3 | SHUNT_N_ADC | U3 (current amp) OUTN | Weld current − |
| PA3 | ADC1 | IN4 | VCAP_OUTP | U2 (AMC1311B) OUTP | Cap voltage + |
| PA4 | ADC2 | IN17 | VCAP_OUTN | U2 (AMC1311B) OUTN | Cap voltage − |

---

*This is a read-only analysis. No code modifications were made.*
