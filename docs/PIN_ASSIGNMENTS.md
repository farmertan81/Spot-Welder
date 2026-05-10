# STM32G474CE Master Pin Assignment Document

> **Project:** Capacitive Discharge Spot Welder  
> **MCU:** STM32G474CEU6 — UFQFPN48 package  
> **Core:** ARM Cortex-M4F, 170 MHz, FPU, DSP  
> **Flash/RAM:** 512 KB / 128 KB  
> **Author:** JQ-DHJ  
> **Revision:** 1.0 — Final Hardware Configuration  
> **Date:** 2026-05-10

---

## Table of Contents

1. [Pin Summary](#pin-summary)
2. [Weld Control](#weld-control)
3. [Buck Charger](#buck-charger)
4. [Balancer (4-Cell)](#balancer-4-cell)
5. [Analog Inputs](#analog-inputs)
6. [Communication](#communication)
7. [Digital I/O](#digital-io)
8. [Debug Interface](#debug-interface)
9. [System / Power Pins](#system--power-pins)
10. [Complete Pin Table](#complete-pin-table)
11. [Available Pins for Future Expansion](#available-pins-for-future-expansion)
12. [Configuration Notes](#configuration-notes)
13. [Conflicts and Special Considerations](#conflicts-and-special-considerations)
14. [Cross-Reference](#cross-reference)

---

## Pin Summary

| Category               | Pins Used | Pin List                                          |
|------------------------|:---------:|---------------------------------------------------|
| Weld Control           | 3         | PA8, PB10, PA6                                    |
| Buck Charger           | 3         | PB9, PB1, PB8                                     |
| Balancer (4-cell)      | 4         | PB3, PB4, PB5, PB6                                |
| Analog Inputs          | 5         | PA0, PA1, PA2, PA3, PA4                            |
| USART1 (ESP32)         | 2         | PA9, PA10                                          |
| I2C1 (INA226 sensors)  | 2         | PA15, PB7                                          |
| Digital I/O            | 3         | PB2, PB12, PC6                                     |
| Debug (SWD)            | 2         | PA13, PA14                                         |
| **Total GPIO Used**    | **24**    |                                                    |
| System/Power           | 12        | VBAT, VREF+, VDDA, VDD×3, VSS, NRST, OSC×4       |
| **Total Pins Used**    | **36**    | of 48 package pins                                 |
| **Available for expansion** | **12** | See [Available Pins](#available-pins-for-future-expansion) |

---

## Weld Control

Controls two banks of 8× IRL40SC228 MOSFETs (16 total) via two UCC21520ADW isolated
half-bridge gate drivers. Each gate driver receives a single PWM/enable signal and drives
high-side and low-side outputs to 8 paralleled MOSFETs.

| Pin  | Pkg Pin | AF    | Peripheral | Net Name      | Direction | Description                          |
|------|:-------:|-------|------------|---------------|-----------|--------------------------------------|
| PA8  | 30      | AF6   | TIM1_CH1   | WELD_GATE_1   | Output    | PWM drive → UCC21520 U18 (MOSFETs Q3–Q10, Groups 1+2) |
| PB10 | 22      | AF6   | TIM1_CH3¹  | WELD_GATE_2   | Output    | PWM drive → UCC21520 U16 (MOSFETs Q11–Q18, Groups 3+4) |
| PA6  | 14      | GPIO  | —          | WELD_FAULT    | Input     | Fault feedback from weld gate drivers (active low) |

### Electrical Configuration — Weld Control

| Pin  | Mode             | Pull     | Speed    | Drive    | Notes                                       |
|------|------------------|----------|----------|----------|---------------------------------------------|
| PA8  | AF Push-Pull     | No pull  | High     | —        | TIM1_CH1: 10 kHz PWM, 12-bit, preload OFF for immediate CCR update from ISR |
| PB10 | AF Push-Pull     | No pull  | High     | —        | TIM1_CH3: mirrors CH1 timing for synchronised dual-bank firing |
| PA6  | Input            | Pull-up  | —        | —        | 10 kΩ external pull-up on PCB; low = fault   |

> **¹ Alternate option:** PB10 can also map to TIM2_CH3 (AF1) or USART3_TX (AF7).
> Using TIM1_CH3 (AF6) keeps both weld channels on the same timer for precise
> synchronisation. If independent timing is needed, TIM2_CH3 is available.

### Gate Driver Details

| Driver | IC           | Ref Des | Input Pin | MOSFET Group        | Output Rail |
|--------|-------------|---------|-----------|---------------------|-------------|
| #1     | UCC21520ADW | U18     | INA (pin 1) ← PA8  | Q3–Q10 (8× IRL40SC228) | 10V_GATE via LM7810 |
| #2     | UCC21520ADW | U16     | INA (pin 1) ← PB10 | Q11–Q18 (8× IRL40SC228) | 10V_GATE via LM7810 |

- DIS (pin 5) on both drivers tied to WELD_FAULT with 10 kΩ pull-down to CTRL_GND
- VCCI (pin 3) powered from 5V_CLEAN
- VDDA/VDDB (pins 16/11) powered from 10V_GATE rail

---

## Buck Charger

Synchronous buck converter charges the supercapacitor bank from the battery pack.
Uses UCC21520ADW (U14) as the isolated half-bridge driver for high-side (Q2) and
low-side (Q1) IRL40SC228 MOSFETs through an inductor (L1: CSQX2918L-8R2MC, 8.2 µH).

| Pin  | Pkg Pin | AF    | Peripheral | Net Name        | Direction | Description                            |
|------|:-------:|-------|------------|-----------------|-----------|----------------------------------------|
| PB9  | 47      | AF10² | TIM8_CH3   | CHARGE_GATE_H   | Output    | High-side gate → UCC21520 U14 INA (pin 1) |
| PB1  | 18      | AF4²  | TIM8_CH3N  | CHARGE_GATE_L   | Output    | Low-side gate → UCC21520 U14 INB (pin 2) |
| PB8  | 46      | GPIO  | —          | CHARGE_FAULT    | Input     | Fault input from charge buck converter  |

### Electrical Configuration — Buck Charger

| Pin  | Mode             | Pull     | Speed    | Notes                                           |
|------|------------------|----------|----------|--------------------------------------------------|
| PB9  | AF Push-Pull     | No pull  | High     | TIM8_CH3: complementary PWM with dead-time       |
| PB1  | AF Push-Pull     | No pull  | High     | TIM8_CH3N: complementary output of PB9           |
| PB8  | Input            | Pull-up  | —        | ⚠️ BOOT0 function — see [Special Considerations](#conflicts-and-special-considerations) |

> **² AF verification:** PB9 → TIM8_CH3 is AF10 per STM32G474 datasheet Table 13.
> PB1 → TIM8_CH3N is AF4. Both confirmed for UFQFPN48 package.

### Buck Converter Specifications

| Parameter            | Value                    | Notes                              |
|---------------------|--------------------------|------------------------------------|
| Topology            | Synchronous buck         | High-side + low-side MOSFETs       |
| Switching frequency | Configurable via TIM8    | Target: 50–100 kHz                 |
| Inductor            | CSQX2918L-8R2MC (8.2 µH) | Coilcraft, high current rated     |
| High-side MOSFET    | IRL40SC228 (Q2)         | V_DS = 150 V, R_DS(on) = 3.2 mΩ   |
| Low-side MOSFET     | IRL40SC228 (Q1)         | Same as above                      |
| Gate driver         | UCC21520ADW (U14)       | Isolated half-bridge driver        |
| Gate supply         | LM7810 (U17) → 10V_GATE | 10 V regulated gate drive voltage  |
| Protection          | SMF15CA TVS diodes (D1, D2) | Across each gate–source       |

---

## Balancer (4-Cell)

Passive cell balancer using a TLP291-4 quad optocoupler and IRLML6244TRPBF N-channel
MOSFETs. Each channel bleeds ~1.26 A through 3× paralleled 10 Ω resistors (3.33 Ω
effective). See [BALANCER_SPECIFICATION.md](BALANCER_SPECIFICATION.md) for full circuit details.

| Pin  | Pkg Pin | AF   | Peripheral | Net Name   | Direction | Description                         |
|------|:-------:|------|------------|------------|-----------|-------------------------------------|
| PB3  | 41      | GPIO | —          | BAL_CELL1  | Output    | TLP291-4 Ch1 LED → Cell 1 (bottom)  |
| PB4  | 42      | GPIO | —          | BAL_CELL2  | Output    | TLP291-4 Ch2 LED → Cell 2           |
| PB5  | 43      | GPIO | —          | BAL_CELL3  | Output    | TLP291-4 Ch3 LED → Cell 3           |
| PB6  | 44      | GPIO | —          | BAL_CELL4  | Output    | TLP291-4 Ch4 LED → Cell 4 (top)     |

### Electrical Configuration — Balancer

| Pin  | Mode             | Pull     | Speed | Drive         | Notes                                |
|------|------------------|----------|-------|---------------|--------------------------------------|
| PB3  | Output Push-Pull | No pull  | Low   | 3.3 V / 470 Ω | JTDO in JTAG mode — safe for SWD-only |
| PB4  | Output Push-Pull | No pull  | Low   | 3.3 V / 470 Ω | NJTRST in JTAG mode — safe for SWD-only |
| PB5  | Output Push-Pull | No pull  | Low   | 3.3 V / 470 Ω | No debug conflict                    |
| PB6  | Output Push-Pull | No pull  | Low   | 3.3 V / 470 Ω | No conflict (I2C1 moved to PA15/PB7) |

- **Batch mask:** `0x0078` (bits 3–6) on GPIOB ODR for simultaneous enable/disable
- **Default state:** All LOW (balancers off) at power-on
- **LED forward current:** I_F = (3.3 V − 1.2 V) / 470 Ω ≈ 4.5 mA (within TLP291-4 spec)

---

## Analog Inputs

Five analog channels across ADC1 and ADC2. ADC configuration: 12-bit resolution,
PCLK/4 clock, single-ended, right-aligned data.

| Pin  | Pkg Pin | Channel     | Peripheral | Net Name     | Description                              |
|------|:-------:|-------------|------------|--------------|------------------------------------------|
| PA0  | 8       | ADC1_IN1    | ADC1       | THERM_IN     | NTC thermistor — temperature monitoring   |
| PA1  | 9       | ADC1_IN2    | ADC1       | SHUNT_P_ADC  | AMC1302 OUTP — current sense positive     |
| PA2  | 10      | ADC1_IN3    | ADC1       | SHUNT_N_ADC  | AMC1302 OUTN — current sense negative     |
| PA3  | 11      | ADC1_IN4    | ADC1       | VCAP_OUTP    | AMC1311B OUTP — cap voltage positive leg  |
| PA4  | 12      | ADC2_IN17   | ADC2       | VCAP_OUTN    | AMC1311B OUTN — cap voltage negative leg  |

### Electrical Configuration — Analog Inputs

| Pin  | Mode   | Pull     | Speed | Notes                                              |
|------|--------|----------|-------|----------------------------------------------------|
| PA0  | Analog | No pull  | —     | 10 kΩ NTC divider; slow-moving signal               |
| PA1  | Analog | No pull  | —     | Differential pair with PA2; fast current capture     |
| PA2  | Analog | No pull  | —     | Differential pair with PA1; fast current capture     |
| PA3  | Analog | No pull  | —     | Differential pair with PA4; cap voltage measurement  |
| PA4  | Analog | No pull  | —     | ⚠️ Only available on ADC2 (IN17), not ADC1          |

### ADC Operating Modes

| Mode            | Channels              | Trigger   | Use Case                             |
|-----------------|-----------------------|-----------|--------------------------------------|
| Single-channel  | ADC1_IN1 (PA0)        | Software  | Periodic thermistor reading           |
| Scan mode       | ADC1_IN2,3,4 (PA1-3)  | Software  | Fast current + voltage during weld pulse |
| Single-channel  | ADC2_IN17 (PA4)       | Software  | Cap voltage negative leg              |

> **Note:** ADC1 switches between single-channel and 3-channel scan mode at runtime.
> `adcPrepareFastCurrentChannels()` reconfigures for weld pulse capture;
> normal polling reverts to single-channel reads.

### Isolated Amplifier Signal Chain

```
Battery Current → Shunt Resistor → AMC1302 (±50 mV) → OUTP (PA1) / OUTN (PA2)
                                                         ↕ 1.44 V ± 0.25 V differential

Cap Voltage → Divider → AMC1311B (0–2 V) → OUTP (PA3) / OUTN (PA4)
                                               ↕ 1.44 V ± 0.25 V differential
```

---

## Communication

### USART1 — ESP32 Display Controller Link

| Pin  | Pkg Pin | AF   | Peripheral | Net Name    | Direction | Description                    |
|------|:-------:|------|------------|-------------|-----------|--------------------------------|
| PA9  | 31      | AF7  | USART1_TX  | USART1_TX   | Output    | STM32 → ESP32 data/status      |
| PA10 | 32      | AF7  | USART1_RX  | USART1_RX   | Input     | ESP32 → STM32 commands          |

| Parameter     | Value           | Notes                                    |
|---------------|-----------------|------------------------------------------|
| Baud rate     | 2,000,000 bps   | High speed for real-time data streaming   |
| Format        | 8N1             | 8 data bits, no parity, 1 stop bit       |
| Mode          | Interrupt RX    | HAL_UART_Receive_IT for incoming commands |
| Pull          | No pull         | Direct connection, no external pull-ups   |
| GPIO speed    | Very High       | Required for 2 Mbps signaling             |

### I2C1 — INA226 Voltage/Power Monitors

| Pin  | Pkg Pin | AF   | Peripheral | Net Name  | Direction   | Description                  |
|------|:-------:|------|------------|-----------|-------------|------------------------------|
| PA15 | 38      | AF4  | I2C1_SCL   | I2C_SCL   | Open-drain  | Clock line — 3 INA226 sensors |
| PB7  | 45      | AF4  | I2C1_SDA   | I2C_SDA   | Open-drain  | Data line — 3 INA226 sensors  |

| Parameter       | Value    | Notes                                          |
|-----------------|----------|------------------------------------------------|
| Speed           | 100 kHz  | Standard mode; adequate for 150 ms poll cycle   |
| Pull-ups        | 4.7 kΩ   | External pull-ups to 3V3_CLEAN (R4, R5)        |
| GPIO mode       | AF Open-Drain | Required for I2C bus specification          |
| Address space   | 3 devices | 0x40, 0x41, 0x44                              |

### I2C Device Map

| Address | Ref Des | IC      | Measurement       | Scaling Factor | Calibration Date |
|:-------:|---------|---------|-------------------|:--------------:|:----------------:|
| 0x40    | U8      | INA226  | V_PACK (full pack) | 1.001149       | 2025-05-08       |
| 0x41    | U10     | INA226  | V_LOW (node 1)     | 1.001327       | 2025-05-08       |
| 0x44    | U12     | INA226  | V_MID (node 2)     | 0.986048       | 2025-05-08       |

- INA226 config register: AVG=64, V_BUS CT=1.1 ms, V_SHUNT CT=1.1 ms, continuous mode
- Poll interval: 150 ms (matches 64-sample conversion time: 64 × 1.1 ms × 2 ≈ 141 ms)
- Cell voltages derived: Cell1 = V_LOW, Cell2 = V_MID − V_LOW, Cell3 = V_PACK − V_MID

### ESP32 Display Link (J16 Connector)

| J16 Pin | Signal     | Notes                                  |
|:-------:|------------|----------------------------------------|
| 1       | 5V_CLEAN   | Power supply to ESP32                  |
| 2       | CTRL_GND   | Ground reference                       |
| 3       | USART1_RX  | ESP32 TX → STM32 RX (PA10)            |
| 4       | USART1_TX  | STM32 TX → ESP32 RX (PA9)             |

---

## Digital I/O

| Pin  | Pkg Pin | AF   | Peripheral | Net Name      | Dir    | Description                          |
|------|:-------:|------|------------|---------------|--------|--------------------------------------|
| PB12 | 25      | GPIO | —          | PEDAL_IN      | Input  | Foot pedal trigger (active low)       |
| PB2  | 19      | GPIO | —          | CHARGER_EN    | Output | Charger enable control (active high)  |
| PC6  | 29      | GPIO | —          | STATUS_LED    | Output | Status indicator LED                  |

### Electrical Configuration — Digital I/O

| Pin  | Mode             | Pull       | Speed  | Notes                                    |
|------|------------------|------------|--------|------------------------------------------|
| PB12 | Input            | Internal pull-up | — | Active low: pressed = GND, released = VCC |
| PB2  | Output Push-Pull | No pull    | Low    | HIGH enables charger; safety-interlocked with INA226 health |
| PC6  | Output Push-Pull | No pull    | Low    | General status LED; blink patterns indicate state |

---

## Debug Interface

| Pin  | Pkg Pin | Function | Notes                                            |
|------|:-------:|----------|--------------------------------------------------|
| PA13 | 36      | SWDIO    | Serial Wire Debug data — **reserved, do not reassign** |
| PA14 | 37      | SWCLK    | Serial Wire Debug clock — **reserved, do not reassign** |

- SWD-only configuration (not full JTAG) — frees PB3 (JTDO) and PB4 (NJTRST) for GPIO
- Debug connector: standard 2-pin SWD header on control PCB

---

## System / Power Pins

| Pin Name    | Pkg Pin | Function                      | Notes                         |
|-------------|:-------:|-------------------------------|-------------------------------|
| VBAT        | 1       | Battery backup domain         | Tied to VDD if no backup used |
| PC13        | 2       | RTC / TAMP                    | Limited 3 mA drive — unused    |
| PC14        | 3       | OSC32_IN                      | 32.768 kHz crystal (if used)   |
| PC15        | 4       | OSC32_OUT                     | 32.768 kHz crystal (if used)   |
| PF0         | 5       | OSC_IN (HSE)                  | 8 MHz crystal / oscillator     |
| PF1         | 6       | OSC_OUT (HSE)                 | 8 MHz crystal / oscillator     |
| PG10/NRST   | 7       | System reset                  | Active low with internal pull-up |
| VREF+       | 20      | ADC reference voltage         | Tied to VDDA (3.3 V)          |
| VDDA        | 21      | Analog power supply           | 3.3 V, 100 nF + 1 µF decoupling |
| VDD         | 23      | Digital power supply #1       | 3.3 V, 100 nF decoupling      |
| VDD         | 35      | Digital power supply #2       | 3.3 V, 100 nF decoupling      |
| VDD         | 48      | Digital power supply #3       | 3.3 V, 100 nF decoupling      |
| VSS         | EP      | Ground (exposed pad)          | Soldered to ground plane       |

---

## Complete Pin Table

Sorted by UFQFPN48 package pin number.

| Pkg Pin | Port.Pin  | Function          | AF  | Peripheral   | Net Name        | Dir    | Group        |
|:-------:|-----------|-------------------|-----|-------------|-----------------|--------|--------------|
| 1       | VBAT      | Power             | —   | —           | VBAT            | —      | System       |
| 2       | PC13      | System            | —   | RTC         | —               | —      | System       |
| 3       | PC14      | System            | —   | OSC32       | OSC32_IN        | —      | System       |
| 4       | PC15      | System            | —   | OSC32       | OSC32_OUT       | —      | System       |
| 5       | PF0       | System            | —   | HSE         | OSC_IN          | —      | System       |
| 6       | PF1       | System            | —   | HSE         | OSC_OUT         | —      | System       |
| 7       | PG10/NRST | System            | —   | —           | NRST            | —      | System       |
| 8       | PA0       | Analog            | —   | ADC1_IN1    | THERM_IN        | Input  | Analog       |
| 9       | PA1       | Analog            | —   | ADC1_IN2    | SHUNT_P_ADC     | Input  | Analog       |
| 10      | PA2       | Analog            | —   | ADC1_IN3    | SHUNT_N_ADC     | Input  | Analog       |
| 11      | PA3       | Analog            | —   | ADC1_IN4    | VCAP_OUTP       | Input  | Analog       |
| 12      | PA4       | Analog            | —   | ADC2_IN17   | VCAP_OUTN       | Input  | Analog       |
| 13      | PA5       | **Available**     | —   | —           | —               | —      | *Free*       |
| 14      | PA6       | Digital Input     | —   | GPIO        | WELD_FAULT      | Input  | Weld         |
| 15      | PA7       | **Available**     | —   | —           | —               | —      | *Free*       |
| 16      | PC4       | **Available**     | —   | —           | —               | —      | *Free*       |
| 17      | PB0       | **Available**     | —   | —           | —               | —      | *Free*       |
| 18      | PB1       | AF Push-Pull      | AF4 | TIM8_CH3N   | CHARGE_GATE_L   | Output | Charger      |
| 19      | PB2       | Digital Output    | —   | GPIO        | CHARGER_EN      | Output | Control      |
| 20      | VREF+     | Power             | —   | —           | VREF+           | —      | System       |
| 21      | VDDA      | Power             | —   | —           | VDDA            | —      | System       |
| 22      | PB10      | AF Push-Pull      | AF6 | TIM1_CH3    | WELD_GATE_2     | Output | Weld         |
| 23      | VDD       | Power             | —   | —           | VDD             | —      | System       |
| 24      | PB11      | **Available**     | —   | —           | —               | —      | *Free*       |
| 25      | PB12      | Digital Input     | —   | GPIO        | PEDAL_IN        | Input  | Control      |
| 26      | PB13      | **Available**     | —   | —           | —               | —      | *Free*       |
| 27      | PB14      | **Available**     | —   | —           | —               | —      | *Free*       |
| 28      | PB15      | **Available**     | —   | —           | —               | —      | *Free*       |
| 29      | PC6       | Digital Output    | —   | GPIO        | STATUS_LED      | Output | Control      |
| 30      | PA8       | AF Push-Pull      | AF6 | TIM1_CH1    | WELD_GATE_1     | Output | Weld         |
| 31      | PA9       | AF Push-Pull      | AF7 | USART1_TX   | USART1_TX       | Output | Comm         |
| 32      | PA10      | AF Push-Pull      | AF7 | USART1_RX   | USART1_RX       | Input  | Comm         |
| 33      | PA11      | **Available**     | —   | —           | —               | —      | *Free*       |
| 34      | PA12      | **Available**     | —   | —           | —               | —      | *Free*       |
| 35      | VDD       | Power             | —   | —           | VDD             | —      | System       |
| 36      | PA13      | Debug             | —   | SWD         | SWDIO           | Bidir  | Debug        |
| 37      | PA14      | Debug             | —   | SWD         | SWCLK           | Input  | Debug        |
| 38      | PA15      | AF Open-Drain     | AF4 | I2C1_SCL    | I2C_SCL         | Bidir  | Comm         |
| 39      | PC10      | **Available**     | —   | —           | —               | —      | *Free*       |
| 40      | PC11      | **Available**     | —   | —           | —               | —      | *Free*       |
| 41      | PB3       | Digital Output    | —   | GPIO        | BAL_CELL1       | Output | Balancer     |
| 42      | PB4       | Digital Output    | —   | GPIO        | BAL_CELL2       | Output | Balancer     |
| 43      | PB5       | Digital Output    | —   | GPIO        | BAL_CELL3       | Output | Balancer     |
| 44      | PB6       | Digital Output    | —   | GPIO        | BAL_CELL4       | Output | Balancer     |
| 45      | PB7       | AF Open-Drain     | AF4 | I2C1_SDA    | I2C_SDA         | Bidir  | Comm         |
| 46      | PB8       | Digital Input     | —   | GPIO        | CHARGE_FAULT    | Input  | Charger      |
| 47      | PB9       | AF Push-Pull      | AF10| TIM8_CH3    | CHARGE_GATE_H   | Output | Charger      |
| 48      | VDD       | Power             | —   | —           | VDD             | —      | System       |
| EP      | VSS       | Power             | —   | —           | GND             | —      | System       |

---

## Available Pins for Future Expansion

12 GPIO pins remain unassigned and available:

| Pin   | Pkg Pin | Noteworthy Alternate Functions                        | Potential Use Cases                  |
|-------|:-------:|-------------------------------------------------------|--------------------------------------|
| PA5   | 13      | DAC1_OUT2, SPI1_SCK, TIM2_CH1, ADC2_IN13             | DAC output, SPI bus, extra ADC       |
| PA7   | 15      | SPI1_MOSI, TIM1_CH1N, ADC2_IN4                       | SPI bus, complementary PWM           |
| PA11  | 33      | USB_DM, FDCAN1_RX, TIM1_CH4, TIM4_CH1                | USB, CAN bus, extra timer            |
| PA12  | 34      | USB_DP, FDCAN1_TX, TIM1_ETR, TIM4_CH2                | USB, CAN bus                         |
| PB0   | 17      | TIM1_CH2N, TIM3_CH3, ADC1_IN15                       | Extra PWM, extra ADC                 |
| PB11  | 24      | USART3_RX, I2C2_SDA, TIM2_CH4                        | Second UART, second I2C bus          |
| PB13  | 26      | SPI2_SCK, USART3_CTS, TIM1_CH1N                      | SPI bus, UART flow control           |
| PB14  | 27      | SPI2_MISO, USART3_RTS, TIM1_CH2N                     | SPI bus, UART flow control           |
| PB15  | 28      | SPI2_MOSI, TIM1_CH3N, TIM15_CH2                      | SPI bus, complementary PWM           |
| PC4   | 16      | USART1_TX³, ADC2_IN5                                  | Backup USART, extra ADC              |
| PC10  | 39      | USART3_TX, UART4_TX, SPI3_SCK                        | Second UART, SPI3 bus                |
| PC11  | 40      | USART3_RX, UART4_RX, SPI3_MISO                       | Second UART, SPI3 bus                |

### Recommended Expansion Priorities

1. **USB (PA11 + PA12):** Native USB-FS for firmware updates or PC data logging
2. **CAN Bus (PA11 + PA12):** FDCAN1 for automotive/industrial integration
3. **SPI (PA5 + PA7 + PB13–PB15):** External flash, additional sensors, or display
4. **Second UART (PC10 + PC11):** USART3 for additional communication channel
5. **Extra ADC (PB0, PA5, PC4):** Additional analog monitoring channels

---

## Configuration Notes

### Clock Configuration

| Peripheral | Clock Source | Frequency | Notes                              |
|------------|-------------|:---------:|------------------------------------|
| System     | HSE → PLL   | 170 MHz   | Maximum performance                |
| APB1       | HCLK/1      | 170 MHz   | TIM2–7, USART2/3, I2C1/2          |
| APB2       | HCLK/1      | 170 MHz   | TIM1/8/15–17, USART1, SPI1, ADC   |
| ADC12      | PCLK/4      | 42.5 MHz  | Shared clock for ADC1 and ADC2     |

### GPIO Speed Selection Rationale

| Speed Setting | Used For                        | Reason                              |
|---------------|--------------------------------|--------------------------------------|
| Very High     | USART1 (PA9/PA10)              | 2 Mbps requires fast edge rates      |
| High          | TIM1/TIM8 PWM outputs          | Clean gate drive switching edges      |
| Low           | Balancer GPIOs, LED, charger EN | Reduces EMI; ms-scale response is fine |

### Pull-Up / Pull-Down Summary

| Pin  | Pull Type         | Value    | Location   | Purpose                          |
|------|-------------------|----------|------------|----------------------------------|
| PA15 | External pull-up  | 4.7 kΩ  | R5 to 3V3  | I2C specification requirement     |
| PB7  | External pull-up  | 4.7 kΩ  | R4 to 3V3  | I2C specification requirement     |
| PA6  | External pull-up  | 10 kΩ   | PCB        | Fault line default-high           |
| PB8  | Internal pull-up  | ~40 kΩ  | STM32      | CHARGE_FAULT default-high         |
| PB10 | External pull-down| 10 kΩ   | R3 to GND  | Ensure gate OFF at power-up       |
| PB12 | Internal pull-up  | ~40 kΩ  | STM32      | Pedal default-high (not pressed)  |

### Power Domain Mapping

```
19.5V Battery Pack
    │
    ├── LM7810 (U17) → 10V_GATE ─── UCC21520 VDDA/VDDB gate drive rails
    │
    ├── 5V_CLEAN ──┬── UCC21520 VCCI (all 3 drivers)
    │              ├── ESP32 (via J16)
    │              └── INA226 sensors (Vbus power)
    │
    └── 3V3_CLEAN ─┬── STM32G474CE VDD / VDDA
                   ├── I2C pull-ups
                   └── Optocoupler LED anodes (via 470 Ω)

Ground domains:
    CTRL_GND ───── STM32, ESP32, INA226 (low-noise control ground)
    GND_H ──────── High-current MOSFET sources, gate drivers (isolated by dashed line in schematic)
```

---

## Conflicts and Special Considerations

### ⚠️ PB8 — BOOT0 Function

PB8 (pin 46) serves as the BOOT0 pin on STM32G4 series. When HIGH during reset, the
MCU boots from system memory (DFU bootloader) instead of Flash.

**Mitigation:**
- CHARGE_FAULT is an input with pull-up — defaults HIGH
- The UCC21520 fault output is open-drain, pulling LOW on fault
- A 10 kΩ pull-down resistor must be added on CHARGE_FAULT to ensure PB8 is LOW
  during power-on/reset, preventing accidental DFU boot
- Alternatively, configure `nBOOT0` option bit to force Flash boot regardless of PB8 state

### ⚠️ PB3/PB4 — JTAG Debug Pins

PB3 (JTDO) and PB4 (NJTRST) are JTAG pins by default after reset.

**Mitigation:**
- Firmware must call `__HAL_AFIO_REMAP_SWJ_NOJTAG()` or configure JTAG pins as GPIO
  during `MX_GPIO_Init()` before using PB3/PB4 as balancer outputs
- SWD debug on PA13/PA14 remains fully functional

### ⚠️ PA4 — ADC2-Only Channel

PA4 maps to ADC2_IN17 and is **not available on ADC1**. The AMC1311B OUTN signal on PA4
requires separate ADC2 initialization and cannot be included in ADC1 scan sequences.

**Impact:**
- Fast current/voltage capture requires coordinated reads from both ADC1 (PA1–PA3)
  and ADC2 (PA4)
- Currently handled by separate `MX_ADC2_Init()` and `adcPrepareVcapNChannel()`

### ⚠️ I2C1 Pin Selection History

I2C1 was moved from PB8/PB9 (default) to PA15/PB7 to avoid:
- PB8 BOOT0 conflict (would cause DFU boot if SDA was high during reset)
- PB9 conflict with TIM8_CH3 (now used for CHARGE_GATE_H)
- PA15 (AF4) and PB7 (AF4) confirmed working with 4.7 kΩ external pull-ups

### Timer Resource Allocation

| Timer | Channel(s) Used        | Purpose                    | Available Channels         |
|-------|------------------------|----------------------------|-----------------------------|
| TIM1  | CH1 (PA8), CH3 (PB10) | Weld gate drive (dual)     | CH2, CH4, CH1N–CH3N free   |
| TIM8  | CH3 (PB9), CH3N (PB1) | Buck charger complementary PWM | CH1, CH2, CH4 free     |
| TIM2  | —                      | Not used                   | All channels available      |
| TIM3  | —                      | Not used                   | All channels available      |
| TIM4  | —                      | Not used                   | All channels available      |

---

## Cross-Reference

| Document                                              | Relevance                              |
|-------------------------------------------------------|----------------------------------------|
| [BALANCER_SPECIFICATION.md](BALANCER_SPECIFICATION.md) | Full 4-cell balancer circuit & firmware |
| [GPIO_BALANCER_ANALYSIS.md](../STM32G474CE/GPIO_BALANCER_ANALYSIS.md) | GPIO selection rationale for PB3–PB6 |
| STM32G474xE Datasheet (DS12288)                       | Pin alternate functions, electrical specs |
| STM32G4 Reference Manual (RM0440)                     | Peripheral register details, timer config |
| IRL40SC228 Datasheet                                  | MOSFET specs for weld & charger FETs    |
| UCC21520ADW Datasheet                                 | Gate driver pinout & application circuit |
| INA226 Datasheet                                      | I2C address configuration, registers     |
| AMC1302 Datasheet                                     | Current sense amplifier signal levels    |
| AMC1311B Datasheet                                    | Voltage sense amplifier signal levels    |
| Si827x Datasheet                                      | ISOdriver for high dV/dt immunity        |

---

## Revision History

| Rev | Date       | Author | Changes                                  |
|-----|------------|--------|------------------------------------------|
| 1.0 | 2026-05-10 | JQ-DHJ | Initial release — final hardware config   |

---

*This document is the authoritative reference for all STM32G474CE pin assignments
in the spot welder project. Any firmware or hardware changes must update this
document first.*
