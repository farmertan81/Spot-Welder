# STM32G474CE Pin Assignments (Final)

> **Project:** Spot Welder (Final Schematic Freeze)
> 
> **Controller board:** WeAct Studio STM32G474CE breakout (removable header format)
> 
> **MCU:** STM32G474CEU6 (UFQFPN48)
> 
> **Revision:** Final configuration for PCB layout handoff
> 
> **Date:** 2026-05-10

---

## 1) Hardware Context (Final)

- 4S pack architecture with weld stage + synchronous buck charge stage + 4-cell balancer
- 16× IRL40SC228 weld MOSFETs (two 8-FET banks)
- 2× UCC21520ADW isolated dual gate drivers for weld banks
- 1× UCC21520ADW for charger half-bridge
- 5× INA226 monitors on shared I2C bus
- 4-cell passive balancer (TLP291-4 + IRLML6244)
- Differential isolated analog sensing (AMC1302 + AMC1311)

---

## 2) Pin Usage Summary

### MCU utilization

- **Header-accessible pins considered:** 40
- **Allocated pins (active + infrastructure):** 26
- **Utilization:** **65%**
- **Available for expansion:** 14 pins

### Allocation by function

| Functional Group | Pins | Notes |
|---|---:|---|
| Weld control | 3 | PA8, PB10, PA6 |
| Charger control | 4 | PB9, PB1, PB8, PB2 |
| Balancer control | 4 | PB3, PB4, PB5, PB6 |
| Communication | 4 | PA9, PA10, PA15, PB7 |
| ADC / analog sensing | 5 | PA0, PA1, PA2, PA3, PA4 |
| Operator / status I/O | 2 | PB12, PC6 |
| Debug / infrastructure | 4 | PA13, PA14, NRST, PB11 |
| **Total** | **26** | **65% of 40** |

---

## 3) Complete Final Pin Table (26 Allocated)

| # | MCU Pin | Signal Net | Primary Function | Direction | Group |
|---:|---|---|---|---|---|
| 1 | PA8 | WELD_GATE_1 | TIM1 PWM to weld driver bank 1 (U18) | Output | Weld |
| 2 | PB10 | WELD_GATE_2 | TIM1 PWM to weld driver bank 2 (U16) | Output | Weld |
| 3 | PA6 | WELD_FAULT | Weld gate fault input / interlock | Input | Weld |
| 4 | PB9 | CHARGE_GATE_H | High-side gate command (charger) | Output | Charger |
| 5 | PB1 | CHARGE_GATE_L | Low-side gate command (charger) | Output | Charger |
| 6 | PB8 | CHARGE_FAULT | Charger fault input | Input | Charger |
| 7 | PB2 | CHG_PWM | Charger control/enable path | Output | Charger |
| 8 | PB3 | CELL_1 | Balancer channel 1 drive (optocoupler LED) | Output | Balancer |
| 9 | PB4 | CELL_2 | Balancer channel 2 drive (optocoupler LED) | Output | Balancer |
| 10 | PB5 | CELL_3 | Balancer channel 3 drive (optocoupler LED) | Output | Balancer |
| 11 | PB6 | CELL_4 | Balancer channel 4 drive (optocoupler LED) | Output | Balancer |
| 12 | PA15 | I2C_SCL | I2C1 SCL for INA226 cluster | Bidirectional (OD) | Communication |
| 13 | PB7 | I2C_SDA | I2C1 SDA for INA226 cluster | Bidirectional (OD) | Communication |
| 14 | PA9 | USART1_TX | STM32 → ESP32 display/control link | Output | Communication |
| 15 | PA10 | USART1_RX | ESP32 → STM32 command link | Input | Communication |
| 16 | PA0 | THERM_IN | NTC thermistor ADC input | Input (Analog) | ADC |
| 17 | PA1 | SHUNT_P_ADC | Current sense differential + | Input (Analog) | ADC |
| 18 | PA2 | SHUNT_N_ADC | Current sense differential − | Input (Analog) | ADC |
| 19 | PA3 | VCAP_OUTP | Capacitor sense differential + | Input (Analog) | ADC |
| 20 | PA4 | VCAP_OUTN | Capacitor sense differential − | Input (Analog) | ADC |
| 21 | PB12 | PEDAL_IN | Foot pedal trigger input | Input | Operator I/O |
| 22 | PC6 | STATUS_LED | Local status LED/indicator | Output | Operator I/O |
| 23 | PA13 | SWDIO | Debug/programming | Bidirectional | Infrastructure |
| 24 | PA14 | SWCLK | Debug/programming | Input | Infrastructure |
| 25 | NRST | NRST | Hardware reset | Input | Infrastructure |
| 26 | PB11 | AUX_IO_RESERVED | Reserved routed I/O for post-layout options | I/O reserved | Infrastructure |

> Notes:
> - PB3/PB4 require SWD-only debug configuration (JTAG disabled) when used as GPIO balancer outputs.
> - PA4 is used as analog negative leg input in the differential measurement chain.

---

## 4) Functional Group Details

### 4.1 Weld control

- **PA8 → U18 INA** (bank 1)
- **PB10 → U16 INA** (bank 2)
- **PA6 ← WELD_FAULT** feedback/interlock

This drives the 16× IRL40SC228 arrangement as two synchronized banks.

### 4.2 Balancer control (4-cell)

- PB3/PB4/PB5/PB6 map to CELL_1..CELL_4 balancer channels
- Each channel drives TLP291-4 input LED path, switching IRLML6244 bleed MOSFETs

### 4.3 Charger control

- PB9 = high-side gate command
- PB1 = low-side gate command
- PB8 = charger fault input
- PB2 = charger PWM/enable control path

### 4.4 Communication

- **USART1:** PA9/PA10 to ESP32 UI board
- **I2C1:** PA15/PB7 to all INA226 monitors (shared bus)

### 4.5 ADC / analog inputs

- PA0: thermistor
- PA1/PA2: current shunt differential path
- PA3/PA4: capacitor voltage differential path

---

## 5) INA226 Sensor Network (All 5 Devices)

All INA226 devices share I2C1 (PA15/PB7, 4.7k pull-ups to 3V3_CLEAN).

| Device | I2C Address | Measured Nodes | Purpose |
|---|---|---|---|
| U8 | **0x40** | CHG_SHUNT_P / CHG_SHUNT_N, PACK_POS reference | Charge/weld shunt monitor |
| U9 | **0x41** | CELL1_TAP to GND_H | Cell 1 monitor |
| U13 | **0x42** | CELL2_TAP to CELL1_TAP | Cell 2 monitor |
| U12 | **0x43** | CELL3_TAP to CELL2_TAP | Cell 3 monitor |
| U10 | **0x44** | CELL4_TAP to CELL3_TAP | Cell 4 monitor |

This gives complete per-cell and pack-level observability for balancing and safety logic.

---

## 6) WeAct Header Connector Mapping

### 6.1 J17 (left-side breakout group)

| Net | MCU Pin | Function |
|---|---|---|
| PEDAL_IN | PB12 | Foot pedal trigger |
| USART1_TX | PA9 | Serial TX to ESP32 |
| USART1_RX | PA10 | Serial RX from ESP32 |
| I2C_SCL | PA15 | I2C1 clock |
| I2C_SDA | PB7 | I2C1 data |
| CELL_1 | PB3 | Balancer channel 1 |
| CELL_2 | PB4 | Balancer channel 2 |
| CELL_3 | PB5 | Balancer channel 3 |
| CELL_4 | PB6 | Balancer channel 4 |
| CHARGE_FAULT | PB8 | Charger fault input |
| WELD_GATE_1 | PA8 | Weld bank 1 gate command |
| CHARGE_GATE_H | PB9 | Charger high-side gate command |

### 6.2 J15 (right-side breakout group)

| Net | MCU Pin | Function |
|---|---|---|
| WELD_GATE_2       | PB10 | Weld bank 2 gate command |
| CHG_PWM           | PB2 | Charger control/enable path |
| CHARGE_GATE_L     | PB1 | Charger low-side gate command |
| WELD_FAULT        | PA6 | Weld stage fault input |
| THERM_IN          | PA0 | Thermistor ADC |
| SHUNT_P_ADC       | PA1 | Current-sense ADC + |
| SHUNT_N_ADC       | PA2 | Current-sense ADC − |
| VCAP_OUTP         | PA3 | Capacitor ADC + |
| VCAP_OUTN         | PA4 | Capacitor ADC − |
| AUX_IO_RESERVED   | PB11 | Reserved post-layout I/O |

---

## 7) Available Pins for Expansion (14)

The following pins remain available for future features (CAN, USB, secondary UART, SPI, additional ADC, etc.):

- **PA5, PA7, PA11, PA12**
- **PB0, PB13, PB14, PB15**
- **PC4, PC10, PC11, PC13, PC14, PC15**

Recommended usage priority:
1. PA11/PA12 for USB or FDCAN
2. PB13/PB14/PB15 + PA5 for SPI expansion
3. PC10/PC11 for secondary UART link
4. PB0/PA5 as additional analog or PWM resources

---

## 8) Revision Log

| Rev | Date | Change |
|---|---|---|
| 1.1 | 2026-05-10 | Finalized complete 26-pin map, added all 5 INA226 mappings, updated J17/J15 connector tables, confirmed 65% utilization |

---

This document is the authoritative pin assignment for the finalized schematic and PCB layout phase.
