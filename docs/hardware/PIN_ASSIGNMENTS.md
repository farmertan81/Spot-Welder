# STM32G474CE Pin Assignments (Final)

> **Project:** Spot Welder (Final Schematic Freeze)
>
> **Controller board:** WeAct Studio STM32G474CE breakout (removable header format)
>
> **MCU:** STM32G474CEU6 (UFQFPN48)
>
> **Revision:** Final configuration for PCB layout handoff
>
> **Date:** 2026-07-15 (Updated)

---

## 1) Hardware Context (Final)

- 4S pack architecture with weld stage + synchronous buck charge stage + 4-cell balancer
- 16× IRL40SC228 weld MOSFETs (two 8-FET banks)
- 2× UCC21520ADW isolated dual gate drivers for weld banks
- 2× UCC21520ADW for charger half-bridge
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
| --- | :---: | --- |
| Weld control | 3 | PA8, PB10, PA6 |
| Charger control | 4 | PB9, PB1, PB8 |
| Balancer control | 4 | PB3, PB4, PB5, PB6 |
| Communication | 4 | PA9, PA10, PA15, PB7 |
| ADC / analog sensing | 6 | PA0, PB0, PA1, PA2, PA3, PA4 |
| Operator / status I/O | 2 | PB12, PC6 |
| Debug / infrastructure | 4 | PA13, PA14, NRST, PB11 |
| **Total** | **26** | **65% of 40** |

---

## 3) Complete Final Pin Table (26 Allocated)

**NOTE:** This document is the master logical pin-allocation record. The schematic must implement these assignments exactly.

| # | MCU Pin | Signal Net | Primary Function | Direction | Group |
| :---: | :---: | --- | --- | :---: | --- |
| 1 | PA8 | WELD_GATE_1 | TIM1 PWM to weld driver bank 1 | Output | Weld |
| 2 | PB10 | WELD_GATE_2 | TIM1 PWM to weld driver bank 2 | Output | Weld |
| 3 | PA6 | WELD_FAULT | Weld gate fault input / interlock | Input | Weld |
| 4 | PB9 | CHARGE_GATE_H | High-side gate command (charger) | Output | Charger |
| 5 | PB1 | CHARGE_GATE_L | Low-side gate command (charger) | Output | Charger |
| 6 | PB8 | CHARGE_FAULT | Charger fault input | Input | Charger |
| 7 | PB3 | CELL_1 | Balancer channel 1 drive | Output | Balancer |
| 8 | PB4 | CELL_2 | Balancer channel 2 drive | Output | Balancer |
| 9 | PB5 | CELL_3 | Balancer channel 3 drive | Output | Balancer |
| 10 | PB6 | CELL_4 | Balancer channel 4 drive | Output | Balancer |
| 11 | PA15 | I2C_SCL | I2C1 SCL for INA226 cluster | Bidir | Communication |
| 12 | PB7 | I2C_SDA | I2C1 SDA for INA226 cluster | Bidir | Communication |
| 13 | PA9 | USART1_TX | STM32 -> ESP32 display | Output | Communication |
| 14 | PA10 | USART1_RX | ESP32 -> STM32 command | Input | Communication |
| 15 | PA0 | FET1_THERM_IN | FET bank 1 NTC thermistor | Analog | ADC |
| 16 | PB0 | FET2_THERM_IN | FET bank 2 NTC thermistor | Analog | ADC |
| 17 | PA1 | SHUNT_P_ADC | Current sense differential + | Analog | ADC |
| 18 | PA2 | SHUNT_N_ADC | Current sense differential - | Analog | ADC |
| 19 | PA3 | VCAP_OUTP | Capacitor sense differential + | Analog | ADC |
| 20 | PA4 | VCAP_OUTN | Capacitor sense differential - | Analog | ADC |
| 21 | PB12 | PEDAL_IN | Foot pedal trigger | Input | Operator I/O |
| 22 | PC6 | STATUS_LED | Local status LED | Output | Operator I/O |
| 23 | PA13 | SWDIO | Debug/programming | Bidir | Infrastructure |
| 24 | PA14 | SWCLK | Debug/programming | Input | Infrastructure |
| 25 | NRST | NRST | Hardware reset | Input | Infrastructure |
| 26 | PB11 | AUX_IO_RESERVED | Reserved routed I/O | I/O | Infrastructure |

> **Notes:**
> - PB3/PB4 require SWD-only debug configuration (JTAG disabled) when used as GPIO balancer outputs.
> - PA0 and PB0 are assigned to the dual FET-bank thermistors per schematic.

---

## 4) Available Pins for Expansion (14)

The following pins remain available for future features (CAN, USB, secondary UART, SPI, additional ADC, etc.):

- **PB2:** Available (was CHG_PWM on early board)
- **PA5, PA7, PA11, PA12**
- **PB13, PB14, PB15**
- **PC4, PC10, PC11, PC13, PC15**

---

**DOCUMENT STATUS:** This is the authority. Inconsistent data in legacy analysis files (e.g., STM32G474CE/GPIO_BALANCER_ANALYSIS.md) should be ignored or archived.