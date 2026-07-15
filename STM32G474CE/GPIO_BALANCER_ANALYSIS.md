# STM32G474CEU6 Balancer GPIO Design

This document details the hardware design and GPIO allocation for the 4-cell passive balancer.

## 1) GPIO Allocation

The balancer uses a contiguous group of pins on Port B to simplify the driver logic and PCB routing.

| Balancer Channel | MCU Pin | Function |
| :--- | :---: | :--- |
| Cell 1 (Bottom) | **PB3** | TLP291-4 LED CH1 |
| Cell 2 | **PB4** | TLP291-4 LED CH2 |
| Cell 3 | **PB5** | TLP291-4 LED CH3 |
| Cell 4 (Top/Spare) | **PB6** | TLP291-4 LED CH4 |

## 2) Design Constraints

### 2.1 Debug Port Conflict
**CRITICAL:** PB3 (JTDO) and PB4 (NJTRST) are multiplexed with the JTAG debug interface. 
- To use these as balancer GPIOs, the MCU must be configured for **SWD-only** debugging. 
- The JTAG interface must be explicitly disabled in firmware to release these pins for GPIO use.

### 2.2 Electrical Interface
Each GPIO drives an input LED of the TLP291-4 quad optoisolator through a current-limiting resistor.
- The optoisolators provide reinforced galvanic isolation between the MCU control logic and the high-voltage battery stack.
- The optoisolator outputs switch **IRLML6244** N-channel MOSFETs which engage the bleed resistors for passive balancing.

---

## 3) Master Pin Map Reference
This document covers the balancer section specifically. For the complete MCU pin-out and signal definitions for the entire board, refer to the master record:
**[docs/hardware/PIN_ASSIGNMENTS.md](../hardware/PIN_ASSIGNMENTS.md)**