# STM32G474CEU6 GPIO Pin Analysis for 4-Cell Balancer

## Package: UFQFPN48 (48 pins)

---

## Current Pin Usage Map

### GPIOA — Used Pins

| Pin   | Pkg Pin | Function              | Peripheral    | Notes                              |
|-------|---------|----------------------|---------------|------------------------------------|
| PA0   | 8       | ADC1_IN1 (analog)    | ADC1          | Thermistor NTC reading             |
| PA1   | 9       | ADC1_IN2 (analog)    | ADC1          | Shunt current P (AMC1302 OUTP)     |
| PA2   | 10      | ADC1_IN3 (analog)    | ADC1          | Shunt current N (AMC1302 OUTN)     |
| PA3   | 11      | ADC1_IN4 (analog)    | ADC1          | AMC1311B OUTP (cap voltage +)      |
| PA4   | 12      | ADC2_IN17 (analog)   | ADC2          | AMC1311B OUTN (cap voltage -)      |
| PA8   | 30      | TIM1_CH1 (AF6)       | TIM1          | PWM weld FET drive                 |
| PA9   | 31      | USART1_TX (AF7)      | USART1        | UART TX → ESP32                    |
| PA10  | 32      | USART1_RX (AF7)      | USART1        | UART RX ← ESP32                    |
| PA13  | 36      | SWDIO                | Debug         | SWD debug (reserved)               |
| PA14  | 37      | SWCLK                | Debug         | SWD debug (reserved)               |
| PA15  | 38      | I2C1_SCL (AF4)       | I2C1          | INA226 sensors SCL                 |

### GPIOB — Used Pins

| Pin   | Pkg Pin | Function              | Peripheral    | Notes                              |
|-------|---------|----------------------|---------------|------------------------------------|
| PB2   | 19      | Digital output (PP)  | GPIO          | Charger enable (active high)       |
| PB7   | 45      | I2C1_SDA (AF4)       | I2C1          | INA226 sensors SDA                 |
| PB12  | 25      | Digital input (PU)   | GPIO          | Foot pedal trigger                 |

### GPIOC — Used Pins

| Pin   | Pkg Pin | Function              | Peripheral    | Notes                              |
|-------|---------|----------------------|---------------|------------------------------------|
| PC6   | 29      | Digital output (PP)  | GPIO          | Status LED                         |

### Other / System Pins

| Pin          | Pkg Pin | Function          |
|-------------|---------|-------------------|
| VBAT        | 1       | Battery backup    |
| PC13        | 2       | RTC domain (limited 3mA drive) |
| PC14        | 3       | OSC32_IN          |
| PC15        | 4       | OSC32_OUT         |
| PF0         | 5       | HSE OSC_IN        |
| PF1         | 6       | HSE OSC_OUT       |
| PG10/NRST   | 7       | Reset             |
| VREF+       | 20      | ADC reference     |
| VDDA        | 21      | Analog VDD        |
| VDD         | 23,35,48| Digital VDD       |
| VSS         | exposed | Ground            |

---

## Available GPIO Pins (Not Used by Firmware)

### Port A — Available

| Pin  | Pkg Pin | Notes                                       |
|------|---------|---------------------------------------------|
| PA5  | 13      | Free. Has ADC2/DAC/SPI/TIM alternate funcs   |
| PA6  | 14      | Free. Has ADC2/TIM/SPI alternate funcs       |
| PA7  | 15      | Free. Has ADC2/TIM/SPI alternate funcs       |
| PA11 | 33      | Free. Has USB_DM/CAN/TIM alternate funcs     |
| PA12 | 34      | Free. Has USB_DP/CAN/TIM alternate funcs     |

### Port B — Available

| Pin   | Pkg Pin | Notes                                       |
|-------|---------|---------------------------------------------|
| PB0   | 17      | Free. Has ADC/TIM alternate funcs            |
| PB1   | 18      | Free. Has ADC/TIM alternate funcs            |
| **PB3**  | **41**  | **Free. JTDO/TRACESWO (unused w/ SWD-only)** |
| **PB4**  | **42**  | **Free. NJTRST (unused w/ SWD-only)**        |
| **PB5**  | **43**  | **Free. No debug conflict**                  |
| **PB6**  | **44**  | **Free. No conflict (I2C1 on PA15/PB7)**     |
| PB8   | 46      | BOOT0 — **avoid** (boot function)            |
| PB9   | 47      | Free. Has I2C/CAN/TIM alternate funcs        |
| PB10  | 22      | Free. Has USART3/I2C/TIM alternate funcs     |
| PB11  | 24      | Free. Has USART3/I2C/TIM alternate funcs     |
| PB13  | 26      | Free. Has SPI2/USART3/TIM alternate funcs    |
| PB14  | 27      | Free. Has SPI2/USART3/TIM alternate funcs    |
| PB15  | 28      | Free. Has SPI2/TIM alternate funcs           |

### Port C — Available

| Pin   | Pkg Pin | Notes                                       |
|-------|---------|---------------------------------------------|
| PC4   | 16      | Free. Has ADC/USART alternate funcs          |
| PC10  | 39      | Free. Has USART3/UART4/SPI3 alternate funcs  |
| PC11  | 40      | Free. Has USART3/UART4/SPI3 alternate funcs  |

---

## ✅ Recommended 4 Pins: PB3, PB4, PB5, PB6

### Why These 4 Pins?

| Criteria                        | PB3/PB4/PB5/PB6                                |
|---------------------------------|-------------------------------------------------|
| **Same port**                   | ✅ All GPIOB — single GPIO port init/write       |
| **Consecutive**                 | ✅ Bits 3–6, can use mask 0x0078 for batch ops   |
| **Physical adjacency**          | ✅ Package pins 41-42-43-44 (adjacent on PCB)    |
| **No peripheral conflict**      | ✅ None used by current firmware                  |
| **Debug safe**                  | ✅ PB3/PB4 are JTAG-only (SWD uses PA13/PA14)    |
| **BOOT0 conflict**              | ✅ None (PB8 avoided)                            |
| **PCB routing**                 | ✅ Adjacent pins simplify trace routing           |
| **Future expansion safe**       | ✅ Leaves PB0/PB1/PB9-15 free for other uses     |

### Pin Assignment for Balancer

| Balancer Channel | GPIO Pin | Pkg Pin | Controls          |
|-----------------|----------|---------|-------------------|
| Cell 1 (bottom) | PB3      | 41      | TLP291-4 LED CH1  |
| Cell 2 (middle) | PB4      | 42      | TLP291-4 LED CH2  |
| Cell 3 (top)    | PB5      | 43      | TLP291-4 LED CH3  |
| Cell 4 (spare)  | PB6      | 44      | TLP291-4 LED CH4  |

> **Note**: With 3S2P configuration (3 cells), you need 3 channels. PB6 gives you a spare
> 4th channel for future 4S expansion or diagnostic use.

---

## Example Initialization Code

```c
/* ============ Balancer GPIO Pins ============ */
#define BAL_PORT        GPIOB
#define BAL_CELL1_PIN   GPIO_PIN_3   /* PB3 - bottom cell */
#define BAL_CELL2_PIN   GPIO_PIN_4   /* PB4 - middle cell */
#define BAL_CELL3_PIN   GPIO_PIN_5   /* PB5 - top cell    */
#define BAL_CELL4_PIN   GPIO_PIN_6   /* PB6 - spare       */
#define BAL_ALL_PINS    (BAL_CELL1_PIN | BAL_CELL2_PIN | BAL_CELL3_PIN | BAL_CELL4_PIN)

/* Add to MX_GPIO_Init(): */
static void balancer_gpio_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* All 4 balancer pins: push-pull output, no pull, low speed */
    GPIO_InitStruct.Pin   = BAL_ALL_PINS;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BAL_PORT, &GPIO_InitStruct);

    /* Start with all balancers OFF */
    HAL_GPIO_WritePin(BAL_PORT, BAL_ALL_PINS, GPIO_PIN_RESET);
}

/* Enable/disable individual cell balancing */
static inline void balancer_set(uint16_t pin, bool enable) {
    HAL_GPIO_WritePin(BAL_PORT, pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Disable all balancers (safety) */
static inline void balancer_all_off(void) {
    HAL_GPIO_WritePin(BAL_PORT, BAL_ALL_PINS, GPIO_PIN_RESET);
}
```

---

## Alternate Options (if PB3-PB6 not suitable)

| Option | Pins             | Pros                              | Cons                         |
|--------|------------------|-----------------------------------|------------------------------|
| **A**  | PB3-PB6          | ✅ Best: consecutive, same port   | PB3/4 are JTAG (ok for SWD) |
| **B**  | PA5, PA6, PA7 + PB0 | All near ADC pins on PCB      | Split across two ports        |
| **C**  | PB13-PB15 + PB9  | Far from sensitive ADC pins       | Non-consecutive, opposite side|
| **D**  | PC4, PC10, PC11 + PB9 | Different port                | Split across ports, fewer PC pins |

---

*Generated: 2026-05-10 | Project: Spot Welder STM32G474CE Firmware*
