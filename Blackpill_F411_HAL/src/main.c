#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"

/* ============ Your pins (fixed) ============ */
#define WELD_TIM TIM1
#define WELD_TIM_CH TIM_CHANNEL_1  // PA8 = TIM1_CH1 (AF1)
#define PEDAL_PORT GPIOB
#define PEDAL_PIN GPIO_PIN_12  // PB12 input pull-up, active low

/* UART1 pins are configured by GPIO AF: PA9/PA10 (AF7) */

/* ============ Limits / Timing ============ */
static const uint32_t WELD_COOLDOWN_MS = 500;
static const uint16_t MAX_WELD_MS = 200;
static const uint32_t PEDAL_DEBOUNCE_MS = 40;
static const uint32_t BOOT_INHIBIT_MS = 5000;
static const uint32_t ARM_TIMEOUT_MS = 0;  // 0 = disabled

/* ============ PWM Settings ============ */
static const uint16_t PWM_MAX = 1023;

/* ============ HAL handles ============ */
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim1;

/* ============ Weld Parameters (defaults) ============ */
static volatile uint8_t weld_mode = 1;  // 1=single, 2=double, 3=triple
static volatile uint16_t weld_d1_ms = 10;
static volatile uint16_t weld_gap1_ms = 0;
static volatile uint16_t weld_d2_ms = 0;
static volatile uint16_t weld_gap2_ms = 0;
static volatile uint16_t weld_d3_ms = 0;

static volatile uint8_t weld_power_pct = 100;  // 50-100%
static volatile bool preheat_enabled = false;
static volatile uint16_t preheat_ms = 20;
static volatile uint8_t preheat_pct = 30;  // 0-100%
static volatile uint16_t preheat_gap_ms = 3;

/* ============ State ============ */
static volatile bool welding_now = false;
static uint32_t last_weld_ms = 0;

static bool armed = false;
static uint32_t armed_until_ms = 0;
static uint32_t boot_ms = 0;

static GPIO_PinState pedal_last_raw = GPIO_PIN_SET;
static GPIO_PinState pedal_stable = GPIO_PIN_SET;
static uint32_t pedal_last_change_ms = 0;

/* ============ UART RX Buffer ============ */
#define RX_LINE_MAX 128
static uint8_t rx_byte;
static char rx_line[RX_LINE_MAX];
static uint8_t rx_idx = 0;
static volatile bool rx_line_ready = false;

/* ============ Helpers ============ */
static void uartSend(const char* s) {
    HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 50);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 50);
}

static inline void pwmOff(void) {
    __HAL_TIM_SET_COMPARE(&htim1, WELD_TIM_CH, 0);
}

static inline void pwmOnDuty(uint16_t duty) {
    if (duty > PWM_MAX) duty = PWM_MAX;
    __HAL_TIM_SET_COMPARE(&htim1, WELD_TIM_CH, duty);
}

static void clampParams(void) {
    if (weld_mode < 1) weld_mode = 1;
    if (weld_mode > 3) weld_mode = 3;

    if (weld_d1_ms > MAX_WELD_MS) weld_d1_ms = MAX_WELD_MS;
    if (weld_d2_ms > MAX_WELD_MS) weld_d2_ms = MAX_WELD_MS;
    if (weld_d3_ms > MAX_WELD_MS) weld_d3_ms = MAX_WELD_MS;

    if (weld_power_pct < 50) weld_power_pct = 50;
    if (weld_power_pct > 100) weld_power_pct = 100;

    if (preheat_pct > 100) preheat_pct = 100;
    if (preheat_ms > MAX_WELD_MS) preheat_ms = MAX_WELD_MS;
}

static void applyArmTimeout(void) {
    if (ARM_TIMEOUT_MS == 0) return;
    if (!armed) return;
    if (armed_until_ms == 0) return;

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - armed_until_ms) >= 0) {
        armed = false;
        armed_until_ms = 0;
        uartSend("EVENT,ARM_TIMEOUT");
    }
}

static uint16_t pctToDuty(uint8_t pct) {
    if (pct >= 100) return PWM_MAX;
    return (uint16_t)((uint32_t)pct * PWM_MAX / 100U);
}

static inline void delayMsExact(uint16_t ms) {
    if (ms == 0) return;
    if (ms > MAX_WELD_MS) ms = MAX_WELD_MS;

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {
        __NOP();
    }
}

static void doPulseMsPwm(uint16_t ms, uint16_t duty) {
    if (ms == 0) return;
    if (ms > MAX_WELD_MS) ms = MAX_WELD_MS;

    pwmOnDuty(duty);
    delayMsExact(ms);
    pwmOff();
}

static void fireRecipe(void) {
    uint32_t now_ms = HAL_GetTick();

    // Boot inhibit
    if (now_ms - boot_ms < BOOT_INHIBIT_MS) {
        char buf[64];
        snprintf(buf, sizeof(buf), "DENY,BOOT_INHIBIT,ms=%lu",
                 BOOT_INHIBIT_MS - (now_ms - boot_ms));
        uartSend(buf);
        return;
    }

    // Arm timeout check
    applyArmTimeout();
    if (!armed) {
        uartSend("DENY,NOT_ARMED");
        return;
    }

    // Already welding check
    if (welding_now) {
        uartSend("DENY,ALREADY_WELDING");
        return;
    }

    // Cooldown check
    uint32_t since = now_ms - last_weld_ms;
    if (since < WELD_COOLDOWN_MS) {
        char buf[64];
        snprintf(buf, sizeof(buf), "DENY,COOLDOWN,ms=%lu",
                 WELD_COOLDOWN_MS - since);
        uartSend(buf);
        return;
    }

    clampParams();

    welding_now = true;
    uartSend("EVENT,WELD_START");

    // Deadtime / ensure off
    pwmOff();
    HAL_Delay(2);

    uint32_t t0 = HAL_GetTick();

    // Preheat pulse
    if (preheat_enabled && preheat_ms > 0) {
        doPulseMsPwm(preheat_ms, pctToDuty(preheat_pct));
        if (preheat_gap_ms > 0) {
            pwmOff();
            delayMsExact(preheat_gap_ms);
        }
    }

    // Main weld pulses
    uint16_t mainDuty = pctToDuty(weld_power_pct);

    if (weld_mode >= 1) {
        doPulseMsPwm(weld_d1_ms, mainDuty);
    }
    if (weld_mode >= 2) {
        if (weld_gap1_ms) delayMsExact(weld_gap1_ms);
        doPulseMsPwm(weld_d2_ms, mainDuty);
    }
    if (weld_mode >= 3) {
        if (weld_gap2_ms) delayMsExact(weld_gap2_ms);
        doPulseMsPwm(weld_d3_ms, mainDuty);
    }

    uint32_t total_ms = HAL_GetTick() - t0;

    pwmOff();
    welding_now = false;
    last_weld_ms = HAL_GetTick();

    // Send detailed completion event
    char buf[256];
    snprintf(buf, sizeof(buf),
             "EVENT,WELD_DONE,total_ms=%lu,mode=%d,d1=%d,gap1=%d,d2=%d,gap2=%d,"
             "d3=%d,"
             "power_pct=%d,preheat_en=%d,preheat_ms=%d,preheat_pct=%d,preheat_"
             "gap_ms=%d",
             total_ms, weld_mode, weld_d1_ms, weld_gap1_ms, weld_d2_ms,
             weld_gap2_ms, weld_d3_ms, weld_power_pct, preheat_enabled ? 1 : 0,
             preheat_ms, preheat_pct, preheat_gap_ms);
    uartSend(buf);
}

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1);  // ← Scale 1 for 100MHz

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;             // 25MHz crystal
    RCC_OscInitStruct.PLL.PLLN = 400;            // ×400
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;  // ÷4 = 100MHz
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        while (1) {
        }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // 100MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   // 50MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   // 100MHz
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) !=
        HAL_OK)  // ← Latency 3 for 100MHz
        while (1) {
        }
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PB12 pedal input pull-up */
    GPIO_InitStruct.Pin = PEDAL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PEDAL_PORT, &GPIO_InitStruct);

    /* PA8 TIM1_CH1 AF1 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA9/PA10 USART1 AF7 */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_USART1_UART_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        while (1) {
        }

    /* Enable UART RX interrupt */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_TIM1_Init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 7;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_MAX;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
        while (1) {
        }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        while (1) {
        }
}

/* ============ UART RX Interrupt Callback ============ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) {
        char c = (char)rx_byte;

        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_line[rx_idx] = '\0';
                rx_line_ready = true;
                rx_idx = 0;
            }
        } else if (rx_idx < RX_LINE_MAX - 1) {
            rx_line[rx_idx++] = c;
        }

        // Re-arm for next byte
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/* ============ Command Parser ============ */
static void parseCommand(char* line) {
    char response[128];

    // ARM,0 or ARM,1
    if (strncmp(line, "ARM,", 4) == 0) {
        int v = atoi(line + 4);
        if (v == 1) {
            armed = true;
            armed_until_ms =
                (ARM_TIMEOUT_MS == 0) ? 0 : (HAL_GetTick() + ARM_TIMEOUT_MS);
            uartSend("ACK,ARM,1");
        } else {
            armed = false;
            armed_until_ms = 0;
            uartSend("ACK,ARM,0");
        }
        return;
    }

    // CMD,SET,PULSE,<d1>
    if (strncmp(line, "CMD,SET,PULSE,", 14) == 0) {
        int val = atoi(line + 14);
        if (val > 0 && val <= MAX_WELD_MS) {
            weld_d1_ms = val;
            snprintf(response, sizeof(response), "ACK,PULSE=%d", weld_d1_ms);
            uartSend(response);
        } else {
            uartSend("ERR,PULSE_RANGE");
        }
        return;
    }

    // SET_PULSE,mode,d1,gap1,d2,gap2,d3
    if (strncmp(line, "SET_PULSE,", 10) == 0) {
        int v_mode = 1, v_d1 = 0, v_gap1 = 0, v_d2 = 0, v_gap2 = 0, v_d3 = 0;
        int n = sscanf(line, "SET_PULSE,%d,%d,%d,%d,%d,%d", &v_mode, &v_d1,
                       &v_gap1, &v_d2, &v_gap2, &v_d3);
        if (n >= 2) {
            weld_mode = (uint8_t)v_mode;
            weld_d1_ms = (uint16_t)v_d1;
            weld_gap1_ms = (uint16_t)v_gap1;
            weld_d2_ms = (uint16_t)v_d2;
            weld_gap2_ms = (uint16_t)v_gap2;
            weld_d3_ms = (uint16_t)v_d3;
            clampParams();
            snprintf(response, sizeof(response), "ACK,SET_PULSE,mode=%d",
                     weld_mode);
            uartSend(response);
        } else {
            uartSend("DENY,BAD_SET_PULSE");
        }
        return;
    }

    // CMD,SET,POWER,<value>
    if (strncmp(line, "CMD,SET,POWER,", 14) == 0) {
        int val = atoi(line + 14);
        if (val >= 50 && val <= 100) {
            weld_power_pct = val;
            snprintf(response, sizeof(response), "ACK,POWER=%d",
                     weld_power_pct);
            uartSend(response);
        } else {
            uartSend("ERR,POWER_RANGE");
        }
        return;
    }

    // SET_POWER,<value>
    if (strncmp(line, "SET_POWER,", 10) == 0) {
        weld_power_pct = (uint8_t)atoi(line + 10);
        clampParams();
        snprintf(response, sizeof(response), "ACK,SET_POWER,pct=%d",
                 weld_power_pct);
        uartSend(response);
        return;
    }

    // SET_PREHEAT,en,ms,pct,gap_ms
    if (strncmp(line, "SET_PREHEAT,", 12) == 0) {
        int v_en = 0, v_ms = 0, v_pct = 0, v_gap = 0;
        int n = sscanf(line, "SET_PREHEAT,%d,%d,%d,%d", &v_en, &v_ms, &v_pct,
                       &v_gap);
        if (n >= 3) {
            preheat_enabled = (v_en == 1);
            preheat_ms = (uint16_t)v_ms;
            preheat_pct = (uint8_t)v_pct;
            if (n >= 4) preheat_gap_ms = (uint16_t)v_gap;
            clampParams();
            snprintf(response, sizeof(response), "ACK,SET_PREHEAT,en=%d",
                     preheat_enabled ? 1 : 0);
            uartSend(response);
        } else {
            uartSend("DENY,BAD_SET_PREHEAT");
        }
        return;
    }

    // CMD,FIRE
    if (strcmp(line, "CMD,FIRE") == 0) {
        fireRecipe();
        return;
    }

    // CMD,ENABLE
    if (strcmp(line, "CMD,ENABLE") == 0) {
        armed = true;
        uartSend("ACK,ENABLED");
        return;
    }

    // CMD,DISABLE
    if (strcmp(line, "CMD,DISABLE") == 0) {
        armed = false;
        uartSend("ACK,DISABLED");
        return;
    }

    // STATUS or CMD,STATUS
    if (strcmp(line, "STATUS") == 0 || strcmp(line, "CMD,STATUS") == 0) {
        int32_t cd =
            (int32_t)WELD_COOLDOWN_MS - (int32_t)(HAL_GetTick() - last_weld_ms);
        if (cd < 0) cd = 0;

        snprintf(response, sizeof(response),
                 "STATUS,armed=%d,cooldown_ms=%ld,welding=%d,mode=%d,power_pct="
                 "%d,preheat_en=%d",
                 armed ? 1 : 0, cd, welding_now ? 1 : 0, weld_mode,
                 weld_power_pct, preheat_enabled ? 1 : 0);
        uartSend(response);
        return;
    }

    // Unknown command
    uartSend("ERR,UNKNOWN_CMD");
}

static void pollPedal(void) {
    GPIO_PinState raw = HAL_GPIO_ReadPin(PEDAL_PORT, PEDAL_PIN);
    uint32_t now = HAL_GetTick();

    if (raw != pedal_last_raw) {
        pedal_last_change_ms = now;
        pedal_last_raw = raw;
    }

    if ((now - pedal_last_change_ms) >= PEDAL_DEBOUNCE_MS) {
        if (raw != pedal_stable) {
            GPIO_PinState prev = pedal_stable;
            pedal_stable = raw;

            /* Falling edge = pedal press */
            if (prev == GPIO_PIN_SET && pedal_stable == GPIO_PIN_RESET) {
                uartSend("EVENT,PEDAL_PRESS");
                fireRecipe();
            }
        }
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef blink = {0};
    blink.Pin = GPIO_PIN_13;
    blink.Mode = GPIO_MODE_OUTPUT_PP;
    blink.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &blink);

    for (int i = 0; i < 10; i++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    pwmOff();

    boot_ms = HAL_GetTick();

    /* init debounce state */
    GPIO_PinState r = HAL_GPIO_ReadPin(PEDAL_PORT, PEDAL_PIN);
    pedal_last_raw = r;
    pedal_stable = r;
    pedal_last_change_ms = boot_ms;

    uartSend("BOOT,STM32_WELD_BRAIN_PWM_READY");

    /* Start UART RX interrupt */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    while (1) {
        pollPedal();
        applyArmTimeout();

        // Process received commands
        if (rx_line_ready) {
            parseCommand(rx_line);
            rx_line_ready = false;
        }
    }
}

// SysTick handler for HAL_GetTick()
void SysTick_Handler(void) { HAL_IncTick(); }

// UART interrupt handler
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }