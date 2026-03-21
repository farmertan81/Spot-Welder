#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32g4xx_hal.h"

/* ===== Forward decls ===== */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);

static void uartSend(const char* s);
static inline void pwmOff(void);
static inline void pwmOnDuty(uint16_t duty);
static void clampParams(void);
static void applyArmTimeout(void);
static void applyReadyTimeout(void);
static uint16_t pctToDuty(uint8_t pct);
static inline void delayMsExact(uint16_t ms);
static void doPulseMsPwm(uint16_t ms, uint16_t duty);
static void fireRecipe(void);
static void parseCommand(char* line);
static void pollPedal(void);
static uint32_t adcReadChannel(uint32_t channel);
static float readThermistor(void);
static float readCapVoltage(void);

/* ============ Pins (fixed) ============ */
#define WELD_TIM TIM1
#define WELD_TIM_CH TIM_CHANNEL_1
#define PEDAL_PORT GPIOB
#define PEDAL_PIN GPIO_PIN_12
#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_6

/* ============ Thermistor / ADC ============ */
#define THERM_SERIES_R 10000.0f
#define THERM_NOMINAL_R 10000.0f
#define THERM_NOMINAL_T 25.0f
#define THERM_BETA 3950.0f
#define THERM_OFFSET_C 0.0f

#define V_CAP_DIVIDER 8.26f

/* ============ Limits / Timing ============ */
static const uint32_t WELD_COOLDOWN_MS = 500;
static const uint16_t MAX_WELD_MS = 200;
static const uint32_t PEDAL_DEBOUNCE_MS = 40;
static const uint32_t BOOT_INHIBIT_MS = 5000;
static const uint32_t ARM_TIMEOUT_MS = 0;
static const uint32_t READY_TIMEOUT_MS = 1500;

/* ============ PWM Settings ============ */
static const uint16_t PWM_MAX = 1023;
static const uint16_t TIM1_PSC = 0;
static const uint32_t TIM1_ARR = 16999;

/* ============ HAL handles ============ */
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;

/* ============ Weld Params (defaults) ============ */
static volatile uint8_t weld_mode = 1;
static volatile uint16_t weld_d1_ms = 10;
static volatile uint16_t weld_gap1_ms = 0;
static volatile uint16_t weld_d2_ms = 0;
static volatile uint16_t weld_gap2_ms = 0;
static volatile uint16_t weld_d3_ms = 0;

static volatile uint8_t weld_power_pct = 100;
static volatile bool preheat_enabled = false;
static volatile uint16_t preheat_ms = 20;
static volatile uint8_t preheat_pct = 30;
static volatile uint16_t preheat_gap_ms = 3;

/* ============ State ============ */
static volatile bool welding_now = false;
static uint32_t last_weld_ms = 0;

static bool armed = false;
static uint32_t armed_until_ms = 0;
static uint32_t boot_ms = 0;

static bool system_ready = false;
static uint32_t ready_until_ms = 0;

static GPIO_PinState pedal_last_raw = GPIO_PIN_SET;
static GPIO_PinState pedal_stable = GPIO_PIN_SET;
static uint32_t pedal_last_change_ms = 0;

static float temp_filtered_c = 25.0f;

/* ============ UART RX Buffer ============ */
#define RX_LINE_MAX 128
static uint8_t rx_byte;
static char rx_build[RX_LINE_MAX];
static uint8_t rx_idx = 0;
static char rx_ready[RX_LINE_MAX];
static volatile bool rx_line_ready = false;

/* ============ UART debug ============ */
#define DEBUG_UART_RX 1
static volatile uint32_t uart_rx_bytes = 0;
static volatile uint32_t uart_rx_errors = 0;
static volatile uint32_t uart_rx_overruns = 0;

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

    uint32_t ccr =
        ((uint32_t)duty * (uint32_t)(TIM1_ARR + 1U)) / (uint32_t)(PWM_MAX + 1U);
    if (ccr > TIM1_ARR) ccr = TIM1_ARR;

    __HAL_TIM_SET_COMPARE(&htim1, WELD_TIM_CH, ccr);
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

static void applyReadyTimeout(void) {
    if (!system_ready) return;
    if (READY_TIMEOUT_MS == 0) return;

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - ready_until_ms) >= 0) {
        system_ready = false;
        ready_until_ms = 0;
        uartSend("EVENT,READY_TIMEOUT");
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

    if (now_ms - boot_ms < BOOT_INHIBIT_MS) {
        char buf[64];
        snprintf(buf, sizeof(buf), "DENY,BOOT_INHIBIT,ms=%lu",
                 (unsigned long)(BOOT_INHIBIT_MS - (now_ms - boot_ms)));
        uartSend(buf);
        return;
    }

    applyArmTimeout();
    applyReadyTimeout();

    if (!system_ready) {
        uartSend("DENY,NOT_READY");
        return;
    }

    if (!armed) {
        uartSend("DENY,NOT_ARMED");
        return;
    }

    if (welding_now) {
        uartSend("DENY,ALREADY_WELDING");
        return;
    }

    uint32_t since = now_ms - last_weld_ms;
    if (since < WELD_COOLDOWN_MS) {
        char buf[64];
        snprintf(buf, sizeof(buf), "DENY,COOLDOWN,ms=%lu",
                 (unsigned long)(WELD_COOLDOWN_MS - since));
        uartSend(buf);
        return;
    }

    clampParams();

    welding_now = true;
    uartSend("EVENT,WELD_START");

    pwmOff();
    HAL_Delay(2);

    uint32_t t0 = HAL_GetTick();

    if (preheat_enabled && preheat_ms > 0) {
        doPulseMsPwm(preheat_ms, pctToDuty(preheat_pct));
        if (preheat_gap_ms > 0) {
            pwmOff();
            delayMsExact(preheat_gap_ms);
        }
    }

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

    char buf[256];
    snprintf(buf, sizeof(buf),
             "EVENT,WELD_DONE,total_ms=%lu,mode=%d,d1=%d,gap1=%d,d2=%d,gap2=%d,"
             "d3=%d,power_pct=%d,preheat_en=%d,preheat_ms=%d,preheat_pct=%d,"
             "preheat_gap_ms=%d",
             (unsigned long)total_ms, weld_mode, weld_d1_ms, weld_gap1_ms,
             weld_d2_ms, weld_gap2_ms, weld_d3_ms, weld_power_pct,
             preheat_enabled ? 1 : 0, preheat_ms, preheat_pct, preheat_gap_ms);
    uartSend(buf);
}

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1) {
        }
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        while (1) {
        }
    }
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PEDAL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PEDAL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                          GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
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

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        while (1) {
        }
    }

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_TIM1_Init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = TIM1_PSC;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = (uint32_t)TIM1_ARR;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        while (1) {
        }
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) !=
        HAL_OK) {
        while (1) {
        }
    }
}

static void MX_ADC1_Init(void) {
    __HAL_RCC_ADC12_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        while (1) {
        }
    }

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

static uint32_t adcReadChannel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return 0xFFFF;
    }

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0xFFFF;
    }

    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

static float readThermistor(void) {
    uint32_t raw = adcReadChannel(ADC_CHANNEL_1);
    if (raw == 0xFFFF || raw < 10 || raw > 4085) return -99.0f;

    float v = (float)raw * (3.3f / 4095.0f);
    if (v <= 0.01f || v >= 3.29f) return -99.0f;

    float r_ntc = THERM_SERIES_R * ((3.3f / v) - 1.0f);
    if (r_ntc <= 0.0f) return -99.0f;

    float s = logf(r_ntc / THERM_NOMINAL_R) / THERM_BETA;
    s += 1.0f / (THERM_NOMINAL_T + 273.15f);
    return (1.0f / s) - 273.15f + THERM_OFFSET_C;
}

static float readCapVoltage(void) {
    uint32_t sumP = 0, sumN = 0;
    const int samples = 16;

    for (int i = 0; i < samples; i++) {
        uint32_t p = adcReadChannel(ADC_CHANNEL_4);
        uint32_t n = adcReadChannel(ADC_CHANNEL_5);
        if (p == 0xFFFF || n == 0xFFFF) return 0.0f;
        sumP += p;
        sumN += n;
    }

    float vP = ((float)sumP / samples) * (3.3f / 4095.0f);
    float vN = ((float)sumN / samples) * (3.3f / 4095.0f);

    return (vP - vN) * V_CAP_DIVIDER;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) {
        char c = (char)rx_byte;
        uart_rx_bytes++;

        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_build[rx_idx] = '\0';
                if (!rx_line_ready) {
                    memcpy(rx_ready, rx_build, rx_idx + 1);
                    rx_line_ready = true;
                }
                rx_idx = 0;
            }
        } else if (rx_idx < RX_LINE_MAX - 1) {
            rx_build[rx_idx++] = c;
        } else {
            rx_idx = 0;
        }

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) {
        uint32_t err = huart->ErrorCode;
        uart_rx_errors++;

        if (err & HAL_UART_ERROR_ORE) uart_rx_overruns++;

        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

static void parseCommand(char* line) {
    char response[192];

    if (strncmp(line, "READY,", 6) == 0) {
        int v = atoi(line + 6);
        if (v == 1) {
            system_ready = true;
            ready_until_ms = HAL_GetTick() + READY_TIMEOUT_MS;
            uartSend("ACK,READY,1");
        } else {
            system_ready = false;
            ready_until_ms = 0;
            uartSend("ACK,READY,0");
        }
        return;
    }

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

    if (strncmp(line, "CMD,SET,PULSE,", 14) == 0) {
        int val = atoi(line + 14);
        if (val > 0 && val <= MAX_WELD_MS) {
            weld_d1_ms = (uint16_t)val;
            snprintf(response, sizeof(response), "ACK,PULSE=%d", weld_d1_ms);
            uartSend(response);
        } else {
            uartSend("ERR,PULSE_RANGE");
        }
        return;
    }

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

    if (strncmp(line, "CMD,SET,POWER,", 14) == 0) {
        int val = atoi(line + 14);
        if (val >= 50 && val <= 100) {
            weld_power_pct = (uint8_t)val;
            snprintf(response, sizeof(response), "ACK,POWER=%d",
                     weld_power_pct);
            uartSend(response);
        } else {
            uartSend("ERR,POWER_RANGE");
        }
        return;
    }

    if (strncmp(line, "SET_POWER,", 10) == 0) {
        weld_power_pct = (uint8_t)atoi(line + 10);
        clampParams();
        snprintf(response, sizeof(response), "ACK,SET_POWER,pct=%d",
                 weld_power_pct);
        uartSend(response);
        return;
    }

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

    if (strcmp(line, "CMD,FIRE") == 0) {
        fireRecipe();
        return;
    }

    if (strcmp(line, "CMD,ENABLE") == 0) {
        armed = true;
        uartSend("ACK,ENABLED");
        return;
    }

    if (strcmp(line, "CMD,DISABLE") == 0) {
        armed = false;
        uartSend("ACK,DISABLED");
        return;
    }

    if (strcmp(line, "STATUS") == 0 || strcmp(line, "CMD,STATUS") == 0) {
        int32_t cd =
            (int32_t)WELD_COOLDOWN_MS - (int32_t)(HAL_GetTick() - last_weld_ms);
        if (cd < 0) cd = 0;

        float vcap = readCapVoltage();

        snprintf(response, sizeof(response),
                 "STATUS,armed=%d,ready=%d,cooldown_ms=%ld,welding=%d,mode=%d,"
                 "power_pct=%d,preheat_en=%d,temp=%.1f,vcap=%.2f",
                 armed ? 1 : 0, system_ready ? 1 : 0, (long)cd,
                 welding_now ? 1 : 0, weld_mode, weld_power_pct,
                 preheat_enabled ? 1 : 0, temp_filtered_c, vcap);
        uartSend(response);
        return;
    }

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
    SystemCoreClockUpdate();
    HAL_InitTick(TICK_INT_PRIORITY);

    MX_GPIO_Init();

    for (int i = 0; i < 3; i++) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(80);
    }

    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_ADC1_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    pwmOff();

    boot_ms = HAL_GetTick();

    GPIO_PinState r = HAL_GPIO_ReadPin(PEDAL_PORT, PEDAL_PIN);
    pedal_last_raw = r;
    pedal_stable = r;
    pedal_last_change_ms = boot_ms;

    uartSend("BOOT,STM32G474_WELD_BRAIN_10KHZ_READY");

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    while (1) {
        pollPedal();
        applyArmTimeout();
        applyReadyTimeout();

        {
            static uint32_t last_temp_ms = 0;
            uint32_t now = HAL_GetTick();
            if ((now - last_temp_ms) >= 1000) {
                last_temp_ms = now;
                float t = readThermistor();
                if (t > -50.0f && t < 200.0f) {
                    temp_filtered_c = (0.1f * t) + (0.9f * temp_filtered_c);
                }
            }
        }

        if (rx_line_ready) {
            char local_line[RX_LINE_MAX];
            __disable_irq();
            memcpy(local_line, rx_ready, sizeof(local_line));
            rx_line_ready = false;
            __enable_irq();
            parseCommand(local_line);
        }

#if DEBUG_UART_RX
        {
            static uint32_t last_health = 0;
            uint32_t now = HAL_GetTick();
            if (now - last_health >= 5000) {
                last_health = now;
                char hb[128];
                snprintf(hb, sizeof(hb),
                         "RXHEALTH,bytes=%lu,errors=%lu,overruns=%lu",
                         (unsigned long)uart_rx_bytes,
                         (unsigned long)uart_rx_errors,
                         (unsigned long)uart_rx_overruns);
                uartSend(hb);

                if (huart1.RxState == HAL_UART_STATE_READY) {
                    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
                    uartSend("RXHEALTH,REARM");
                }
            }
        }
#endif
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }