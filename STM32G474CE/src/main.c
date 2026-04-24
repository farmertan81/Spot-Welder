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
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void i2c_bus_recovery(void);

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
static void pollContactTrigger(void);
static uint32_t adcReadChannel(uint32_t channel);
static uint32_t adcReadChannel2(uint32_t channel);
static float measureVDDA(void);
static float readThermistor(void);
static float readCapVoltage(void);
static bool contactDetectedRaw(float* out_vcap);
static bool contactDetected(void);

/* NEW: Shunt Sensing Forward Decl */
static uint32_t adcReadFast(uint32_t channel);
static uint32_t adcReadFast2(uint32_t channel);
static void capturePulseAmps(uint16_t ms, uint16_t duty);

/* NEW: INA226 Forward Decl */
static bool ina226_write_reg(uint8_t addr, uint8_t reg, uint16_t val);
static bool ina226_read_reg(uint8_t addr, uint8_t reg, uint16_t* val);
static bool ina226_read_bus_voltage(uint8_t addr, float* voltage);
static bool ina226_read_shunt_current(uint8_t addr, float* current);
static bool ina226_health_check(void);
static void ina226_read_all(void);
static void chargerStateMachine(void);
static void sendStatusPacket(void);

/* Waveform capture helpers (Phase 3) */
static uint32_t micros_now(void);
static void start_weld_pulse_capture(void);
static void capture_waveform_samples(uint16_t sample_count);
static void end_weld_pulse_capture(void);
static void apply_waveform_voltage_interpolation(float vcap_start,
                                                 float vcap_end,
                                                 uint16_t pulse_start_index,
                                                 uint16_t pulse_end_index);
static void send_waveform_data(void);
static uint16_t get_planned_active_pulse_ms(void);

/* ============ Pins (fixed) ============ */
#define WELD_TIM TIM1
#define WELD_TIM_CH TIM_CHANNEL_1
#define PEDAL_PORT GPIOB
#define PEDAL_PIN GPIO_PIN_12
#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_6

/* Legacy PA6 contact input is intentionally not used anymore.
 * Contact is now detected via AMC1311B capacitor voltage. */

/* NEW: Charger enable pin */
#define CHARGER_EN_PORT GPIOB
#define CHARGER_EN_PIN GPIO_PIN_2

/* ============ Billet Shunt Calibration (0.3885" @ 15mm) ============ */
/* Physical chain: Amps → µV across shunt → ×41 (AMC1302) → ADC counts */
#define SHUNT_GAIN 8.2f          /* AMC1301 fixed gain                  */
#define SHUNT_EFF_OHMS 0.000050f /* 50 µΩ effective shunt (calibrated) */

/* ============ Current Calibration Factor ============ */
/* Calibrated against 400F capacitance discharge test (2025-04-21):
 *   Discharge test: 8.74V -> 7.70V @30s -> 6.81V @60s, R=0.6Ω
 *   Result: C = 400F (confirmed via exponential decay)
 *   Weld test: ΔV=0.103V over 10ms on 400F caps => I_real = 4,120A
 *   Reported I = 2,339A => error = 1.76x too low
 * Applied as multiplier on computed amps. Adjust if shunt/chain changes.
 */
#define CURRENT_CAL_FACTOR 1.76f

/* ============ Thermistor / ADC ============ */
#define THERM_SERIES_R 10000.0f
#define THERM_NOMINAL_R 10000.0f
#define THERM_NOMINAL_T 25.0f
#define THERM_BETA 3950.0f
#define THERM_OFFSET_C 0.0f

#define V_CAP_DIVIDER 6.0f

/* ============ INA226 Addresses ============ */
#define INA226_ADDR_PACK 0x40
#define INA226_ADDR_NODE1 0x41
#define INA226_ADDR_NODE2 0x44

/* ============ INA226 Registers ============ */
#define INA226_REG_CONFIG 0x00
#define INA226_REG_SHUNT_V 0x01
#define INA226_REG_BUS_V 0x02
#define INA226_REG_CALIBRATION 0x05

/* ============ INA226 Shunt Resistor ============ */
#define INA226_SHUNT_R 0.0002f /* 200 µΩ */
/* Rev1 charge current correction factor (matches ESP32 value) */
#define CHARGE_CURRENT_CORRECTION 1.0f

/* ============ Charger Thresholds (INA226 ONLY) ============ */
#define CHG_VPACK_ON 8.7f   /* Turn charger ON below this */
#define CHG_VPACK_OFF 9.11f /* Turn charger OFF at/above this */

/* ============ INA226 Scaling ============ */
#define V_NODE1_SCALE 0.9965f
#define V_NODE2_SCALE 0.9835f
#define VPACK_SCALE 1.000f

/* ============ Limits / Timing ============ */
static const uint32_t WELD_COOLDOWN_MS = 500;
static const uint16_t MAX_WELD_MS = 200;
static const uint32_t PEDAL_DEBOUNCE_MS = 40;
static const uint32_t BOOT_INHIBIT_MS = 5000;
static const uint32_t ARM_TIMEOUT_MS = 0;
static const uint32_t READY_TIMEOUT_MS = 1500;
static const float CONTACT_THRESHOLD_V_DEFAULT = 1.5f;
static const uint32_t CONTACT_SAMPLE_MS = 10;
static const uint32_t CONTACT_DEBOUNCE_MS = 50;

/* UART TX pacing for long WAVEFORM lines.
 * 256 bytes @115200 baud is ~22ms on wire, so 250ms timeout is generous. */
static const uint32_t UART_TX_CHUNK_SIZE = 256U;
static const uint32_t UART_TX_TIMEOUT_MS = 250U;

/* ============ PWM Settings ============ */
static const uint16_t PWM_MAX = 1023;
static const uint16_t TIM1_PSC = 0;
static const uint32_t TIM1_ARR = 16999;

/* ============ HAL handles ============ */
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
I2C_HandleTypeDef hi2c1;

/* ==== VDDA Calibration ==== */
/* STM32G474: Factory VREFINT cal was measured at VDDA = 3.0V (3000 mV).
 * The HAL/LL headers may define VREFINT_CAL_VREF (in mV) and VREFINT_CAL_ADDR.
 * We use our own names to avoid redefinition warnings/errors. */
#define VREFINT_CAL_MV 3000UL /* Calibration VDDA in millivolts */
#define VREFINT_CAL_PTR \
    ((volatile uint16_t*)0x1FFF75AAUL) /* Factory cal value address    */
static float measured_vdda = 3.3f; /* Updated periodically by measureVDDA() */

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

/* ============ Trigger Settings ============ */
static volatile uint8_t trigger_mode = 1;        // 1 = pedal, 2 = contact
static volatile uint8_t contact_hold_steps = 2;  // each step = 500 ms
static volatile uint8_t contact_with_pedal =
    1;  // 1 = require contact in pedal mode (safe default)

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
static float current_peak_amps = 0.0f;  // Track peak of recipe

/* Added calibration capture values */
static float cal_vcap_before = 0.0f;
static float cal_vcap_after = 0.0f;
static uint32_t cal_adc_peak_raw = 0;
static float cal_current_avg = 0.0f;

/* Configurable lead resistance (default 1.1 mΩ).
 * Measured: 0.54 mΩ + 0.57 mΩ = 1.1 mΩ total (4-wire Kelvin test).
 * 41% improvement over previous leads (1.87 mΩ → 1.1 mΩ). */
static float lead_resistance_ohms = 0.0011f;           // 1.1 mΩ measured
static const float LEAD_RESISTANCE_MIN_OHMS = 0.0001f; /* 0.1 mΩ */
static const float LEAD_RESISTANCE_MAX_OHMS = 0.0100f; /* 10.0 mΩ */

/* ============ Waveform Capture (Phase 3) ============ */
#define WAVEFORM_BUFFER_SIZE 1024
#define WAVEFORM_LINE_BUFFER_SIZE 25000
#define WAVEFORM_CHUNK_SAMPLES 50U
#define WAVEFORM_SAMPLE_INTERVAL_US 100U
#define PWM_PERIOD_US 100U
#define WAVEFORM_PWM_PHASE_SWEEP_STEP_US 12U
#define WAVEFORM_PWM_PHASE_SWEEP_SAMPLES 6U
#define WAVEFORM_PRE_SAMPLES 10U  /* 1ms pre-buffer */
#define WAVEFORM_POST_SAMPLES 10U /* 1ms post-buffer */
#define WAVEFORM_MAX_PULSE_MS 100U
#define WAVEFORM_MAX_ACTIVE_SAMPLES \
    ((WAVEFORM_MAX_PULSE_MS * 1000U) / WAVEFORM_SAMPLE_INTERVAL_US)

/* Capture capacity supports up to 1ms pre + 100ms pulse + 1ms post = 1020
 * samples. */
_Static_assert(WAVEFORM_BUFFER_SIZE >=
                   (WAVEFORM_PRE_SAMPLES + WAVEFORM_MAX_ACTIVE_SAMPLES +
                    WAVEFORM_POST_SAMPLES),
               "WAVEFORM_BUFFER_SIZE too small for 100ms capture window");

/* Safety check: each sample adds ~14 chars (",AAAA.AA,VV.VV") to WAVEFORM line.
 */
_Static_assert(WAVEFORM_BUFFER_SIZE <= 1024,
               "Unexpected waveform buffer growth; re-check UART line budget");
_Static_assert(WAVEFORM_LINE_BUFFER_SIZE >= (WAVEFORM_BUFFER_SIZE * 18),
               "WAVEFORM_LINE_BUFFER_SIZE too small for waveform CSV payload");

typedef struct {
    float current_amps;
    float voltage_volts;
    uint32_t timestamp_us;
} WaveformSample;

static WaveformSample waveform_buffer[WAVEFORM_BUFFER_SIZE];
static volatile uint16_t waveform_index = 0;
static volatile bool waveform_capture_active = false;
static uint32_t waveform_capture_start_us = 0;
static uint32_t waveform_last_sample_us = 0;
static uint16_t waveform_pulse_start_index = 0;
static uint16_t waveform_pulse_end_index = 0;
/* Cached once before each pulse to avoid slow 16x averaged voltage reads during
 * 200us sampling. */
static float cached_vcap = 0.0f;

/* ============ Contact trigger state ============ */
static bool contact_hold_active = false;
static uint32_t contact_hold_start_ms = 0;
static bool contact_fire_lock = false;

/* ==== Voltage-based contact detection state (AMC1311B vcap) ==== */
static float g_contact_threshold_v = 1.5f;
static bool g_contact_state = false;
static bool g_contact_pending_high = false;
static uint32_t g_contact_pending_since_ms = 0;
static uint32_t g_contact_last_sample_ms = 0;
static float g_contact_last_vcap = 0.0f;

/* ============ INA226 state (CONTROL SYSTEM) ============ */
static volatile bool ina226_ok = false;
static float ina_vpack = 0.0f;
static float vlow = 0.0f;      /* TAP_LOW  (0x41) – bottom of stack */
static float vmid = 0.0f;      /* TAP_MID  (0x44) – middle tap      */
static float ina_cell1 = 0.0f; /* Bottom cell: vlow                  */
static float ina_cell2 = 0.0f; /* Middle cell: vmid - vlow           */
static float ina_cell3 = 0.0f; /* Top cell:    vpack - vmid          */
static float ina_ichg = 0.0f;

/* ============ Charger state ============ */
static bool charger_enabled = false;

/* 🔴 Charger lockout (post-weld) */
static bool charger_lockout = false;
static uint32_t charger_lockout_until = 0;

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

/* ============ INA226 Bare-Metal I2C Driver ============ */

static bool ina226_write_reg(uint8_t addr, uint8_t reg, uint16_t val) {
    uint8_t data[2];
    data[0] = (val >> 8) & 0xFF;
    data[1] = val & 0xFF;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), reg,
                                             I2C_MEMADD_SIZE_8BIT, data, 2, 10);
    return (st == HAL_OK);
}

static bool ina226_read_reg(uint8_t addr, uint8_t reg, uint16_t* val) {
    uint8_t data[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg,
                                            I2C_MEMADD_SIZE_8BIT, data, 2, 10);
    if (st != HAL_OK) return false;
    *val = ((uint16_t)data[0] << 8) | data[1];
    return true;
}

static bool ina226_read_bus_voltage(uint8_t addr, float* voltage) {
    uint16_t raw;
    if (!ina226_read_reg(addr, INA226_REG_BUS_V, &raw)) return false;
    *voltage = raw * 0.00125f; /* 1.25 mV/bit */
    return true;
}

static bool ina226_read_shunt_current(uint8_t addr, float* current) {
    uint16_t raw;
    if (!ina226_read_reg(addr, INA226_REG_SHUNT_V, &raw)) return false;
    int16_t signed_raw = (int16_t)raw;
    float shunt_voltage_v = signed_raw * 0.0000025f; /* 2.5 µV/bit */
    float calculated_current = shunt_voltage_v / INA226_SHUNT_R;
    *current = (-calculated_current) * CHARGE_CURRENT_CORRECTION;
    if (fabsf(*current) < 0.010f) *current = 0.0f;
    if (*current > 50.0f) *current = 50.0f;
    return true;
}

static bool ina226_health_check(void) {
    /* Verify all three sensors respond using HAL_I2C_IsDeviceReady (3 trials,
     * 10ms) */
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(INA226_ADDR_PACK << 1), 3,
                              10) != HAL_OK)
        return false;
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(INA226_ADDR_NODE1 << 1), 3,
                              10) != HAL_OK)
        return false;
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(INA226_ADDR_NODE2 << 1), 3,
                              10) != HAL_OK)
        return false;
    return true;
}

static void ina226_read_all(void) {
    /* Step 1: Verify all three sensors are present on the bus */
    if (!ina226_health_check()) {
        ina226_ok = false;
        /* Safety: disable charger if any sensor missing */
        HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_RESET);
        charger_enabled = false;
        return;
    }

    float vp = 0, vl = 0, vm = 0, ichg = 0;
    bool ok = true;

    if (!ina226_read_bus_voltage(INA226_ADDR_PACK, &vp)) ok = false;
    if (!ina226_read_bus_voltage(INA226_ADDR_NODE1, &vl)) ok = false;
    if (!ina226_read_bus_voltage(INA226_ADDR_NODE2, &vm)) ok = false;
    if (!ina226_read_shunt_current(INA226_ADDR_PACK, &ichg)) ok = false;

    ina226_ok = ok;

    if (!ok) {
        /* Safety: disable charger on read failure */
        HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_RESET);
        charger_enabled = false;
        return;
    }

    ina_vpack = vp * VPACK_SCALE;
    vlow = vl * V_NODE1_SCALE;
    vmid = vm * V_NODE2_SCALE;

    /* Correct cell voltage math:
     * [Pack-] ── Cell_Bot ── (vlow/0x41) ── Cell_Mid ── (vmid/0x44) ── Cell_Top
     * ── [Pack+]
     */
    ina_cell1 = vlow;             /* Bottom cell */
    ina_cell2 = vmid - vlow;      /* Middle cell */
    ina_cell3 = ina_vpack - vmid; /* Top cell    */

    ina_ichg = ichg;
}

/* ============ Charger State Machine (INA226 ONLY) ============ */

static void chargerStateMachine(void) {
    uint32_t now = HAL_GetTick();

    /* 🔴 LOCKOUT: block charger after weld */
    if (charger_lockout) {
        if ((int32_t)(now - charger_lockout_until) >= 0) {
            charger_lockout = false;  // release lock
        } else {
            HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_RESET);
            charger_enabled = false;
            return;
        }
    }

    /* SAFETY: INA226 failure → charger OFF */
    if (!ina226_ok) {
        HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_RESET);
        charger_enabled = false;
        return;
    }

    /* DISARMED → charger OFF */
    if (!armed) {
        HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_RESET);
        charger_enabled = false;
        return;
    }

    /* WELDING → charger OFF */
    if (welding_now) {
        HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_RESET);
        charger_enabled = false;
        return;
    }

    /* NORMAL HYSTERESIS CONTROL */
    if (ina_vpack >= CHG_VPACK_OFF) {
        HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_RESET);
        charger_enabled = false;
    } else if (ina_vpack <= CHG_VPACK_ON) {
        HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_SET);
        charger_enabled = true;
    }
}

/* ============ UART Status Packets (split into two for ESP32 parsing)
 * ============ */

static void sendStatusPacket(void) {
    char buf[192];

    /* --- Packet 1: STATUS (system state, ADC-based diagnostics) --- */
    float vcap = readCapVoltage();
    float power_f = (float)weld_power_pct;
    snprintf(buf, sizeof(buf),
             "STATUS,armed=%d,ready=%d,welding=%d,vcap=%.2f,"
             "temp=%.2f,mode=%d,power=%.2f,vdda=%.3f,lead_r_ohm=%.6f",
             armed ? 1 : 0, system_ready ? 1 : 0, welding_now ? 1 : 0, vcap,
             temp_filtered_c, (int)weld_mode, power_f, measured_vdda,
             lead_resistance_ohms);
    uartSend(buf);

    /* --- Packet 2: STATUS2 (INA226 telemetry) --- */
    snprintf(buf, sizeof(buf),
             "STATUS2,ina_ok=%d,chg_en=%d,vpack=%.2f,vlow=%.2f,"
             "vmid=%.2f,cell1=%.2f,cell2=%.2f,cell3=%.2f,ichg=%.2f",
             ina226_ok ? 1 : 0, charger_enabled ? 1 : 0, ina_vpack, vlow, vmid,
             ina_cell1, ina_cell2, ina_cell3, ina_ichg);
    uartSend(buf);
}

/* ============ Shunt Sensing Functions ============ */
static uint32_t adcReadFast(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;  // Speed for pulses
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) return 0xFFFF;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0xFFFF;
    }
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

static uint32_t adcReadFast2(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) return 0xFFFF;
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 1) != HAL_OK) {
        HAL_ADC_Stop(&hadc2);
        return 0xFFFF;
    }
    uint32_t val = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    return val;
}

static void capturePulseAmps(uint16_t ms, uint16_t duty) {
    uint32_t peak_raw = 0;
    float peak_current = 0.0f;
    float sum_current = 0.0f;
    uint32_t sample_count = 0;

    const float v_per_count = measured_vdda / 4095.0f;

    /*
     * IMPORTANT (preheat fix): when PWM duty < 100%, sampling exactly every
     * 100us can phase-lock to TIM1's 100us period and repeatedly hit OFF time,
     * producing false near-zero current during preheat. Sweep several ADC reads
     * across each 100us sample slot and keep the max to reliably capture
     * ON-time current.
     */
    const bool pulse_uses_pwm_window =
        (duty > 0U) && (duty < PWM_MAX) &&
        (WAVEFORM_SAMPLE_INTERVAL_US == PWM_PERIOD_US);

    /* Time-based pulse window: keep MOSFET on until exact deadline.
     * Sampling is anchored to pulse_start_us so waveform samples stay aligned
     * with the real MOSFET on-window. */
    uint32_t pulse_start_us = micros_now();
    uint32_t pulse_deadline_us = pulse_start_us + ((uint32_t)ms * 1000U);
    uint32_t next_sample_us = pulse_start_us;
    uint32_t pulse_target_samples =
        (((uint32_t)ms * 1000U) / WAVEFORM_SAMPLE_INTERVAL_US);

    while ((int32_t)(micros_now() - pulse_deadline_us) < 0) {
        uint32_t now_us = micros_now();
        if ((int32_t)(now_us - next_sample_us) < 0) {
            continue;
        }

        int32_t diff = 0;
        float amps = 0.0f;

        if (pulse_uses_pwm_window) {
            int32_t best_diff = 0;
            float best_amps = 0.0f;
            uint32_t slot_anchor_us = next_sample_us;

            for (uint16_t sweep_i = 0;
                 sweep_i < WAVEFORM_PWM_PHASE_SWEEP_SAMPLES; sweep_i++) {
                uint32_t target_us =
                    slot_anchor_us +
                    ((uint32_t)sweep_i * WAVEFORM_PWM_PHASE_SWEEP_STEP_US);
                while ((int32_t)(micros_now() - target_us) < 0) {
                    __NOP();
                }

                uint32_t p = adcReadFast(ADC_CHANNEL_2);
                uint32_t n = adcReadFast(ADC_CHANNEL_3);
                if (p == 0xFFFF || n == 0xFFFF) {
                    continue;
                }

                int32_t sweep_diff = (int32_t)p - (int32_t)n;
                if (sweep_diff < 0) {
                    sweep_diff = 0;
                }

                float sweep_v_adc = (float)sweep_diff * v_per_count;
                float sweep_v_shunt = sweep_v_adc / SHUNT_GAIN;
                float sweep_amps =
                    (sweep_v_shunt / SHUNT_EFF_OHMS) * CURRENT_CAL_FACTOR;
                if (!isfinite(sweep_amps) || sweep_amps < 0.0f) {
                    sweep_amps = 0.0f;
                }

                if (sweep_amps >= best_amps) {
                    best_amps = sweep_amps;
                    best_diff = sweep_diff;
                }
            }

            diff = best_diff;
            amps = best_amps;
        } else {
            uint32_t p = adcReadFast(ADC_CHANNEL_2);
            uint32_t n = adcReadFast(ADC_CHANNEL_3);
            if (p == 0xFFFF || n == 0xFFFF) {
                continue;
            }

            diff = (int32_t)p - (int32_t)n;
            if (diff < 0) {
                diff = 0;
            }

            float v_adc = (float)diff * v_per_count;
            float v_shunt = v_adc / SHUNT_GAIN;
            amps = (v_shunt / SHUNT_EFF_OHMS) * CURRENT_CAL_FACTOR;
            if (!isfinite(amps) || amps < 0.0f) amps = 0.0f;
        }

        if ((uint32_t)diff > peak_raw) peak_raw = (uint32_t)diff;
        if (amps > peak_current) peak_current = amps;
        sum_current += amps;
        sample_count++;

        if (waveform_capture_active && waveform_index < WAVEFORM_BUFFER_SIZE) {
            waveform_buffer[waveform_index].current_amps = amps;
            waveform_buffer[waveform_index].voltage_volts = cached_vcap;
            waveform_buffer[waveform_index].timestamp_us =
                (uint32_t)(next_sample_us - waveform_capture_start_us);
            waveform_index++;
        }

        waveform_last_sample_us = next_sample_us;
        next_sample_us += WAVEFORM_SAMPLE_INTERVAL_US;

        if (pulse_target_samples > 0U && sample_count >= pulse_target_samples) {
            while ((int32_t)(micros_now() - pulse_deadline_us) < 0) {
                __NOP();
            }
            break;
        }

        now_us = micros_now();
        while ((int32_t)(now_us - next_sample_us) >= 0) {
            next_sample_us += WAVEFORM_SAMPLE_INTERVAL_US;
        }
    }

    uint32_t pulse_end_us = micros_now();
    uint32_t actual_duration_us = pulse_end_us - pulse_start_us;

    char dbg[156];
    snprintf(
        dbg, sizeof(dbg),
        "DBG,PULSE_TIMING,pulse_start_us=%lu,pulse_end_us=%lu,actual_duration_"
        "us=%lu,target_duration_us=%lu,pwm_duty=%u,phase_sweep=%u",
        (unsigned long)pulse_start_us, (unsigned long)pulse_end_us,
        (unsigned long)actual_duration_us,
        (unsigned long)((uint32_t)ms * 1000U), (unsigned int)duty,
        pulse_uses_pwm_window ? 1U : 0U);
    uartSend(dbg);

    cal_adc_peak_raw = peak_raw;
    if (peak_current > current_peak_amps) current_peak_amps = peak_current;

    cal_current_avg =
        (sample_count > 0U) ? (sum_current / (float)sample_count) : 0.0f;
}

/* ============ Waveform Capture Helpers (Phase 3) ============ */
static uint32_t micros_now(void) {
    /* Use DWT cycle counter for microsecond timing. Fallback to ms tick if
     * unavailable. */
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U && SystemCoreClock != 0U) {
        return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
    }
    return HAL_GetTick() * 1000U;
}

static void start_weld_pulse_capture(void) {
    waveform_index = 0;
    waveform_pulse_start_index = 0;
    waveform_pulse_end_index = 0;
    waveform_capture_start_us = micros_now();
    waveform_last_sample_us = waveform_capture_start_us;
    waveform_capture_active = true;
    uartSend("DBG,WAVEFORM_CAPTURE_START");
}

static void capture_waveform_samples(uint16_t sample_count) {
    if (!waveform_capture_active || sample_count == 0U) {
        return;
    }

    const float v_per_count = measured_vdda / 4095.0f;
    uint32_t next_sample_us =
        waveform_last_sample_us + WAVEFORM_SAMPLE_INTERVAL_US;

    for (uint16_t captured = 0; captured < sample_count;) {
        if (waveform_index >= WAVEFORM_BUFFER_SIZE) {
            break;
        }

        uint32_t now_us = micros_now();
        if ((int32_t)(now_us - next_sample_us) < 0) {
            continue;
        }

        uint32_t p = adcReadFast(ADC_CHANNEL_2);
        uint32_t n = adcReadFast(ADC_CHANNEL_3);
        if (p == 0xFFFF || n == 0xFFFF) {
            continue;
        }

        int32_t diff = (int32_t)p - (int32_t)n;
        if (diff < 0) {
            diff = 0;
        }

        float v_adc = (float)diff * v_per_count;
        float v_shunt = v_adc / SHUNT_GAIN;
        float amps = (v_shunt / SHUNT_EFF_OHMS) * CURRENT_CAL_FACTOR;
        if (!isfinite(amps) || amps < 0.0f) amps = 0.0f;

        waveform_buffer[waveform_index].current_amps = amps;
        waveform_buffer[waveform_index].voltage_volts = cached_vcap;
        waveform_buffer[waveform_index].timestamp_us =
            (uint32_t)(next_sample_us - waveform_capture_start_us);
        waveform_index++;
        captured++;

        waveform_last_sample_us = next_sample_us;
        next_sample_us += WAVEFORM_SAMPLE_INTERVAL_US;

        while ((int32_t)(now_us - next_sample_us) >= 0) {
            next_sample_us += WAVEFORM_SAMPLE_INTERVAL_US;
        }
    }
}

static void end_weld_pulse_capture(void) {
    waveform_capture_active = false;
    char dbg[96];
    snprintf(dbg, sizeof(dbg),
             "DBG,WAVEFORM_CAPTURE_STOP,count=%u,pulse_start_idx=%u,pulse_end_"
             "idx=%u",
             (unsigned int)waveform_index,
             (unsigned int)waveform_pulse_start_index,
             (unsigned int)waveform_pulse_end_index);
    uartSend(dbg);
}

static void apply_waveform_voltage_interpolation(float vcap_start,
                                                 float vcap_end,
                                                 uint16_t pulse_start_index,
                                                 uint16_t pulse_end_index) {
    if (waveform_index == 0U) {
        return;
    }

    if (pulse_start_index > waveform_index) {
        pulse_start_index = waveform_index;
    }

    if (pulse_end_index < pulse_start_index) {
        pulse_end_index = pulse_start_index;
    }

    if (pulse_end_index > waveform_index) {
        pulse_end_index = waveform_index;
    }

    for (uint16_t i = 0; i < pulse_start_index; i++) {
        waveform_buffer[i].voltage_volts = vcap_start;
    }

    if (pulse_end_index > pulse_start_index) {
        const float dv = vcap_start - vcap_end;
        const float denom = (float)(pulse_end_index - pulse_start_index);

        for (uint16_t i = pulse_start_index; i < pulse_end_index; i++) {
            const float t = (float)(i - pulse_start_index) / denom;
            waveform_buffer[i].voltage_volts = vcap_start - (dv * t);
        }
    }

    for (uint16_t i = pulse_end_index; i < waveform_index; i++) {
        waveform_buffer[i].voltage_volts = vcap_end;
    }
}

static void send_waveform_data(void) {
    if (waveform_index == 0) {
        uartSend("DBG,WAVEFORM_EMPTY");
        return;
    }

    char line[WAVEFORM_LINE_BUFFER_SIZE];
    int n = snprintf(line, sizeof(line), "WAVEFORM_START,%u,%u,%u",
                     (unsigned int)waveform_index,
                     (unsigned int)waveform_pulse_start_index,
                     (unsigned int)waveform_pulse_end_index);
    if (n > 0 && n < (int)sizeof(line)) {
        uartSend(line);
    }

    for (uint16_t chunk_start = 0; chunk_start < waveform_index;
         chunk_start += (uint16_t)WAVEFORM_CHUNK_SAMPLES) {
        uint16_t remaining = (uint16_t)(waveform_index - chunk_start);
        uint16_t chunk_count = (remaining > (uint16_t)WAVEFORM_CHUNK_SAMPLES)
                                   ? (uint16_t)WAVEFORM_CHUNK_SAMPLES
                                   : remaining;

        n = snprintf(line, sizeof(line), "WAVEFORM_DATA,%u,%u",
                     (unsigned int)chunk_start, (unsigned int)chunk_count);

        for (uint16_t i = 0; i < chunk_count; i++) {
            uint16_t sample_idx = (uint16_t)(chunk_start + i);
            if (n <= 0 || n >= (int)sizeof(line)) break;
            n += snprintf(line + n, sizeof(line) - (size_t)n, ",%.2f,%.2f",
                          waveform_buffer[sample_idx].current_amps,
                          waveform_buffer[sample_idx].voltage_volts);
            if (n >= (int)sizeof(line)) break;
        }

        if (n <= 0 || n >= (int)sizeof(line)) {
            char warn[96];
            snprintf(warn, sizeof(warn),
                     "DBG,WAVEFORM_CHUNK_TRUNCATED,start=%u,count=%u,buf=%u",
                     (unsigned int)chunk_start, (unsigned int)chunk_count,
                     (unsigned int)sizeof(line));
            uartSend(warn);
            continue;
        }

        uartSend(line);
    }

    uartSend("WAVEFORM_END");

    char dbg[96];
    snprintf(dbg, sizeof(dbg),
             "DBG,WAVEFORM_TX,count=%u,chunks=%u,chunk_size=%u,format=CHUNKED",
             (unsigned int)waveform_index,
             (unsigned int)((waveform_index + WAVEFORM_CHUNK_SAMPLES - 1U) /
                            WAVEFORM_CHUNK_SAMPLES),
             (unsigned int)WAVEFORM_CHUNK_SAMPLES);
    uartSend(dbg);
}

static uint16_t get_planned_active_pulse_ms(void) {
    uint32_t active_ms = 0U;

    /* FINDING: preheat path uses the same TIM1 PWM pulse engine (doPulseMsPwm)
     * as the main weld pulse; it is not simple GPIO on/off toggling. */
    if (preheat_enabled && preheat_ms > 0U) {
        active_ms += preheat_ms;

        /* Preheat OFF gap is now waveform-captured to make the drop visible. */
        if (weld_mode >= 1U && preheat_gap_ms > 0U) {
            active_ms += preheat_gap_ms;
        }
    }

    if (weld_mode >= 1U) active_ms += weld_d1_ms;
    if (weld_mode >= 2U) active_ms += weld_d2_ms;
    if (weld_mode >= 3U) active_ms += weld_d3_ms;

    if (active_ms > WAVEFORM_MAX_PULSE_MS) {
        active_ms = WAVEFORM_MAX_PULSE_MS;
    }

    return (uint16_t)active_ms;
}

/* ============ Helpers ============ */
static void uartSend(const char* s) {
    if (s == NULL) {
        return;
    }

    const uint8_t* msg = (const uint8_t*)s;
    size_t remaining = strlen(s);

    while (remaining > 0U) {
        uint16_t chunk = (remaining > UART_TX_CHUNK_SIZE)
                             ? (uint16_t)UART_TX_CHUNK_SIZE
                             : (uint16_t)remaining;

        HAL_StatusTypeDef st = HAL_UART_Transmit(&huart1, (uint8_t*)msg, chunk,
                                                 UART_TX_TIMEOUT_MS);
        if (st != HAL_OK) {
            break;
        }

        msg += chunk;
        remaining -= chunk;

        if (remaining > 0U) {
            HAL_Delay(1);
        }
    }

    static const uint8_t crlf[2] = {'\r', '\n'};
    (void)HAL_UART_Transmit(&huart1, (uint8_t*)crlf, 2U, UART_TX_TIMEOUT_MS);
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

static bool contactDetectedRaw(float* out_vcap) {
    float vcap = readCapVoltage();
    if (out_vcap != NULL) {
        *out_vcap = vcap;
    }

    return (vcap > g_contact_threshold_v);
}

static bool contactDetected(void) {
    uint32_t now = HAL_GetTick();

    if ((now - g_contact_last_sample_ms) < CONTACT_SAMPLE_MS) {
        return g_contact_state;
    }
    g_contact_last_sample_ms = now;

    bool current_raw = contactDetectedRaw(&g_contact_last_vcap);

    if (!current_raw) {
        g_contact_state = false;
        g_contact_pending_high = false;
        g_contact_pending_since_ms = 0;
        return g_contact_state;
    }

    if (g_contact_state) {
        return true;
    }

    if (!g_contact_pending_high) {
        g_contact_pending_high = true;
        g_contact_pending_since_ms = now;
        return false;
    }

    if ((now - g_contact_pending_since_ms) >= CONTACT_DEBOUNCE_MS) {
        g_contact_state = true;
    }

    return g_contact_state;
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

    if (trigger_mode < 1) trigger_mode = 1;
    if (trigger_mode > 2) trigger_mode = 2;

    if (contact_hold_steps < 1) contact_hold_steps = 1;
    if (contact_hold_steps > 10) contact_hold_steps = 10;
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
    float duty_pct;

    // Empirical calibration from measured 50% power data at constant charge
    // level:
    // - duty=723 (70.7%) -> 1502A (~37% of 4033A)
    // - duty=788 (77.0%) -> 2378A (~59% of 4033A)
    // - duty=870 (85.0%) -> 2656A (~66% of 4033A)
    // Target: ~2017A (50% of 4033A)
    // Linear interpolation between duty=723 and duty=788 gives ~duty=761,
    // which is ~74% duty at 50% power. Use piecewise-linear mapping around 74%.
    // Expected duty checkpoints: 25%->379, 50%->757, 75%->890, 100%->1023.
    if (pct == 0U) {
        return 0U;
    } else if (pct <= 50U) {
        // 0-50% power -> 0-74% duty (linear)
        duty_pct = ((float)pct / 50.0f) * 0.74f;
    } else if (pct < 100U) {
        // 50-100% power -> 74-100% duty (linear)
        duty_pct = 0.74f + (((float)pct - 50.0f) / 50.0f) * 0.26f;
    } else {
        duty_pct = 1.0f;
    }

    uint16_t duty = (uint16_t)lroundf(duty_pct * (float)PWM_MAX);

    // Clamp to valid range
    if (duty > PWM_MAX) duty = PWM_MAX;

    return duty;
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

    /*
     * IMPORTANT: keep inter-pulse timing deterministic.
     * Do not perform slow ADC reads here, otherwise OFF gaps get stretched
     * before the next PWM ON edge.
     */
    pwmOnDuty(duty);

    capturePulseAmps(ms, duty);  // Measure while FET is on
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

    if (temp_filtered_c > 65.0f) {
        uartSend("DENY,OVERHEAT");
        return;
    }

    {
        bool need_contact = true;
        if (trigger_mode == 1 && contact_with_pedal == 0) {
            need_contact = false;  // pedal mode with contact check disabled
        }
        if (need_contact && !contactDetected()) {
            uartSend("DENY,NO_CONTACT");
            return;
        }
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

    const uint16_t planned_pulse_ms = get_planned_active_pulse_ms();
    uint32_t planned_active_samples =
        ((uint32_t)planned_pulse_ms * 1000U) / WAVEFORM_SAMPLE_INTERVAL_US;
    uint32_t planned_total_samples = (uint32_t)WAVEFORM_PRE_SAMPLES +
                                     planned_active_samples +
                                     (uint32_t)WAVEFORM_POST_SAMPLES;
    if (planned_total_samples > WAVEFORM_BUFFER_SIZE) {
        planned_total_samples = WAVEFORM_BUFFER_SIZE;
    }

    /* === WELD SEQUENCE (modified per refactor plan) === */

    /* Step 1: Disable charger */
    HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_RESET);
    charger_enabled = false;

    /* Step 2: Wait 50ms for charger to settle */
    HAL_Delay(50);

    welding_now = true;
    current_peak_amps = 0.0f;
    /* Step 3: ADC diagnostic snapshot BEFORE weld (diagnostic only) */
    cal_vcap_before = readCapVoltage();
    cached_vcap = cal_vcap_before;

    uartSend("EVENT,WELD_START");

    pwmOff();
    HAL_Delay(2);

    /* Start capture before pulse so waveform includes 1ms pre-buffer baseline
     * before MOSFET firing (samples 0-9). */
    start_weld_pulse_capture();
    capture_waveform_samples((uint16_t)WAVEFORM_PRE_SAMPLES);
    waveform_pulse_start_index = (uint16_t)WAVEFORM_PRE_SAMPLES;

    /* Measure weld duration with microsecond precision to avoid +1ms skew from
     * HAL tick quantization while preserving waveform alignment. */
    uint32_t weld_start_us = micros_now();

    /* === Execute weld pulse (HRTIM/PWM code UNTOUCHED) === */
    if (preheat_enabled && preheat_ms > 0) {
        uint16_t preheat_start_idx = waveform_index;
        uint16_t preheat_duty = pctToDuty(preheat_pct);
        uint16_t preheat_linear_duty =
            (uint16_t)(((uint32_t)preheat_pct * PWM_MAX) / 100U);

        {
            char cdbg[160];
            snprintf(
                cdbg, sizeof(cdbg),
                "DBG,PWM_COMP,phase=preheat,pct=%u,linear_duty=%u,comp_duty=%u",
                (unsigned int)preheat_pct, (unsigned int)preheat_linear_duty,
                (unsigned int)preheat_duty);
            uartSend(cdbg);
        }

        doPulseMsPwm(preheat_ms, preheat_duty);

        {
            uint16_t preheat_end_idx = waveform_index;
            uint16_t preheat_samples =
                (preheat_end_idx >= preheat_start_idx)
                    ? (uint16_t)(preheat_end_idx - preheat_start_idx)
                    : 0U;
            float preheat_max_amps = 0.0f;
            float preheat_sum_amps = 0.0f;
            for (uint16_t i = preheat_start_idx; i < preheat_end_idx; i++) {
                float a = waveform_buffer[i].current_amps;
                if (a > preheat_max_amps) preheat_max_amps = a;
                preheat_sum_amps += a;
            }
            float preheat_avg_amps =
                (preheat_samples > 0U)
                    ? (preheat_sum_amps / (float)preheat_samples)
                    : 0.0f;

            char pdbg[220];
            snprintf(pdbg, sizeof(pdbg),
                     "DBG,PREHEAT_CAPTURE_END,cfg_ms=%u,duty=%u,samples=%u,"
                     "start_idx=%u,end_idx=%u,avg_a=%.2f,max_a=%.2f",
                     (unsigned int)preheat_ms, (unsigned int)preheat_duty,
                     (unsigned int)preheat_samples,
                     (unsigned int)preheat_start_idx,
                     (unsigned int)preheat_end_idx, preheat_avg_amps,
                     preheat_max_amps);
            uartSend(pdbg);
        }

        if (preheat_gap_ms > 0U) {
            pwmOff();  // hard-off during gap

            uint32_t gap_start_us = micros_now();
            uint32_t gap_deadline_us =
                gap_start_us + ((uint32_t)preheat_gap_ms * 1000U);
            uint16_t gap_start_idx = waveform_index;
            uint16_t planned_gap_samples =
                (uint16_t)(((uint32_t)preheat_gap_ms * 1000U) /
                           WAVEFORM_SAMPLE_INTERVAL_US);

            {
                char gdbg[140];
                snprintf(gdbg, sizeof(gdbg),
                         "DBG,PREHEAT_GAP_START,cfg_ms=%u,start_us=%lu,mosfet="
                         "OFF,planned_samples=%u",
                         (unsigned int)preheat_gap_ms,
                         (unsigned long)gap_start_us,
                         (unsigned int)planned_gap_samples);
                uartSend(gdbg);
            }

            if (planned_gap_samples > 0U) {
                capture_waveform_samples(planned_gap_samples);
            }

            while ((int32_t)(micros_now() - gap_deadline_us) < 0) {
                __NOP();
            }

            uint32_t gap_end_us = micros_now();
            uint16_t gap_end_idx = waveform_index;
            uint16_t captured_gap_samples =
                (gap_end_idx >= gap_start_idx)
                    ? (uint16_t)(gap_end_idx - gap_start_idx)
                    : 0U;

            float gap_max_amps = 0.0f;
            float gap_sum_amps = 0.0f;
            for (uint16_t i = gap_start_idx; i < gap_end_idx; i++) {
                float a = waveform_buffer[i].current_amps;
                if (a > gap_max_amps) gap_max_amps = a;
                gap_sum_amps += a;
            }
            float gap_avg_amps =
                (captured_gap_samples > 0U)
                    ? (gap_sum_amps / (float)captured_gap_samples)
                    : 0.0f;

            {
                char gdbg[196];
                snprintf(gdbg, sizeof(gdbg),
                         "DBG,PREHEAT_GAP_END,cfg_ms=%u,actual_us=%lu,captured_"
                         "samples=%u,avg_a=%.2f,max_a=%.2f,mosfet=OFF",
                         (unsigned int)preheat_gap_ms,
                         (unsigned long)(gap_end_us - gap_start_us),
                         (unsigned int)captured_gap_samples, gap_avg_amps,
                         gap_max_amps);
                uartSend(gdbg);
            }
        }
    }

    uint16_t mainDuty = pctToDuty(weld_power_pct);
    uint16_t main_linear_duty =
        (uint16_t)(((uint32_t)weld_power_pct * PWM_MAX) / 100U);

    {
        char cdbg[160];
        snprintf(cdbg, sizeof(cdbg),
                 "DBG,PWM_COMP,phase=main,pct=%u,linear_duty=%u,comp_duty=%u",
                 (unsigned int)weld_power_pct, (unsigned int)main_linear_duty,
                 (unsigned int)mainDuty);
        uartSend(cdbg);
    }

    if (weld_mode >= 1) {
        uint16_t main1_start_idx = waveform_index;
        doPulseMsPwm(weld_d1_ms, mainDuty);
        uint16_t main1_end_idx = waveform_index;
        char mdbg[180];
        snprintf(
            mdbg, sizeof(mdbg),
            "DBG,MAIN1_CAPTURE_END,cfg_ms=%u,samples=%u,start_idx=%u,end_idx=%"
            "u",
            (unsigned int)weld_d1_ms,
            (unsigned int)((main1_end_idx >= main1_start_idx)
                               ? (uint16_t)(main1_end_idx - main1_start_idx)
                               : 0U),
            (unsigned int)main1_start_idx, (unsigned int)main1_end_idx);
        uartSend(mdbg);
    }
    if (weld_mode >= 2) {
        if (weld_gap1_ms) delayMsExact(weld_gap1_ms);
        uint16_t main2_start_idx = waveform_index;
        doPulseMsPwm(weld_d2_ms, mainDuty);
        uint16_t main2_end_idx = waveform_index;
        char mdbg[180];
        snprintf(
            mdbg, sizeof(mdbg),
            "DBG,MAIN2_CAPTURE_END,cfg_ms=%u,samples=%u,start_idx=%u,end_idx=%"
            "u",
            (unsigned int)weld_d2_ms,
            (unsigned int)((main2_end_idx >= main2_start_idx)
                               ? (uint16_t)(main2_end_idx - main2_start_idx)
                               : 0U),
            (unsigned int)main2_start_idx, (unsigned int)main2_end_idx);
        uartSend(mdbg);
    }
    if (weld_mode >= 3) {
        if (weld_gap2_ms) delayMsExact(weld_gap2_ms);
        uint16_t main3_start_idx = waveform_index;
        doPulseMsPwm(weld_d3_ms, mainDuty);
        uint16_t main3_end_idx = waveform_index;
        char mdbg[180];
        snprintf(
            mdbg, sizeof(mdbg),
            "DBG,MAIN3_CAPTURE_END,cfg_ms=%u,samples=%u,start_idx=%u,end_idx=%"
            "u",
            (unsigned int)weld_d3_ms,
            (unsigned int)((main3_end_idx >= main3_start_idx)
                               ? (uint16_t)(main3_end_idx - main3_start_idx)
                               : 0U),
            (unsigned int)main3_start_idx, (unsigned int)main3_end_idx);
        uartSend(mdbg);
    }

    uint32_t weld_end_us = micros_now();
    uint32_t total_us = weld_end_us - weld_start_us;
    uint32_t total_ms = (total_us + 500U) / 1000U;

    pwmOff();

    /* Active pulse end is derived from configured pulse_ms, not loop latency.
     * Formula: pulse_end_idx = pre_samples + (pulse_ms * 10 samples/ms). */
    {
        uint32_t pulse_end_index =
            (uint32_t)waveform_pulse_start_index + planned_active_samples;
        if (pulse_end_index > planned_total_samples) {
            pulse_end_index = planned_total_samples;
        }
        if (pulse_end_index > WAVEFORM_BUFFER_SIZE) {
            pulse_end_index = WAVEFORM_BUFFER_SIZE;
        }
        waveform_pulse_end_index = (uint16_t)pulse_end_index;
    }
    if (waveform_index < WAVEFORM_BUFFER_SIZE) {
        uint16_t remaining_samples =
            (uint16_t)(WAVEFORM_BUFFER_SIZE - waveform_index);
        uint16_t post_samples =
            (remaining_samples < (uint16_t)WAVEFORM_POST_SAMPLES)
                ? remaining_samples
                : (uint16_t)WAVEFORM_POST_SAMPLES;
        capture_waveform_samples(post_samples);
    }

    /* Phase 3: stop waveform capture after full window. */
    end_weld_pulse_capture();

    /* Timing/debug uses configured pulse length and actual captured samples. */
    {
        uint32_t pulse_samples =
            (waveform_pulse_end_index > waveform_pulse_start_index)
                ? ((uint32_t)waveform_pulse_end_index -
                   (uint32_t)waveform_pulse_start_index)
                : 0U;
        uint32_t pulse_us = pulse_samples * WAVEFORM_SAMPLE_INTERVAL_US;
        total_ms = (pulse_us + 500U) / 1000U;

        uint32_t capture_ms =
            ((uint32_t)WAVEFORM_PRE_SAMPLES * WAVEFORM_SAMPLE_INTERVAL_US +
             (uint32_t)planned_pulse_ms * 1000U +
             (uint32_t)WAVEFORM_POST_SAMPLES * WAVEFORM_SAMPLE_INTERVAL_US +
             500U) /
            1000U;

        char tdbg[192];
        snprintf(tdbg, sizeof(tdbg),
                 "DBG,WAVEFORM_TIMING,pulse_ms=%lu,capture_ms=%lu,wf_samples=%"
                 "u,planned_samples=%lu,pulse_start_idx=%u,pulse_end_idx=%u",
                 (unsigned long)total_ms, (unsigned long)capture_ms,
                 (unsigned int)waveform_index,
                 (unsigned long)planned_total_samples,
                 (unsigned int)waveform_pulse_start_index,
                 (unsigned int)waveform_pulse_end_index);
        uartSend(tdbg);
    }

    welding_now = false;
    last_weld_ms = HAL_GetTick();

    /* 🔴 Engage post-weld lockout (MOVED HERE) */
    charger_lockout = true;
    charger_lockout_until = HAL_GetTick() + 500;

    /* Step 5: Wait 50ms post-weld */
    HAL_Delay(50);

    /* Step 6: ADC diagnostic snapshot AFTER weld (diagnostic only) */
    cal_vcap_after = readCapVoltage();

    /* Voltage overlay: hold pre-pulse at vcap_before, interpolate during active
     * pulse window, then hold post-pulse at vcap_after. */
    apply_waveform_voltage_interpolation(cal_vcap_before, cal_vcap_after,
                                         waveform_pulse_start_index,
                                         waveform_pulse_end_index);
    {
        char vdbg[160];
        snprintf(
            vdbg, sizeof(vdbg),
            "DBG,WAVEFORM_VOLTAGE,method=pre_hold_pulse_interp_post_hold,start="
            "%.3f,end=%.3f,count=%u,pulse_start_idx=%u,pulse_end_idx=%u",
            cal_vcap_before, cal_vcap_after, (unsigned int)waveform_index,
            (unsigned int)waveform_pulse_start_index,
            (unsigned int)waveform_pulse_end_index);
        uartSend(vdbg);
    }

    float delta_v = cal_vcap_before - cal_vcap_after;
    float time_s = total_ms / 1000.0f;

    /* Average lead drop and I^2R loss from live configurable lead resistance.
     */
    float v_drop_leads = cal_current_avg * lead_resistance_ohms;
    float lead_loss_joules =
        cal_current_avg * cal_current_avg * lead_resistance_ohms * time_s;

    /* Gross capacitor-delivered energy minus lead I^2R losses. */
    float gross_energy_joules = cal_vcap_before * cal_current_avg * time_s;
    float energy_joules = gross_energy_joules - lead_loss_joules;
    if (!isfinite(energy_joules) || energy_joules < 0.0f) {
        energy_joules = 0.0f;
    }

    /* Equivalent average voltage delivered at the weld tips. */
    float v_at_tips = (cal_current_avg > 0.0f && time_s > 0.0f)
                          ? (energy_joules / (cal_current_avg * time_s))
                          : (cal_vcap_before - v_drop_leads);
    if (!isfinite(v_at_tips) || v_at_tips < 0.0f) v_at_tips = 0.0f;

    char buf[384];
    snprintf(buf, sizeof(buf),
             "EVENT,WELD_DONE,total_ms=%lu,mode=%d,d1=%d,gap1=%d,d2=%d,gap2=%d,"
             "d3=%d,power_pct=%d,preheat_en=%d,preheat_ms=%d,"
             "peak_a=%.1f,adc_raw=%lu,vcap_b=%.3f,vcap_a=%.3f,delta_v=%.3f,"
             "avg_a=%.1f,v_tips=%.3f,energy_j=%.2f,pulse_start_sample=%u,pulse_"
             "end_sample=%u,wf_samples=%u",
             (unsigned long)total_ms, weld_mode, weld_d1_ms, weld_gap1_ms,
             weld_d2_ms, weld_gap2_ms, weld_d3_ms, weld_power_pct,
             preheat_enabled ? 1 : 0, preheat_ms, current_peak_amps,
             (unsigned long)cal_adc_peak_raw, cal_vcap_before, cal_vcap_after,
             delta_v, cal_current_avg, v_at_tips, energy_joules,
             (unsigned int)waveform_pulse_start_index,
             (unsigned int)waveform_pulse_end_index,
             (unsigned int)waveform_index);
    uartSend(buf);

    /* Phase 3: transmit waveform CSV burst right after weld completion event.
     */
    send_waveform_data();
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

    GPIO_InitStruct.Pin =
        GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Legacy PA6 input kept configured as a hardware fallback only.
     * Runtime contact detection now uses AMC1311B vcap thresholding. */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* NEW: PB2 = CHARGER_EN output, default LOW (charger disabled) */
    GPIO_InitStruct.Pin = CHARGER_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CHARGER_EN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(CHARGER_EN_PORT, CHARGER_EN_PIN, GPIO_PIN_RESET);

    /* I2C1 pins: PA15 = SCL, PB7 = SDA (configured in MX_I2C1_Init)
     * Changed from PB8/PB9 which interfered with DFU boot.
     * PA14 (SWCLK) avoided – SWD pull-down conflicts with I2C SDA. */
}

static void MX_USART1_UART_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 460800;
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

    /* Enable TIM1 Update Interrupt */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
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

/**
 * @brief ADC2 initialization for PA4 (AMC1311B OUTN = ADC2_IN17).
 *
 * PA4 is NOT available on ADC1; it maps to ADC2 Channel 17 on STM32G474.
 * Same settings as ADC1 for consistency.
 */
static void MX_ADC2_Init(void) {
    /* ADC12 clock already enabled by MX_ADC1_Init */

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation = 0;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc2.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        while (1) {
        }
    }

    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
}

/**
 * @brief Read a single channel from ADC2 with long sampling time.
 */
static uint32_t adcReadChannel2(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        return 0xFFFF;
    }

    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc2);
        return 0xFFFF;
    }

    uint32_t val = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    return val;
}

/**
 * @brief Measure actual VDDA using internal VREFINT channel.
 *
 * Uses factory calibration value stored in ROM at VREFINT_CAL_PTR.
 * Formula: VDDA = (VREFINT_CAL_MV / 1000) × (*VREFINT_CAL_PTR) / ADC_reading
 *
 * Returns measured VDDA in volts (typically 3.2–3.4V).
 * Falls back to 3.3V on error.
 */
static float measureVDDA(void) {
    /* VREFINT is on ADC1 internal channel 18 for STM32G474 */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return 3.3f;
    }

    /* Average 16 readings for stability */
    uint32_t sum = 0;
    const int samples = 16;
    for (int i = 0; i < samples; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
            HAL_ADC_Stop(&hadc1);
            return 3.3f;
        }
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    float avg_raw = (float)sum / (float)samples;
    if (avg_raw < 100.0f) return 3.3f; /* Sanity check */

    uint16_t vrefint_cal = *VREFINT_CAL_PTR;
    if (vrefint_cal < 100) return 3.3f; /* Sanity check on factory cal */

    /* VREFINT_CAL_MV is in millivolts (3000), convert to volts for result */
    float vdda =
        ((float)VREFINT_CAL_MV / 1000.0f) * (float)vrefint_cal / avg_raw;

    /* Sanity clamp: VDDA should be 2.5V–3.6V for STM32G474 */
    if (vdda < 2.5f) vdda = 3.3f;
    if (vdda > 3.6f) vdda = 3.3f;

    return vdda;
}

/**
 * @brief I2C bus recovery – unstick hung I2C devices.
 * Toggles SCL 9 times with SDA released to complete any partial transaction.
 * Must be called BEFORE MX_I2C1_Init() to clear stuck bus conditions.
 *
 * Pins: PA15 = I2C1_SCL, PB7 = I2C1_SDA
 */
static void i2c_bus_recovery(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Configure PA15 (SCL) as open-drain output */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure PB7 (SDA) as open-drain output */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Release SDA high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    /* Toggle SCL 9 times to unstick any hung device */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); /* SCL low  */
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); /* SCL high */
        HAL_Delay(1);
    }

    /* Generate STOP condition: SDA low → SCL high → SDA high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); /* SDA low  */
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); /* SCL high */
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); /* SDA high */
    HAL_Delay(1);

    /* Reconfigure PA15 as I2C1_SCL alternate function (AF4) */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Reconfigure PB7 as I2C1_SDA alternate function (AF4) */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief I2C1 initialization for INA226 sensors – graceful error handling.
 *
 * Pins: PA15 = I2C1_SCL (AF4), PB7 = I2C1_SDA (AF4)
 * Changed from PB8/PB9 (DFU boot conflict) and PA14 (SWCLK conflict).
 * On failure, sets ina226_ok = false and returns – does NOT hang.
 */
static void MX_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* Configure PA15 = I2C1_SCL */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure PB7 = I2C1_SDA */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10909CEC; /* 100kHz @ 170MHz PCLK1 (validated) */
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        /* Graceful failure – set flag and return, do NOT hang */
        ina226_ok = false;
        return;
    }

    /* Configure analog filter */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) !=
        HAL_OK) {
        ina226_ok = false;
        return;
    }

    /* Configure digital filter */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        ina226_ok = false;
        return;
    }
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

    float vdda =
        measured_vdda; /* Use calibrated VDDA instead of hardcoded 3.3V */
    float v = (float)raw * (vdda / 4095.0f);
    if (v <= 0.01f || v >= (vdda - 0.01f)) return -99.0f;

    float r_ntc = THERM_SERIES_R * ((vdda / v) - 1.0f);
    if (r_ntc <= 0.0f) return -99.0f;

    float s = logf(r_ntc / THERM_NOMINAL_R) / THERM_BETA;
    s += 1.0f / (THERM_NOMINAL_T + 273.15f);
    return (1.0f / s) - 273.15f + THERM_OFFSET_C;
}

/**
 * @brief Read capacitor bank voltage via AMC1311B isolated amplifier.
 *
 * AMC1311B OUTP (Pin 7) → PA3 → ADC1_IN4  (VOLT_AIN0)
 * AMC1311B OUTN (Pin 6) → PA4 → ADC2_IN17 (VOLT_AIN1)
 *
 * BUG FIX: PA4 is ADC2_IN17, NOT ADC1_IN5. The old code used
 * adcReadChannel(ADC_CHANNEL_5) which reads PB14 on ADC1, not PA4.
 * Now uses adcReadChannel2(ADC_CHANNEL_17) to correctly read PA4 via ADC2.
 *
 * VDDA calibration: Uses measured_vdda from VREFINT instead of hardcoded 3.3V.
 *
 * Verification notes:
 * - With VIN ≈ 0V: expect OUTP ≈ OUTN ≈ 1.44V, so vP - vN ≈ 0V
 * - At steady state: vcap should match INA226 vpack reading
 */
static float readCapVoltage(void) {
    uint32_t sumP = 0, sumN = 0;
    const int samples = 16;

    for (int i = 0; i < samples; i++) {
        uint32_t p = adcReadChannel(ADC_CHANNEL_4);    // PA3
        uint32_t n = adcReadChannel2(ADC_CHANNEL_17);  // PA4

        if (p == 0xFFFF || n == 0xFFFF) return 0.0f;

        sumP += p;
        sumN += n;
    }

    float vdda = measured_vdda;

    float vP = ((float)sumP / samples) * (vdda / 4095.0f);
    float vN = ((float)sumN / samples) * (vdda / 4095.0f);

    float v = (vP - vN) * V_CAP_DIVIDER;

    /* Clamp negative noise to zero (AMC1311B outputs ~0 differential at idle
     * with no return path – this is correct hardware behavior, not a bug).
     * Upper clamp removed: allow readings above 9.5V for accurate delta_v
     * capture during high-charge welds. */
    if (v < 0.0f) v = 0.0f;

    return v;
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
    char response[256];

    {
        char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0') return;
    }

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

    if (strncmp(line, "SET_TRIGGER_MODE,", 17) == 0) {
        int v = atoi(line + 17);
        if (v < 1) v = 1;
        if (v > 2) v = 2;
        trigger_mode = (uint8_t)v;

        snprintf(response, sizeof(response), "ACK,SET_TRIGGER_MODE,mode=%d",
                 trigger_mode);
        uartSend(response);
        return;
    }

    const char* cwp_prefix = "SET_CONTACT_WITH_PEDAL,";
    if (strncmp(line, cwp_prefix, strlen(cwp_prefix)) == 0) {
        int v = atoi(line + strlen(cwp_prefix));
        contact_with_pedal = (v == 0) ? 0 : 1;
        snprintf(response, sizeof(response), "ACK,SET_CONTACT_WITH_PEDAL,%d",
                 contact_with_pedal);
        uartSend(response);
        return;
    }

    if (strncmp(line, "CONTACT_THRESH=", 15) == 0) {
        float v = strtof(line + 15, NULL);
        if (v < 0.1f) v = 0.1f;
        if (v > 9.0f) v = 9.0f;
        g_contact_threshold_v = v;
        g_contact_state = false;
        g_contact_pending_high = false;
        g_contact_pending_since_ms = 0;
        snprintf(response, sizeof(response), "OK,CONTACT_THRESH=%.2f",
                 g_contact_threshold_v);
        uartSend(response);
        return;
    }

    if (strncmp(line, "SET_CONTACT_THRESH,", 19) == 0) {
        float v = strtof(line + 19, NULL);
        if (v < 0.1f) v = 0.1f;
        if (v > 9.0f) v = 9.0f;
        g_contact_threshold_v = v;
        g_contact_state = false;
        g_contact_pending_high = false;
        g_contact_pending_since_ms = 0;
        snprintf(response, sizeof(response), "ACK,SET_CONTACT_THRESH,%.2f",
                 g_contact_threshold_v);
        uartSend(response);
        return;
    }

    if (strncmp(line, "SET_CONTACT_HOLD,", 17) == 0) {
        int v = atoi(line + 17);
        if (v < 1) v = 1;
        if (v > 10) v = 10;
        contact_hold_steps = (uint8_t)v;

        snprintf(response, sizeof(response), "ACK,SET_CONTACT_HOLD,steps=%d",
                 contact_hold_steps);
        uartSend(response);
        return;
    }

    /* UI save flakiness root cause: some clients send lead_r_mohm while
     * firmware only accepted LEAD_R in ohms. Accept both ohm and mΩ command
     * variants. */
    if (strncmp(line, "LEAD_R,", 7) == 0 ||
        strncmp(line, "SET_LEAD_R,", 11) == 0 ||
        strncmp(line, "LEAD_R_MOHM,", 12) == 0 ||
        strncmp(line, "SET_LEAD_R_MOHM,", 16) == 0 ||
        strncmp(line, "lead_r_mohm,", 12) == 0 ||
        strncmp(line, "set_lead_r_mohm,", 16) == 0) {
        bool value_is_mohm = false;
        const char* value_str = NULL;

        if (strncmp(line, "LEAD_R,", 7) == 0) {
            value_str = line + 7;
        } else if (strncmp(line, "SET_LEAD_R,", 11) == 0) {
            value_str = line + 11;
        } else if (strncmp(line, "LEAD_R_MOHM,", 12) == 0) {
            value_is_mohm = true;
            value_str = line + 12;
        } else if (strncmp(line, "SET_LEAD_R_MOHM,", 16) == 0) {
            value_is_mohm = true;
            value_str = line + 16;
        } else if (strncmp(line, "lead_r_mohm,", 12) == 0) {
            value_is_mohm = true;
            value_str = line + 12;
        } else {
            value_is_mohm = true;
            value_str = line + 16;
        }

        char* endptr = NULL;
        float v = strtof(value_str, &endptr);

        if (endptr == value_str || (endptr && *endptr != '\0') ||
            !isfinite(v)) {
            uartSend("ERR,LEAD_R_PARSE");
            return;
        }

        if (value_is_mohm) {
            v *= 0.001f;
        }

        if (v < LEAD_RESISTANCE_MIN_OHMS || v > LEAD_RESISTANCE_MAX_OHMS) {
            snprintf(response, sizeof(response),
                     "ERR,LEAD_R_RANGE,min=%.6f,max=%.6f",
                     LEAD_RESISTANCE_MIN_OHMS, LEAD_RESISTANCE_MAX_OHMS);
            uartSend(response);
            return;
        }

        lead_resistance_ohms = v;
        snprintf(response, sizeof(response), "ACK,LEAD_R,ohm=%.6f,mohm=%.3f",
                 lead_resistance_ohms, lead_resistance_ohms * 1000.0f);
        uartSend(response);
        return;
    }

    if (strcmp(line, "GET_LEAD_R") == 0 || strcmp(line, "LEAD_R?") == 0 ||
        strcmp(line, "GET_LEAD_R_MOHM") == 0 ||
        strcmp(line, "lead_r_mohm?") == 0) {
        snprintf(response, sizeof(response), "LEAD_R,ohm=%.6f,mohm=%.3f",
                 lead_resistance_ohms, lead_resistance_ohms * 1000.0f);
        uartSend(response);
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
        /* Send both STATUS and STATUS2 packets on demand */
        sendStatusPacket();
        return;
    }
    if (strcmp(line, "DBG_SHUNT") == 0) {
        uint32_t p = adcReadFast(ADC_CHANNEL_2);
        uint32_t n = adcReadFast(ADC_CHANNEL_3);

        char buf[128];
        snprintf(buf, sizeof(buf), "DBG,SHUNT,p=%lu,n=%lu,diff=%ld,vdda=%.3f",
                 (unsigned long)p, (unsigned long)n,
                 (long)((int32_t)p - (int32_t)n), measured_vdda);

        uartSend(buf);
        return;
    }

    snprintf(response, sizeof(response), "ERR,UNKNOWN_CMD,rx=%s", line);
    uartSend(response);
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
                if (trigger_mode == 1) {
                    fireRecipe();
                }
            }
        }
    }
}

static void pollContactTrigger(void) {
    if (trigger_mode != 2) {
        contact_hold_active = false;
        contact_hold_start_ms = 0;
        contact_fire_lock = false;
        return;
    }

    bool contact_now = contactDetected();
    uint32_t now = HAL_GetTick();
    uint32_t hold_ms_required = (uint32_t)contact_hold_steps * 500U;

    if (!contact_now) {
        contact_hold_active = false;
        contact_hold_start_ms = 0;
        contact_fire_lock = false;
        return;
    }

    if (contact_fire_lock) {
        return;
    }

    if (!contact_hold_active) {
        contact_hold_active = true;
        contact_hold_start_ms = now;
        return;
    }

    if ((now - contact_hold_start_ms) >= hold_ms_required) {
        uartSend("EVENT,CONTACT_TRIGGER");
        fireRecipe();
        contact_fire_lock = true;
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    SystemCoreClockUpdate();
    HAL_InitTick(TICK_INT_PRIORITY);

    /* Enable DWT cycle counter for microsecond timestamps used by waveform
     * capture. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    MX_GPIO_Init();

    /* 100ms delay for INA226 power rail stabilization */
    HAL_Delay(100);

    for (int i = 0; i < 3; i++) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(80);
    }

    /* I2C bus recovery before initialization – unstick any hung devices */
    i2c_bus_recovery();

    MX_I2C1_Init(); /* I2C1 for INA226 sensors – graceful, no hang */
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    OPAMP1->CSR &= ~OPAMP_CSR_OPAMPxEN;
    MX_ADC2_Init(); /* ADC2 for PA4 (AMC1311B OUTN = ADC2_IN17) */
    MX_TIM1_Init();

    /* Initial VDDA calibration using VREFINT */
    measured_vdda = measureVDDA();

    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    pwmOff();

    boot_ms = HAL_GetTick();

    /* Always boot in PEDAL trigger mode (MODE=1). Runtime switching is allowed,
     * but mode is intentionally non-persistent and resets to PEDAL after
     * reboot. */
    trigger_mode = 1;

    g_contact_threshold_v = CONTACT_THRESHOLD_V_DEFAULT;
    g_contact_state = false;
    g_contact_pending_high = false;
    g_contact_pending_since_ms = 0;
    g_contact_last_sample_ms = 0;
    g_contact_last_vcap = 0.0f;
    GPIO_PinState r = HAL_GPIO_ReadPin(PEDAL_PORT, PEDAL_PIN);
    pedal_last_raw = r;
    pedal_stable = r;
    pedal_last_change_ms = boot_ms;

    /* NEW: Initialize INA226 sensors */
    {
        /* INA226 0x40 (pack): reset + configure */
        ina226_write_reg(INA226_ADDR_PACK, INA226_REG_CONFIG, 0x8000);
        HAL_Delay(20);
        /* CONFIG: AVG=16, VBUSCT=1.1ms, VSHCT=1.1ms, MODE=cont shunt+bus */
        uint16_t ina_config = (0x02 << 9) | (0x04 << 6) | (0x04 << 3) | 0x07;
        ina226_write_reg(INA226_ADDR_PACK, INA226_REG_CONFIG, ina_config);
        HAL_Delay(10);
        /* Calibration register (same as ESP32 used) */
        ina226_write_reg(INA226_ADDR_PACK, INA226_REG_CALIBRATION, 41967);
        HAL_Delay(10);

        /* INA226 0x41 (node1): reset + configure (bus voltage only) */
        ina226_write_reg(INA226_ADDR_NODE1, INA226_REG_CONFIG, 0x8000);
        HAL_Delay(20);
        ina226_write_reg(INA226_ADDR_NODE1, INA226_REG_CONFIG, ina_config);
        HAL_Delay(10);

        /* INA226 0x44 (node2): reset + configure (bus voltage only) */
        ina226_write_reg(INA226_ADDR_NODE2, INA226_REG_CONFIG, 0x8000);
        HAL_Delay(20);
        ina226_write_reg(INA226_ADDR_NODE2, INA226_REG_CONFIG, ina_config);
        HAL_Delay(10);

        /* Initial read to populate values */
        ina226_read_all();
    }

    uartSend("BOOT,STM32G474_WELD_BRAIN_10KHZ_READY");

    {
        char boot_mode_msg[32];
        snprintf(boot_mode_msg, sizeof(boot_mode_msg), "BOOT,MODE=%d",
                 (int)trigger_mode);
        uartSend(boot_mode_msg);
    }

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    while (1) {
        pollPedal();
        pollContactTrigger();
        applyArmTimeout();
        applyReadyTimeout();

        {
            static int last_contact_dbg = -1;
            int c = contactDetected() ? 1 : 0;
            if (c != last_contact_dbg) {
                last_contact_dbg = c;
                char cbuf[96];
                snprintf(cbuf, sizeof(cbuf), "DBG,CONTACT=%d,vcap=%.2f,th=%.2f",
                         c, g_contact_last_vcap, g_contact_threshold_v);
                uartSend(cbuf);
            }
        }

        {
            static uint32_t last_temp_ms = 0;
            uint32_t now = HAL_GetTick();
            if ((now - last_temp_ms) >= 1000) {
                last_temp_ms = now;

                /* Refresh VDDA calibration every 1s alongside temperature */
                measured_vdda = measureVDDA();

                float t = readThermistor();
                if (t > -50.0f && t < 200.0f) {
                    temp_filtered_c = (0.1f * t) + (0.9f * temp_filtered_c);
                }
            }
        }

        /* NEW: INA226 read + charger state machine @ ~100ms interval */
        {
            static uint32_t last_ina_ms = 0;
            uint32_t now = HAL_GetTick();
            if ((now - last_ina_ms) >= 100) {
                last_ina_ms = now;
                ina226_read_all();
                chargerStateMachine();
            }
        }

        /* Send STATUS + STATUS2 packets every 500ms */
        {
            static uint32_t last_chgstat_ms = 0;
            uint32_t now = HAL_GetTick();
            if ((now - last_chgstat_ms) >= 500) {
                last_chgstat_ms = now;
                sendStatusPacket();
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

/**
 * @brief Timer Period Elapsed Callback - Forces FETs off
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM1) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
    }
}

/**
 * @brief TIM1 Update Interrupt Handler
 */
void TIM1_UP_TIM16_IRQHandler(void) { HAL_TIM_IRQHandler(&htim1); }