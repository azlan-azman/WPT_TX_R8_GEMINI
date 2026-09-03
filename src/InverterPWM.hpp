#pragma once

#include <stdint.h>
#include <math.h>
#include "xmc_ccu4.h"
#include "xmc_ccu8.h"
#include "xmc_gpio.h"
#include "xmc_scu.h"
#include "PinDefine.h"

class InverterPWM {
private:
    uint32_t currentPeriod = 639; // 100 kHz at 64 MHz (640 total ticks)

public:
    void init() {
        // 1. Configure Pin Multiplexing (P0.6 ALT4, P0.7 ALT5)
        XMC_GPIO_CONFIG_t gpio_p06 = {};
        gpio_p06.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT4;
        XMC_GPIO_Init(P0_6, &gpio_p06);

        XMC_GPIO_CONFIG_t gpio_p07 = {};
        gpio_p07.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5;
        XMC_GPIO_Init(P0_7, &gpio_p07);

        // 2. Enable CCU Clocks
        XMC_CCU4_Init(CCU40, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
        XMC_CCU4_EnableClock(CCU40, 0); // Slice 0 for PWM Leg A
        XMC_CCU4_EnableClock(CCU40, 1); // Slice 1 for Zero-Crossing Capture

        XMC_CCU8_Init(CCU80, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
        XMC_CCU8_EnableClock(CCU80, 1); // Slice 1 for PWM Leg B

        // 3. CCU40 SLICE 0 Config (Leg A - P0.6)
        XMC_CCU4_SLICE_COMPARE_CONFIG_t ccu4_cfg = {};
        ccu4_cfg.timer_mode    = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA;
        ccu4_cfg.monoshot      = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT;
        ccu4_cfg.passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW;
        XMC_CCU4_SLICE_CompareInit(CCU40_CC40, &ccu4_cfg);
        XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC40, currentPeriod);

        XMC_CCU4_SLICE_EVENT_CONFIG_t ccu4_evt0 = {};
        ccu4_evt0.mapped_input = XMC_CCU4_SLICE_INPUT_I;
        ccu4_evt0.edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE;
        ccu4_evt0.level        = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW;
        XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC40, XMC_CCU4_SLICE_EVENT_0, &ccu4_evt0);
        XMC_CCU4_SLICE_StartConfig(CCU40_CC40, XMC_CCU4_SLICE_EVENT_0, XMC_CCU4_SLICE_START_MODE_TIMER_START_CLEAR);

        // 4. CCU80 SLICE 1 Config (Leg B - P0.7)
        XMC_CCU8_SLICE_COMPARE_CONFIG_t ccu8_cfg = {};
        ccu8_cfg.timer_mode         = XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA;
        ccu8_cfg.monoshot           = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT;
        ccu8_cfg.slice_status       = XMC_CCU8_SLICE_STATUS_CHANNEL_1;
        ccu8_cfg.passive_level_out0 = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW;
        ccu8_cfg.asymmetric_pwm     = 1U;
        ccu8_cfg.invert_out0        = 0U;

        XMC_CCU8_SLICE_CompareInit(CCU80_CC81, &ccu8_cfg);
        XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU80_CC81, currentPeriod);

        XMC_CCU8_SLICE_EVENT_CONFIG_t ccu8_evt0 = {};
        ccu8_evt0.mapped_input = XMC_CCU8_SLICE_INPUT_H;
        ccu8_evt0.edge         = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE;
        ccu8_evt0.level        = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW;
        XMC_CCU8_SLICE_ConfigureEvent(CCU80_CC81, XMC_CCU8_SLICE_EVENT_0, &ccu8_evt0);
        XMC_CCU8_SLICE_StartConfig(CCU80_CC81, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);

        XMC_CCU8_SLICE_SetShadowTransferMode(CCU80_CC81, XMC_CCU8_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH);

        // 5. CCU40 SLICE 1 Config (Zero-Crossing Frequency Capture)
        XMC_CCU4_SLICE_CAPTURE_CONFIG_t cap_cfg = {};
        cap_cfg.fifo_enable       = 0U;
        cap_cfg.timer_clear_mode  = XMC_CCU4_SLICE_TIMER_CLEAR_MODE_ALWAYS;
        cap_cfg.prescaler_mode    = XMC_CCU4_SLICE_PRESCALER_MODE_FLOAT;
        cap_cfg.prescaler_initval = 2U;
        XMC_CCU4_SLICE_CaptureInit(CCU40_CC41, &cap_cfg);

        XMC_CCU4_SLICE_EVENT_CONFIG_t cap_evt0 = {};
        cap_evt0.mapped_input = XMC_CCU4_SLICE_INPUT_D;
        cap_evt0.edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE;
        XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC41, XMC_CCU4_SLICE_EVENT_0, &cap_evt0);
        XMC_CCU4_SLICE_Capture0Config(CCU40_CC41, XMC_CCU4_SLICE_EVENT_0);

        // 6. Start Prescalers
        XMC_CCU4_StartPrescaler(CCU40);
        XMC_CCU8_StartPrescaler(CCU80);

        setEffectiveDuty(0.0f); // 0% Power Baseline
    }

    inline void setFrequency(float freqHz) {
        if (freqHz < 20000.0f)  freqHz = 20000.0f;
        if (freqHz > 150000.0f) freqHz = 150000.0f;

        currentPeriod = static_cast<uint32_t>((64000000.0f / freqHz) - 1.0f);

        XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC40, currentPeriod);
        XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU80_CC81, currentPeriod);

        XMC_CCU4_EnableShadowTransfer(CCU40, XMC_CCU4_SHADOW_TRANSFER_SLICE_0);
        XMC_CCU8_EnableShadowTransfer(CCU80, XMC_CCU8_SHADOW_TRANSFER_SLICE_1);
    }

    inline void setEffectiveDuty(float dutyVal) {
        float dutyPercent = dutyVal / 100.0f;
        if (dutyPercent < 0.0f)  dutyPercent = 0.0f;
        if (dutyPercent > 50.0f) dutyPercent = 50.0f;

        uint16_t totalPeriod = currentPeriod + 1;
        uint16_t halfPeriod  = totalPeriod / 2;

        XMC_CCU4_SLICE_SetTimerCompareMatch(CCU40_CC40, halfPeriod);

        uint16_t shiftTicks = static_cast<uint16_t>(lroundf((dutyPercent / 50.0f) * static_cast<float>(halfPeriod)));
        if (shiftTicks > halfPeriod) {
            shiftTicks = halfPeriod;
        }

        uint16_t cr1 = halfPeriod - shiftTicks;
        uint16_t cr2 = cr1 + halfPeriod - 1;

        XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC81, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, cr1);
        XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC81, XMC_CCU8_SLICE_COMPARE_CHANNEL_2, cr2);

        XMC_CCU4_EnableShadowTransfer(CCU40, XMC_CCU4_SHADOW_TRANSFER_SLICE_0);
        XMC_CCU8_EnableShadowTransfer(CCU80, XMC_CCU8_SHADOW_TRANSFER_SLICE_1);
    }

    // Single-shot Excitation Pulse Ping (20 kHz)
    inline void startPulse() {
        stop();

        setFrequency(20000.0f);
        setEffectiveDuty(5000.0f); // 50% shift

        XMC_CCU4_SLICE_SetTimerRepeatMode(CCU40_CC40, XMC_CCU4_SLICE_TIMER_REPEAT_MODE_SINGLE);
        XMC_CCU8_SLICE_SetTimerRepeatMode(CCU80_CC81, XMC_CCU8_SLICE_TIMER_REPEAT_MODE_SINGLE);

        XMC_GPIO_SetOutputLow(PWM_EN_PORT, PWM_EN_PIN); // Enable Gate Driver

        XMC_CCU4_SLICE_ClearTimer(CCU40_CC40);
        XMC_CCU8_SLICE_ClearTimer(CCU80_CC81);
        XMC_CCU4_SLICE_ClearTimer(CCU40_CC41);

        XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);
        XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);

        XMC_CCU4_SLICE_StartTimer(CCU40_CC41);

        // Allow 50 us pulse to excite LC circuit, then disable driver so tank rings freely
        XMC_DelayUs(50);
        XMC_GPIO_SetOutputHigh(PWM_EN_PORT, PWM_EN_PIN); // Disable Gate Driver
    }

    inline void restoreRepeatMode() {
        XMC_CCU4_SLICE_SetTimerRepeatMode(CCU40_CC40, XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT);
        XMC_CCU8_SLICE_SetTimerRepeatMode(CCU80_CC81, XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT);
    }

    inline uint32_t getCapturedPeriodNs() {
        uint32_t regVal = XMC_CCU4_SLICE_GetCaptureRegisterValue(CCU40_CC41, 0);
        if (regVal == 0) {
            regVal = XMC_CCU4_SLICE_GetCaptureRegisterValue(CCU40_CC41, 1);
        }
        return (regVal * 62U); // ~62.5ns per tick at 64 MHz with prescaler = 4
    }

    inline void start() {
        restoreRepeatMode();
        XMC_GPIO_SetOutputLow(PWM_EN_PORT, PWM_EN_PIN); // Enable Gate Driver

        XMC_CCU4_SLICE_ClearTimer(CCU40_CC40);
        XMC_CCU8_SLICE_ClearTimer(CCU80_CC81);

        XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);
        XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);
    }

    inline void stop() {
        XMC_GPIO_SetOutputHigh(PWM_EN_PORT, PWM_EN_PIN); // Disable Gate Driver
        XMC_CCU4_SLICE_StopTimer(CCU40_CC40);
        XMC_CCU8_SLICE_StopTimer(CCU80_CC81);
        XMC_CCU4_SLICE_StopTimer(CCU40_CC41);
        XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);
    }
};
