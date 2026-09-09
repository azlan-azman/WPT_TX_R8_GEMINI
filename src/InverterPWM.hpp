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

    inline uint32_t decodeTicks(uint32_t cvReg, bool isIncrement) {
        if (cvReg == 0) return 0;

        uint32_t rawVal = cvReg & 0xFFFFU;
        if (isIncrement) rawVal += 1U;

        uint32_t fpcv = (cvReg >> 16U) & 0xFU;
        uint32_t psc = 2U; // prescaler_initval = 2
        uint32_t timer_val = 0;

        if (fpcv > psc) {
            uint32_t diff = fpcv - psc;
            for (uint32_t i = diff; i > 0; i--) {
                timer_val = (timer_val << 1U) + 0xFFFFU;
            }
            timer_val += (rawVal << diff);
        } else {
            timer_val = rawVal;
        }
        return timer_val;
    }

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
        ccu4_cfg.timer_mode        = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA;
        ccu4_cfg.monoshot          = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT;
        ccu4_cfg.passive_level     = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW;
        ccu4_cfg.prescaler_mode    = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL;
        ccu4_cfg.prescaler_initval = 0U;
        XMC_CCU4_SLICE_CompareInit(CCU40_CC40, &ccu4_cfg);
        XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC40, currentPeriod);

        XMC_CCU4_SLICE_EVENT_CONFIG_t ccu4_evt0 = {};
        ccu4_evt0.mapped_input = XMC_CCU4_SLICE_INPUT_I;
        ccu4_evt0.edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE;
        ccu4_evt0.level        = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW;
        XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC40, XMC_CCU4_SLICE_EVENT_0, &ccu4_evt0);
        XMC_CCU4_SLICE_StartConfig(CCU40_CC40, XMC_CCU4_SLICE_EVENT_0, XMC_CCU4_SLICE_START_MODE_TIMER_START);

        // Bind Period Match event to SR2 to trigger VADC Iout scan on REQ_TR_A
        XMC_CCU4_SLICE_SetInterruptNode(CCU40_CC40, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU4_SLICE_SR_ID_2);
        XMC_CCU4_SLICE_EnableEvent(CCU40_CC40, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);

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
        XMC_CCU8_SLICE_StartConfig(CCU80_CC81, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);

        XMC_CCU8_SLICE_SetShadowTransferMode(CCU80_CC81, XMC_CCU8_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH);

        // 5. CCU40 SLICE 1 Config (Zero-Crossing Frequency Capture)
        XMC_CCU4_SLICE_CAPTURE_CONFIG_t cap_cfg = {};
        cap_cfg.fifo_enable       = 0U;
        cap_cfg.timer_clear_mode  = XMC_CCU4_SLICE_TIMER_CLEAR_MODE_ALWAYS;
        cap_cfg.prescaler_mode    = XMC_CCU4_SLICE_PRESCALER_MODE_FLOAT;
        cap_cfg.prescaler_initval = 2U;
        cap_cfg.float_limit       = 15U;
        XMC_CCU4_SLICE_CaptureInit(CCU40_CC41, &cap_cfg);

        XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC41, 0xFFFFU);

        // Event 0 (Rising Edge) -> Low Register Set (CV0/CV1)
        XMC_CCU4_SLICE_EVENT_CONFIG_t cap_evt0 = {};
        cap_evt0.mapped_input = XMC_CCU4_SLICE_INPUT_D;
        cap_evt0.edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE;
        XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC41, XMC_CCU4_SLICE_EVENT_0, &cap_evt0);
        XMC_CCU4_SLICE_Capture0Config(CCU40_CC41, XMC_CCU4_SLICE_EVENT_0);

        // Event 1 (Falling Edge) -> High Register Set (CV2/CV3)
        XMC_CCU4_SLICE_EVENT_CONFIG_t cap_evt1 = {};
        cap_evt1.mapped_input = XMC_CCU4_SLICE_INPUT_D;
        cap_evt1.edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE;
        XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC41, XMC_CCU4_SLICE_EVENT_1, &cap_evt1);
        XMC_CCU4_SLICE_Capture1Config(CCU40_CC41, XMC_CCU4_SLICE_EVENT_1);

        XMC_CCU4_EnableShadowTransfer(CCU40, XMC_CCU4_SHADOW_TRANSFER_SLICE_0 |
                                             XMC_CCU4_SHADOW_TRANSFER_SLICE_1 |
                                             XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1);

        // 6. Start Prescalers
        XMC_CCU4_StartPrescaler(CCU40);
        XMC_CCU8_StartPrescaler(CCU80);

        setEffectiveDuty(1500.0f); // Default 15.00% duty cycle (1500 basis points)
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

    // Symmetrical 180-Degree Anti-Phase Duty Modulation
    inline void setEffectiveDuty(float dutyVal) {
        uint32_t dutyBP = (dutyVal <= 50.0f) ? static_cast<uint32_t>(dutyVal * 100.0f) : static_cast<uint32_t>(dutyVal);
        if (dutyBP > 5000U) dutyBP = 5000U; // 50.00% max per leg

        uint32_t totalTicks = currentPeriod + 1U; // 640 total ticks
        uint32_t halfPeriod = totalTicks / 2U;    // 320 ticks (180 degrees)

        uint32_t legTicks = (dutyBP * totalTicks) / 10000U; // 96 ticks for 15% duty
        if (legTicks > halfPeriod) legTicks = halfPeriod;

        if (dutyBP == 0U || legTicks == 0U) {
            XMC_CCU4_SLICE_SetTimerCompareMatch(CCU40_CC40, 0U);
            XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC81, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, static_cast<uint16_t>(halfPeriod));
            XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC81, XMC_CCU8_SLICE_COMPARE_CHANNEL_2, static_cast<uint16_t>(halfPeriod));
        } else {
            // Leg A (P0.6): Active HIGH from tick 0 to legTicks (Ticks 0 -> 96)
            XMC_CCU4_SLICE_SetTimerCompareMatch(CCU40_CC40, static_cast<uint16_t>(legTicks));

            // Leg B (P0.7): Active HIGH from 180 deg (halfPeriod) to (180 deg + legTicks)
            // Active HIGH from 50% -> 65% (Ticks 320 -> 416)
            // Exactly 35% freewheeling space (224 ticks) on both sides of every pulse!
            uint16_t cr1 = static_cast<uint16_t>(halfPeriod);
            uint16_t cr2 = static_cast<uint16_t>(halfPeriod + legTicks - 1U);

            XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC81, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, cr1);
            XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC81, XMC_CCU8_SLICE_COMPARE_CHANNEL_2, cr2);
        }

        XMC_CCU4_EnableShadowTransfer(CCU40, XMC_CCU4_SHADOW_TRANSFER_SLICE_0);
        XMC_CCU8_EnableShadowTransfer(CCU80, XMC_CCU8_SHADOW_TRANSFER_SLICE_1);
    }

    inline void startPulse() {
        stop();

        setFrequency(20000.0f);
        setEffectiveDuty(5000.0f); // 50% shift

        XMC_CCU4_SLICE_SetTimerRepeatMode(CCU40_CC40, XMC_CCU4_SLICE_TIMER_REPEAT_MODE_SINGLE);
        XMC_CCU8_SLICE_SetTimerRepeatMode(CCU80_CC81, XMC_CCU8_SLICE_TIMER_REPEAT_MODE_SINGLE);

        XMC_DelayUs(50);

        (void)CCU40_CC41->CV[0];
        (void)CCU40_CC41->CV[1];
        (void)CCU40_CC41->CV[2];
        (void)CCU40_CC41->CV[3];

        XMC_CCU4_SLICE_ClearTimer(CCU40_CC40);
        XMC_CCU8_SLICE_ClearTimer(CCU80_CC81);
        XMC_CCU4_SLICE_ClearTimer(CCU40_CC41);

        XMC_CCU4_SLICE_StartTimer(CCU40_CC41);
        XMC_CCU4_SLICE_StartTimer(CCU40_CC40);
        XMC_CCU8_SLICE_StartTimer(CCU80_CC81);

        XMC_GPIO_SetOutputLow(PWM_EN_PORT, PWM_EN_PIN);

        XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);
        XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);

        XMC_DelayUs(1000);
    }

    inline void restoreRepeatMode() {
        XMC_CCU8_SLICE_SetTimerRepeatMode(CCU80_CC81, XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT);
        XMC_CCU4_SLICE_SetTimerRepeatMode(CCU40_CC41, XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT);
    }

    inline uint32_t getCapturedPeriodNs() {
        uint32_t capLow = CCU40_CC41->CV[1];
        if ((capLow & 0xFFFFU) == 0) {
            capLow = CCU40_CC41->CV[0];
        }

        uint32_t capHigh = CCU40_CC41->CV[3];
        if ((capHigh & 0xFFFFU) == 0) {
            capHigh = CCU40_CC41->CV[2];
        }

        uint32_t ticksLow   = decodeTicks(capLow, true);
        uint32_t ticksHigh  = decodeTicks(capHigh, false);
        uint32_t totalTicks = ticksLow + ticksHigh;

        if (totalTicks == 0) {
            totalTicks = ticksLow ? ticksLow : ticksHigh;
        }

        if (totalTicks == 0) return 0;

        return static_cast<uint32_t>(static_cast<float>(totalTicks) * 62.5f);
    }

    inline void start() {
        XMC_CCU4_SLICE_StopTimer(CCU40_CC41); // Stop capture slice

        restoreRepeatMode();

        XMC_CCU4_SLICE_ClearTimer(CCU40_CC40);
        XMC_CCU8_SLICE_ClearTimer(CCU80_CC81);

        XMC_CCU4_SLICE_StartTimer(CCU40_CC40);
        XMC_CCU8_SLICE_StartTimer(CCU80_CC81);

        XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);
        XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);

        XMC_GPIO_SetOutputLow(PWM_EN_PORT, PWM_EN_PIN); // Enable Gate Driver
    }

    inline void stop() {
        XMC_GPIO_SetOutputHigh(PWM_EN_PORT, PWM_EN_PIN); // Disable Gate Driver
        XMC_CCU4_SLICE_StopTimer(CCU40_CC40);
        XMC_CCU8_SLICE_StopTimer(CCU80_CC81);
        XMC_CCU4_SLICE_StopTimer(CCU40_CC41);
        XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);
    }
};