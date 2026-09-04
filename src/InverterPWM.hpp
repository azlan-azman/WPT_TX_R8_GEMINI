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

    // Decodes DAVE floating prescaler ticks
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

        // =====================================================================
        // 5. CCU40 SLICE 1 Config (Zero-Crossing Frequency Capture)
        // =====================================================================
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
        cap_evt0.mapped_input = XMC_CCU4_SLICE_INPUT_A;                     // CONNECTED TO ERU0_IOUT0
        cap_evt0.edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE;
        XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC41, XMC_CCU4_SLICE_EVENT_0, &cap_evt0);
        XMC_CCU4_SLICE_Capture0Config(CCU40_CC41, XMC_CCU4_SLICE_EVENT_0);

        // Event 1 (Falling Edge) -> High Register Set (CV2/CV3)
        XMC_CCU4_SLICE_EVENT_CONFIG_t cap_evt1 = {};
        cap_evt1.mapped_input = XMC_CCU4_SLICE_INPUT_A;                     // CONNECTED TO ERU0_IOUT0
        cap_evt1.edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE;
        XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC41, XMC_CCU4_SLICE_EVENT_1, &cap_evt1);
        XMC_CCU4_SLICE_Capture1Config(CCU40_CC41, XMC_CCU4_SLICE_EVENT_1);

        XMC_CCU4_EnableShadowTransfer(CCU40, XMC_CCU4_SHADOW_TRANSFER_SLICE_1 |
                                           XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1);

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

        XMC_DelayUs(50);

        // Clear CV registers to reset flags before ping
        (void)CCU40_CC41->CV[0];
        (void)CCU40_CC41->CV[1];
        (void)CCU40_CC41->CV[2];
        (void)CCU40_CC41->CV[3];

        XMC_CCU4_SLICE_ClearTimer(CCU40_CC40);
        XMC_CCU8_SLICE_ClearTimer(CCU80_CC81);
        XMC_CCU4_SLICE_ClearTimer(CCU40_CC41);

        // Start CCU4 capture timer FIRST
        XMC_CCU4_SLICE_StartTimer(CCU40_CC41);

        // Start PWM timers
        XMC_CCU4_SLICE_StartTimer(CCU40_CC40);
        XMC_CCU8_SLICE_StartTimer(CCU80_CC81);

        // Trigger PWM hardware timers via SCU CCU trigger
        XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);
        XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU40 | XMC_SCU_CCU_TRIGGER_CCU80);

        // Wait 1ms for the tank to ring and generate ACMP1 zero-crossings
        XMC_DelayUs(1000);
    }

    inline void restoreRepeatMode() {
        XMC_CCU4_SLICE_SetTimerRepeatMode(CCU40_CC40, XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT);
        XMC_CCU8_SLICE_SetTimerRepeatMode(CCU80_CC81, XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT);
    }

    inline uint32_t getCapturedPeriodNs() {
        uint32_t capLow = 0;
        uint32_t capHigh = 0;

        // Get Latest Low Register Set (CV0 / CV1)
        if (CCU40_CC41->CV[1] & CCU4_CC4_CV_FFL_Msk) {
            capLow = CCU40_CC41->CV[1];
        } else if (CCU40_CC41->CV[0] & CCU4_CC4_CV_FFL_Msk) {
            capLow = CCU40_CC41->CV[0];
        }

        // Get Latest High Register Set (CV2 / CV3)
        if (CCU40_CC41->CV[3] & CCU4_CC4_CV_FFL_Msk) {
            capHigh = CCU40_CC41->CV[3];
        } else if (CCU40_CC41->CV[2] & CCU4_CC4_CV_FFL_Msk) {
            capHigh = CCU40_CC41->CV[2];
        }

        // Return single edge ticks if only one register set captured, or full wave sum
        uint32_t ticksLow  = decodeTicks(capLow, true);
        uint32_t ticksHigh = decodeTicks(capHigh, false);
        uint32_t totalTicks = ticksLow + ticksHigh;

        if (totalTicks == 0) return 0;

        // Base clock (64MHz) / prescaler_initval (2^2 = 4) -> 16 MHz -> 1 tick = 62.5 ns
        return static_cast<uint32_t>(static_cast<float>(totalTicks) * 62.5f);
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