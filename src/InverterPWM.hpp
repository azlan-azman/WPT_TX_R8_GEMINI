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
    uint32_t currentPeriod = 319U; // 100 kHz at 64 MHz Center-Aligned: (64 MHz / (2 * 100 kHz)) - 1

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
        // 1. Output Pin Configuration matching pwm_ccu8_conf.c
        // Ch1 Out0 -> P1.2 (ALT5)
        XMC_GPIO_CONFIG_t p1_2_cfg = {};
        p1_2_cfg.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5;
        p1_2_cfg.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD;
        p1_2_cfg.output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW;
        XMC_GPIO_Init((XMC_GPIO_PORT_t *)PORT1_BASE, 2U, &p1_2_cfg);

        // Ch2 Out3 -> P0.4 (ALT5)
        XMC_GPIO_CONFIG_t p0_4_cfg = {};
        p0_4_cfg.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5;
        p0_4_cfg.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD;
        p0_4_cfg.output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW;
        XMC_GPIO_Init((XMC_GPIO_PORT_t *)PORT0_BASE, 4U, &p0_4_cfg);

        // 2. Enable Peripheral Clocks
        XMC_CCU8_Init(CCU80, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
        XMC_CCU8_EnableClock(CCU80, 1U); // Slice 1

        XMC_CCU4_Init(CCU40, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
        XMC_CCU4_EnableClock(CCU40, 1U); // Slice 1 (Zero-Crossing Frequency Capture)

        // =====================================================================
        // 3. CCU80 SLICE 1 Full-Bridge PWM Config (pwm_ccu8_conf.c)
        // =====================================================================
        XMC_CCU8_SLICE_COMPARE_CONFIG_t ccu8_cfg = {};
        ccu8_cfg.timer_mode            = XMC_CCU8_SLICE_TIMER_COUNT_MODE_CA; // Center-Aligned (Dual-Slope)
        ccu8_cfg.monoshot              = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT;
        ccu8_cfg.prescaler_mode        = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL;
        ccu8_cfg.prescaler_initval     = 0U;
        ccu8_cfg.slice_status          = XMC_CCU8_SLICE_STATUS_CHANNEL_1;
        ccu8_cfg.passive_level_out0    = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW;
        ccu8_cfg.passive_level_out1    = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW;
        ccu8_cfg.passive_level_out2    = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW;
        ccu8_cfg.passive_level_out3    = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW;
        ccu8_cfg.asymmetric_pwm        = 0U;

        // Invert Out1 and Out3
        ccu8_cfg.invert_out0           = 0U;
        ccu8_cfg.invert_out1           = 1U;
        ccu8_cfg.invert_out2           = 0U;
        ccu8_cfg.invert_out3           = 1U;

        XMC_CCU8_SLICE_CompareInit(CCU80_CC81, &ccu8_cfg);
        XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU80_CC81, currentPeriod);

        // Hardware Dead-Time Generator (8 rising cycles = 125 ns at 64 MHz)
        XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t dt_cfg = {};
        dt_cfg.enable_dead_time_channel1        = 1U;
        dt_cfg.enable_dead_time_channel2        = 1U;
        dt_cfg.channel1_st_path                 = 1U;
        dt_cfg.channel1_inv_st_path             = 1U;
        dt_cfg.channel2_st_path                 = 1U;
        dt_cfg.channel2_inv_st_path             = 1U;
        dt_cfg.div                              = XMC_CCU8_SLICE_DTC_DIV_1;
        dt_cfg.channel1_st_rising_edge_counter  = 8U;
        dt_cfg.channel1_st_falling_edge_counter = 0U;
        dt_cfg.channel2_st_rising_edge_counter  = 8U;
        dt_cfg.channel2_st_falling_edge_counter = 0U;
        XMC_CCU8_SLICE_DeadTimeInit(CCU80_CC81, &dt_cfg);

        // External Event 0: Hardware Timer Start via SCU Trigger (Input H)
        XMC_CCU8_SLICE_EVENT_CONFIG_t ccu8_evt0 = {};
        ccu8_evt0.mapped_input = XMC_CCU8_SLICE_INPUT_H;
        ccu8_evt0.edge         = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE;
        ccu8_evt0.level        = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW;
        XMC_CCU8_SLICE_ConfigureEvent(CCU80_CC81, XMC_CCU8_SLICE_EVENT_0, &ccu8_evt0);
        XMC_CCU8_SLICE_StartConfig(CCU80_CC81, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START);

        // Synchronous VADC Trigger: Bind Period Match Event to Service Request Line 2 (SR2)
        XMC_CCU8_SLICE_SetInterruptNode(CCU80_CC81, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU8_SLICE_SR_ID_2);
        XMC_CCU8_SLICE_EnableEvent(CCU80_CC81, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);

        XMC_CCU8_SLICE_SetShadowTransferMode(CCU80_CC81, XMC_CCU8_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH);

        // =====================================================================
        // 4. CCU40 SLICE 1 Config (Zero-Crossing Frequency Capture)
        // =====================================================================
        XMC_CCU4_SLICE_CAPTURE_CONFIG_t cap_cfg = {};
        cap_cfg.fifo_enable       = 0U;
        cap_cfg.timer_clear_mode  = XMC_CCU4_SLICE_TIMER_CLEAR_MODE_ALWAYS;
        cap_cfg.prescaler_mode    = XMC_CCU4_SLICE_PRESCALER_MODE_FLOAT;
        cap_cfg.prescaler_initval = 2U;
        cap_cfg.float_limit       = 15U;
        XMC_CCU4_SLICE_CaptureInit(CCU40_CC41, &cap_cfg);
        XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC41, 0xFFFFU);

        // Event 0 (Rising Edge) -> Low Register Set
        XMC_CCU4_SLICE_EVENT_CONFIG_t cap_evt0 = {};
        cap_evt0.mapped_input = XMC_CCU4_SLICE_INPUT_D;
        cap_evt0.edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE;
        XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC41, XMC_CCU4_SLICE_EVENT_0, &cap_evt0);
        XMC_CCU4_SLICE_Capture0Config(CCU40_CC41, XMC_CCU4_SLICE_EVENT_0);

        // Event 1 (Falling Edge) -> High Register Set
        XMC_CCU4_SLICE_EVENT_CONFIG_t cap_evt1 = {};
        cap_evt1.mapped_input = XMC_CCU4_SLICE_INPUT_D;
        cap_evt1.edge         = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE;
        XMC_CCU4_SLICE_ConfigureEvent(CCU40_CC41, XMC_CCU4_SLICE_EVENT_1, &cap_evt1);
        XMC_CCU4_SLICE_Capture1Config(CCU40_CC41, XMC_CCU4_SLICE_EVENT_1);

        XMC_CCU4_EnableShadowTransfer(CCU40, XMC_CCU4_SHADOW_TRANSFER_SLICE_1 |
                                             XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1);

        // 5. Start Prescalers
        XMC_CCU8_StartPrescaler(CCU80);
        XMC_CCU4_StartPrescaler(CCU40);

        // Default 20.00% symmetric duty cycle (2000 basis points) matching DAVE configuration
        setEffectiveDuty(2000.0f);
    }

    inline void setFrequency(float freqHz) {
        if (freqHz < 20000.0f)  freqHz = 20000.0f;
        if (freqHz > 150000.0f) freqHz = 150000.0f;

        // Center-aligned: period = (64 MHz / (2 * freqHz)) - 1
        currentPeriod = static_cast<uint32_t>((32000000.0f / freqHz) - 1.0f);

        XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU80_CC81, currentPeriod);
        XMC_CCU8_EnableShadowTransfer(CCU80, XMC_CCU8_SHADOW_TRANSFER_SLICE_1);
    }

    inline void setEffectiveDuty(float dutyVal) {
        uint32_t dutyBP = (dutyVal <= 50.0f) ? static_cast<uint32_t>(dutyVal * 100.0f) : static_cast<uint32_t>(dutyVal);
        if (dutyBP > 5000U) dutyBP = 5000U; // 50.00% max conduction per leg

        uint32_t halfPeriodTicks = currentPeriod + 1U; // 320 ticks for 100 kHz

        if (dutyBP == 0U) {
            // Completely disable output pulses
            XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC81, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, static_cast<uint16_t>(halfPeriodTicks));
            XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC81, XMC_CCU8_SLICE_COMPARE_CHANNEL_2, 0U);
        } else {
            uint32_t activeTicks = (halfPeriodTicks * dutyBP) / 10000U; // 64 ticks at 20%
            if (activeTicks >= halfPeriodTicks) activeTicks = halfPeriodTicks - 1U;

            // Channel 1 (Out0 -> P1.2): Active HIGH around Period Match (top)
            uint16_t cr1 = static_cast<uint16_t>(halfPeriodTicks - activeTicks); // 320 - 64 = 256

            // Channel 2 (Out3 Inverted -> P0.4): Active HIGH around Zero Match (bottom)
            uint16_t cr2 = static_cast<uint16_t>(activeTicks);                   // 64

            XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC81, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, cr1);
            XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC81, XMC_CCU8_SLICE_COMPARE_CHANNEL_2, cr2);
        }

        XMC_CCU8_EnableShadowTransfer(CCU80, XMC_CCU8_SHADOW_TRANSFER_SLICE_1);
    }

    inline void startPulse() {
        stop();

        setFrequency(20000.0f);
        setEffectiveDuty(5000.0f); // 50% duty pulse

        XMC_CCU8_SLICE_SetTimerRepeatMode(CCU80_CC81, XMC_CCU8_SLICE_TIMER_REPEAT_MODE_SINGLE);

        XMC_DelayUs(50);

        (void)CCU40_CC41->CV[0];
        (void)CCU40_CC41->CV[1];
        (void)CCU40_CC41->CV[2];
        (void)CCU40_CC41->CV[3];

        XMC_CCU8_SLICE_ClearTimer(CCU80_CC81);
        XMC_CCU4_SLICE_ClearTimer(CCU40_CC41);

        XMC_CCU4_SLICE_StartTimer(CCU40_CC41);
        XMC_CCU8_SLICE_StartTimer(CCU80_CC81);

        XMC_GPIO_SetOutputLow(PWM_EN_PORT, PWM_EN_PIN);

        XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU80);
        XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU80);

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
        XMC_CCU4_SLICE_StopTimer(CCU40_CC41);

        restoreRepeatMode();

        XMC_CCU8_SLICE_ClearTimer(CCU80_CC81);
        XMC_CCU8_SLICE_StartTimer(CCU80_CC81);

        XMC_SCU_SetCcuTriggerHigh(XMC_SCU_CCU_TRIGGER_CCU80);
        XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU80);

        XMC_GPIO_SetOutputLow(PWM_EN_PORT, PWM_EN_PIN);
    }

    inline void stop() {
        XMC_GPIO_SetOutputHigh(PWM_EN_PORT, PWM_EN_PIN);
        XMC_CCU8_SLICE_StopTimer(CCU80_CC81);
        XMC_CCU4_SLICE_StopTimer(CCU40_CC41);
        XMC_SCU_SetCcuTriggerLow(XMC_SCU_CCU_TRIGGER_CCU80);
    }
};