#pragma once

#include <stdint.h>
#include "XMC1300.h"
#include "xmc_ccu8.h"
#include "xmc_gpio.h"
#include "xmc_scu.h"
#include "PinDefine.h"

extern uint32_t SystemCoreClock;

class FanMonitor {
public:
    float    fanSpeed1_RPM;
    float    fanSpeed2_RPM;
    uint32_t fanCount1;
    uint32_t fanCount2;
    bool     fanOK;

    FanMonitor()
        : fanSpeed1_RPM(0.0f),
          fanSpeed2_RPM(0.0f),
          fanCount1(0),
          fanCount2(0),
          fanOK(true) {}

    void init() {
        XMC_CCU8_Init(CCU80, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
        XMC_CCU8_StartPrescaler(CCU80);

        XMC_GPIO_CONFIG_t pinConfig = {};
        pinConfig.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
        pinConfig.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD;

        XMC_GPIO_Init(XMC_GPIO_PORT0, 12U, &pinConfig); // CAPTURE_1 Input
        XMC_GPIO_Init(XMC_GPIO_PORT0, 13U, &pinConfig); // CAPTURE_2 Input

        initSliceCapture(CCU80_CC80, 0U, XMC_CCU8_SLICE_INPUT_A); // Slice 0
        initSliceCapture(CCU80_CC83, 3U, XMC_CCU8_SLICE_INPUT_B); // Slice 3
    }

    bool update() {
        uint32_t periodNs1 = 0;
        uint32_t periodNs2 = 0;

        if (getSlicePeriodNs(CCU80_CC80, &periodNs1) && periodNs1 > 0) {
            fanSpeed1_RPM = 30000000000.0f / static_cast<float>(periodNs1);
            fanCount1 = 0;
        } else {
            fanCount1++;
            if (fanCount1 > 50U) fanSpeed1_RPM = 0.0f;
        }

        if (getSlicePeriodNs(CCU80_CC83, &periodNs2) && periodNs2 > 0) {
            fanSpeed2_RPM = 30000000000.0f / static_cast<float>(periodNs2);
            fanCount2 = 0;
        } else {
            fanCount2++;
            if (fanCount2 > 50U) fanSpeed2_RPM = 0.0f;
        }

        fanOK = (fanSpeed1_RPM > 0.0f) && (fanSpeed2_RPM > 0.0f);
        return fanOK;
    }

private:
    void initSliceCapture(XMC_CCU8_SLICE_t *const slice, uint32_t sliceNum, XMC_CCU8_SLICE_INPUT_t input) {
        XMC_CCU8_SLICE_CAPTURE_CONFIG_t capConfig = {};
        capConfig.fifo_enable       = 0U;
        capConfig.timer_clear_mode  = XMC_CCU8_SLICE_TIMER_CLEAR_MODE_ALWAYS;
        capConfig.prescaler_mode    = XMC_CCU8_SLICE_PRESCALER_MODE_FLOAT;
        capConfig.prescaler_initval = 0U;

        XMC_CCU8_SLICE_CaptureInit(slice, &capConfig);

        // Event 0: Rising Edge -> Capture Set 0 (CV0/CV1)
        XMC_CCU8_SLICE_EVENT_CONFIG_t ev0 = {};
        ev0.mapped_input = input;
        ev0.edge         = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE;
        ev0.level        = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH;
        XMC_CCU8_SLICE_ConfigureEvent(slice, XMC_CCU8_SLICE_EVENT_0, &ev0);
        XMC_CCU8_SLICE_Capture0Config(slice, XMC_CCU8_SLICE_EVENT_0);

        // Event 1: Falling Edge -> Capture Set 1 (CV2/CV3)
        XMC_CCU8_SLICE_EVENT_CONFIG_t ev1 = {};
        ev1.mapped_input = input;
        ev1.edge         = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE;
        ev1.level        = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH;
        XMC_CCU8_SLICE_ConfigureEvent(slice, XMC_CCU8_SLICE_EVENT_1, &ev1);
        XMC_CCU8_SLICE_Capture1Config(slice, XMC_CCU8_SLICE_EVENT_1);

        XMC_CCU8_SLICE_SetTimerPeriodMatch(slice, 0xFFFFU);
        XMC_CCU8_EnableShadowTransfer(CCU80, (XMC_CCU8_SHADOW_TRANSFER_SLICE_0 << (sliceNum * 4U)));

        XMC_CCU8_EnableClock(CCU80, sliceNum);
        XMC_CCU8_SLICE_ClearTimer(slice);
        XMC_CCU8_SLICE_StartTimer(slice);
    }

    bool getSlicePeriodNs(XMC_CCU8_SLICE_t *const slice, uint32_t *const periodNs) {
        uint32_t cvLow  = slice->CV[0]; // Set Low
        uint32_t cvHigh = slice->CV[2]; // Set High

        bool lowFull  = (cvLow & CCU8_CC8_CV_FFL_Msk) != 0;
        bool highFull = (cvHigh & CCU8_CC8_CV_FFL_Msk) != 0;

        if (lowFull || highFull) {
            uint32_t ticksLow  = decodeTicks(cvLow, true);
            uint32_t ticksHigh = decodeTicks(cvHigh, false);
            uint32_t totalTicks = ticksLow + ticksHigh;

            uint32_t clk = SystemCoreClock;
            if (clk > 0 && totalTicks > 0) {
                *periodNs = static_cast<uint32_t>((static_cast<uint64_t>(totalTicks) * 1000000000ULL) / clk);
                return true;
            }
        }
        return false;
    }

    uint32_t decodeTicks(uint32_t cvReg, bool isLowSet) {
        uint32_t rawVal = cvReg & 0xFFFFU;
        if (isLowSet) rawVal += 1U;

        uint32_t fpcv = (cvReg >> 16U) & 0xFU; // Floating Prescaler Count
        uint32_t basePsc = 0U;                 // Prescaler Init Val = 0
        uint32_t pscDiff = (fpcv > basePsc) ? (fpcv - basePsc) : 0U;

        uint32_t ticks = 0U;
        for (uint32_t i = pscDiff; i > 0; --i) {
            ticks = (ticks << 1U) + 65535U;
        }
        ticks += (rawVal << pscDiff);
        return ticks;
    }
};
