#pragma once

#include "xmc_ccu4.h"
#include "xmc_ccu8.h"

class FrequencyCapture {
public:
    void init() {
        // Initialize CCU80 Slice 0 for P0.12 Capture
        XMC_CCU8_SLICE_CAPTURE_CONFIG_t cap1_cfg = {};
        cap1_cfg.timer_clear_mode = XMC_CCU8_SLICE_TIMER_CLEAR_MODE_ALWAYS;
        XMC_CCU8_SLICE_CaptureInit(CCU80_CC80, &cap1_cfg);
        XMC_CCU8_SLICE_StartTimer(CCU80_CC80);

        // Initialize CCU80 Slice 3 for P0.13 Capture
        XMC_CCU8_SLICE_CAPTURE_CONFIG_t cap2_cfg = {};
        cap2_cfg.timer_clear_mode = XMC_CCU8_SLICE_TIMER_CLEAR_MODE_ALWAYS;
        XMC_CCU8_SLICE_CaptureInit(CCU80_CC83, &cap2_cfg);
        XMC_CCU8_SLICE_StartTimer(CCU80_CC83);
    }
};