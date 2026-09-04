#pragma once

extern "C" {
#include "xmc_eru.h"
}

class HardwareERU {
public:
    void init() {
        // 1. ERU0 ETL Channel 1 Configuration (Reads ACMP1.OUT)
        XMC_ERU_ETL_CONFIG_t etl_cfg = {};
        etl_cfg.input_a = XMC_ERU_ETL_INPUT_A0;                  // ACMP1.OUT on XMC1302
        etl_cfg.input_b = XMC_ERU_ETL_INPUT_B0;
        etl_cfg.enable_output_trigger = 1U;                       // Enable output trigger in ETL
        etl_cfg.status_flag_mode = XMC_ERU_ETL_STATUS_FLAG_MODE_HWCTRL;
        etl_cfg.edge_detection   = XMC_ERU_ETL_EDGE_DETECTION_RISING;
        etl_cfg.output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL0; // Route to OGU Channel 0
        etl_cfg.source = XMC_ERU_ETL_SOURCE_A;                    // Read ACMP1.OUT from Input A0

        XMC_ERU_ETL_Init(XMC_ERU0, 1U, &etl_cfg);

        // Sets EXICON1.PE bit so pulses pass to OGU0
        XMC_ERU_ETL_EnableOutputTrigger(XMC_ERU0, 1U, XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL0);

        // 2. ERU0 OGU Channel 0 Configuration (Outputs on line ERU0_IOUT0)
        XMC_ERU_OGU_CONFIG_t ogu_cfg = {};
        ogu_cfg.peripheral_trigger       = 0U;
        ogu_cfg.enable_pattern_detection = false;
        ogu_cfg.service_request          = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER;
        ogu_cfg.pattern_detection_input  = XMC_ERU_OGU_PATTERN_DETECTION_INPUT1;

        XMC_ERU_OGU_Init(XMC_ERU0, 0U, &ogu_cfg);
    }
};