#pragma once

#include "xmc_eru.h"

class HardwareERU {
public:
    void init() {
        // Configure ERU0 Event Detection (ETL Channel 1)
        XMC_ERU_ETL_CONFIG_t etl_cfg = {};
        etl_cfg.input_a = XMC_ERU_ETL_INPUT_A0;
        etl_cfg.input_b = XMC_ERU_ETL_INPUT_B0;
        etl_cfg.status_flag_mode = XMC_ERU_ETL_STATUS_FLAG_MODE_HWCTRL;
        etl_cfg.edge_detection   = XMC_ERU_ETL_EDGE_DETECTION_RISING;
        etl_cfg.output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL0;
        etl_cfg.source = XMC_ERU_ETL_SOURCE_A;

        XMC_ERU_ETL_Init(XMC_ERU0, 1U, &etl_cfg);

        // Configure ERU0 Event Generation (OGU Channel 0)
        XMC_ERU_OGU_CONFIG_t ogu_cfg = {};
        ogu_cfg.service_request = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER;
        
        XMC_ERU_OGU_Init(XMC_ERU0, 0U, &ogu_cfg);
    }
};