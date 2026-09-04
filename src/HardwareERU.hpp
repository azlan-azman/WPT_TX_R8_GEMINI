#pragma once

#include "xmc_eru.h"

class HardwareERU {
public:
    void init() {
        // Configure ERU0 Event Detection (ETL Channel 1)
        XMC_ERU_ETL_CONFIG_t etl_cfg = {};
        etl_cfg.input_a = XMC_ERU_ETL_INPUT_A0; // Not used (see source = B below)
        // ERU0_ETL1.InputB[2] = ACMP1.OUT on XMC1302 TSSOP28.
        // From xmc1_eru_map.h: ERU0_ETL1_INPUTB_ACMP1_OUT = XMC_ERU_ETL_INPUT_B2
        etl_cfg.input_b = XMC_ERU_ETL_INPUT_B2;
        etl_cfg.status_flag_mode = XMC_ERU_ETL_STATUS_FLAG_MODE_HWCTRL;
        etl_cfg.edge_detection   = XMC_ERU_ETL_EDGE_DETECTION_RISING;
        // Route the ACMP1 zero-crossing event to OGU3 -> ERU0_IOUT3,
        // which is the ONLY path that reaches CCU40_CC41 event input D.
        // See xmc1_ccu4_map.h for TSSOP28: CCU41_IN3_ERU0_IOUT3 = 10
        // (XMC_CCU4_SLICE_INPUT_D = input selector 3, which maps to IOUT3).
        // Previously: OGU0 (CHANNEL0 -> IOUT0 -> CCU41_IN0, not input D).
        // Also INPUT_A was used (ACMP0.OUT), but the firmware uses ACMP1.
        etl_cfg.output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL3;
        etl_cfg.source = XMC_ERU_ETL_SOURCE_B;  // Detect edges on InputB (ACMP1.OUT)

        XMC_ERU_ETL_Init(XMC_ERU0, 1U, &etl_cfg);

        // Configure ERU0 Event Generation (OGU Channel 3) - matched to ETL trigger
        XMC_ERU_OGU_CONFIG_t ogu_cfg = {};
        ogu_cfg.service_request = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER;

        XMC_ERU_OGU_Init(XMC_ERU0, 3U, &ogu_cfg);
    }
};