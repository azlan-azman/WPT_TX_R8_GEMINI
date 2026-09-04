#pragma once

extern "C" {
#include "XMC1300.h"
#include "xmc_acmp.h"
}

class AnalogACMP {
public:
    void init() {
        XMC_ACMP_CONFIG_t acmp_config = {};
        acmp_config.filter_disable = true;                  // Matches COMP_REF_0_comp_module_config
        acmp_config.output_invert  = false;
        acmp_config.hysteresis     = XMC_ACMP_HYSTERESIS_10; // 10 mV Hysteresis

        XMC_ACMP_ClearLowPowerMode();

        // Initialize ACMP Instance 1 (ACMP1) on COMPARATOR base
        XMC_ACMP_Init((XMC_ACMP_t*)COMPARATOR, 1U, &acmp_config);
        XMC_ACMP_EnableComparator((XMC_ACMP_t*)COMPARATOR, 1U);
    }
};