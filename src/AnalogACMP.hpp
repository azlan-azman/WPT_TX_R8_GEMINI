#pragma once

#include "XMC1300.h"
#include "xmc_acmp.h"

class AnalogACMP {
public:
    void init() {
        XMC_ACMP_CONFIG_t acmp_config = {};
        acmp_config.filter_disable = true;
        acmp_config.output_invert  = false;
        acmp_config.hysteresis     = XMC_ACMP_HYSTERESIS_10;

        XMC_ACMP_ClearLowPowerMode();

        // Cast COMPARATOR macro explicitly to (XMC_ACMP_t*)
        XMC_ACMP_Init((XMC_ACMP_t*)COMPARATOR, 1U, &acmp_config);
        XMC_ACMP_EnableComparator((XMC_ACMP_t*)COMPARATOR, 1U);
    }
};
