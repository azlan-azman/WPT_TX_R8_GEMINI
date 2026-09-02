#pragma once

#include <stdint.h>
#include "xmc_i2c.h"
#include "xmc_gpio.h"
#include "PinDefine.h"

class I2CController {
public:
    void init() {
        XMC_I2C_CH_CONFIG_t cfg = {};
        cfg.baudrate = 100000;
        cfg.address  = 0;

        XMC_I2C_CH_Init(XMC_I2C0_CH1, &cfg);

        // Input stage routing matching DAVE configuration
        XMC_USIC_CH_SetInputSource(XMC_I2C0_CH1, XMC_USIC_CH_INPUT_DX0, 5); // SDA Input
        XMC_USIC_CH_SetInputSource(XMC_I2C0_CH1, XMC_USIC_CH_INPUT_DX1, 4); // SCL Input
        XMC_USIC_CH_SetInputSource(XMC_I2C0_CH1, XMC_USIC_CH_INPUT_DX3, 0U);
        XMC_USIC_CH_SetInputSource(XMC_I2C0_CH1, XMC_USIC_CH_INPUT_DX4, 0U);
        XMC_USIC_CH_SetInputSource(XMC_I2C0_CH1, XMC_USIC_CH_INPUT_DX5, 0U);

        XMC_USIC_CH_EnableInputDigitalFilter(XMC_I2C0_CH1, XMC_USIC_CH_INPUT_DX0);
        XMC_USIC_CH_EnableInputSync(XMC_I2C0_CH1, XMC_USIC_CH_INPUT_DX0);
        XMC_USIC_CH_EnableInputDigitalFilter(XMC_I2C0_CH1, XMC_USIC_CH_INPUT_DX1);
        XMC_USIC_CH_EnableInputSync(XMC_I2C0_CH1, XMC_USIC_CH_INPUT_DX1);

        XMC_I2C_CH_Start(XMC_I2C0_CH1);

        // Explicit GPIO Pin Initializations matching DAVE
        XMC_GPIO_CONFIG_t sda_cfg = {};
        sda_cfg.mode = XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT7; // ALT7 for P2.10 SDA
        sda_cfg.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
        XMC_GPIO_Init((XMC_GPIO_PORT_t *)PORT2_BASE, 10, &sda_cfg);

        XMC_GPIO_CONFIG_t scl_cfg = {};
        scl_cfg.mode = XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT6; // ALT6 for P2.11 SCL
        scl_cfg.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
        XMC_GPIO_Init((XMC_GPIO_PORT_t *)PORT2_BASE, 11, &scl_cfg);
    }
};
