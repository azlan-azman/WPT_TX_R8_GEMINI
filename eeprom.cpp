#include "eeprom.h"
#include "WPTController.hpp"
#include "PinDefine.h"
#include "xmc_i2c.h"

extern "C" {
#include "xmc_common.h"
}

extern WPT_Controller WPT_Controller1;

int i2c_eeprom_write_long(unsigned int intDeviceAddress, unsigned int intEEAddress, unsigned char *pdata) {
    unsigned char rdata[6];
    intDeviceAddress = 0xA0;

    rdata[0] = static_cast<unsigned char>((intEEAddress >> 8) & 0xFF);
    rdata[1] = static_cast<unsigned char>(intEEAddress & 0xFF);
    rdata[2] = pdata[0];
    rdata[3] = pdata[1];
    rdata[4] = pdata[2];
    rdata[5] = pdata[3];

    // Lock bus to block LCD writes during EEPROM transmission
    WPT_Controller1.I2C_TX_Done = 0;

    uint32_t timeout = 1000U;
    while ((XMC_I2C0_CH1->TCSR & USIC_CH_TCSR_TDV_Msk) && --timeout) { __NOP(); }

    XMC_I2C_CH_MasterStart(XMC_I2C0_CH1, intDeviceAddress, XMC_I2C_CH_CMD_WRITE);

    for (int i = 0; i < 6; i++) {
        timeout = 1000U;
        while ((XMC_I2C0_CH1->TCSR & USIC_CH_TCSR_TDV_Msk) && --timeout) { __NOP(); }
        XMC_I2C_CH_MasterTransmit(XMC_I2C0_CH1, rdata[i]);
    }

    timeout = 1000U;
    while ((XMC_I2C0_CH1->TCSR & USIC_CH_TCSR_TDV_Msk) && --timeout) { __NOP(); }

    XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);

    XMC_Delay(10); // Internal EEPROM write cycle delay

    // Unlock bus
    WPT_Controller1.I2C_TX_Done = 1;
    return 1;
}

int i2c_eeprom_read_long(unsigned int intDeviceAddress, unsigned int intEEAddress, unsigned char *pdata) {
    unsigned char addrBuf[2];
    intDeviceAddress = 0xA0;

    addrBuf[0] = static_cast<unsigned char>((intEEAddress >> 8) & 0xFF);
    addrBuf[1] = static_cast<unsigned char>(intEEAddress & 0xFF);

    WPT_Controller1.I2C_TX_Done = 0;

    uint32_t timeout = 1000U;
    while ((XMC_I2C0_CH1->TCSR & USIC_CH_TCSR_TDV_Msk) && --timeout) { __NOP(); }

    XMC_I2C_CH_MasterStart(XMC_I2C0_CH1, intDeviceAddress, XMC_I2C_CH_CMD_WRITE);

    timeout = 1000U;
    while ((XMC_I2C0_CH1->TCSR & USIC_CH_TCSR_TDV_Msk) && --timeout) { __NOP(); }
    XMC_I2C_CH_MasterTransmit(XMC_I2C0_CH1, addrBuf[0]);

    timeout = 1000U;
    while ((XMC_I2C0_CH1->TCSR & USIC_CH_TCSR_TDV_Msk) && --timeout) { __NOP(); }
    XMC_I2C_CH_MasterTransmit(XMC_I2C0_CH1, addrBuf[1]);

    XMC_Delay(1);

    XMC_I2C_CH_MasterRepeatedStart(XMC_I2C0_CH1, intDeviceAddress, XMC_I2C_CH_CMD_READ);
    XMC_Delay(1);

    for (int i = 0; i < 4; i++) {
        if (i < 3) {
            XMC_I2C_CH_MasterReceiveAck(XMC_I2C0_CH1);
        } else {
            XMC_I2C_CH_MasterReceiveNack(XMC_I2C0_CH1);
        }
        XMC_Delay(1);
        pdata[i] = static_cast<unsigned char>(XMC_I2C_CH_GetReceivedData(XMC_I2C0_CH1));
    }

    XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);

    WPT_Controller1.I2C_TX_Done = 1;
    return 1;
}
