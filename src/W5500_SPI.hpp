#ifndef W5500_SPI_HPP_
#define W5500_SPI_HPP_

#include <cstdint>
#include <cstddef>
#include "xmc_gpio.h"
#include "xmc_spi.h"
#include "xmc_usic.h"
#include "xmc_scu.h"

#define CS_PORT XMC_GPIO_PORT0
#define CS_PIN  9

class W5500SPI {
public:
    static const uint16_t MAX_DATA_LEN = 512;

    W5500SPI() {}

    void init() {
        XMC_USIC_CH_Enable(XMC_SPI0_CH0);

        XMC_SPI_CH_CONFIG_t channel_config = {};
        channel_config.baudrate       = 1000000U; // 8 MHz SPI clock
        channel_config.bus_mode       = XMC_SPI_CH_BUS_MODE_MASTER;
        channel_config.selo_inversion = XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS;
        channel_config.parity_mode    = XMC_USIC_CH_PARITY_MODE_NONE;

        XMC_SPI_CH_Init(XMC_SPI0_CH0, &channel_config);
        XMC_SPI_CH_DisableFEM(XMC_SPI0_CH0);
        XMC_SPI_CH_SetBitOrderMsbFirst(XMC_SPI0_CH0);
        XMC_SPI_CH_SetWordLength(XMC_SPI0_CH0, 8U);
        XMC_SPI_CH_SetFrameLength(XMC_SPI0_CH0, 64U);

        XMC_SPI_CH_ConfigureShiftClockOutput(
            XMC_SPI0_CH0,
            XMC_SPI_CH_BRG_SHIFT_CLOCK_PASSIVE_LEVEL_0_DELAY_ENABLED,
            XMC_SPI_CH_BRG_SHIFT_CLOCK_OUTPUT_SCLK
        );

        XMC_SPI_CH_SetSlaveSelectDelay(XMC_SPI0_CH0, 2U);

        // MISO (P1.4)
        XMC_GPIO_CONFIG_t miso_config = {};
        miso_config.mode             = XMC_GPIO_MODE_INPUT_TRISTATE;
        miso_config.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD;
        XMC_GPIO_Init((XMC_GPIO_PORT_t *)PORT1_BASE, 4U, &miso_config);

        // Map MISO DX0 to Source 6 (DX0G on P1.4)
        XMC_SPI_CH_SetInputSource(XMC_SPI0_CH0, XMC_SPI_CH_INPUT_DIN0, 6U);
        XMC_SPI_CH_SetInputSource(XMC_SPI0_CH0, XMC_SPI_CH_INPUT_DIN1, 5U);
        XMC_SPI_CH_SetInputSource(XMC_SPI0_CH0, XMC_SPI_CH_INPUT_DIN2, 0U);
        XMC_SPI_CH_SetInputSource(XMC_SPI0_CH0, XMC_SPI_CH_INPUT_DIN3, 4U);

        XMC_SPI_CH_Start(XMC_SPI0_CH0);

        // MOSI (P1.5)
        XMC_GPIO_CONFIG_t mosi_config = {};
        mosi_config.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2;
        mosi_config.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH;
        mosi_config.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD;
        XMC_GPIO_Init((XMC_GPIO_PORT_t *)PORT1_BASE, 5U, &mosi_config);

        // SCLK (P2.0)
        XMC_GPIO_CONFIG_t sclk_config = {};
        sclk_config.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7;
        sclk_config.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH;
        sclk_config.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD;
        XMC_GPIO_Init((XMC_GPIO_PORT_t *)PORT2_BASE, 0U, &sclk_config);

        // CS (P0.9)
        XMC_GPIO_CONFIG_t cs_config = {};
        cs_config.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
        cs_config.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH;
        cs_config.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD;
        XMC_GPIO_Init(CS_PORT, CS_PIN, &cs_config);

        // Configure 16-Word Hardware TX & RX FIFOs
        XMC_USIC_CH_TXFIFO_Configure(XMC_SPI0_CH0, 16U, XMC_USIC_CH_FIFO_SIZE_16WORDS, 1U);
        XMC_USIC_CH_RXFIFO_Configure(XMC_SPI0_CH0, 0U,  XMC_USIC_CH_FIFO_SIZE_16WORDS, 0U);
    }

    inline void selectCS() {
        XMC_USIC_CH_TXFIFO_Flush(XMC_SPI0_CH0);
        XMC_USIC_CH_RXFIFO_Flush(XMC_SPI0_CH0);
        XMC_GPIO_SetOutputLow(CS_PORT, CS_PIN);
        for (volatile int i = 0; i < 30; ++i);
    }

    inline void deselectCS() {
        while (XMC_USIC_CH_GetTransmitBufferStatus(XMC_SPI0_CH0) == XMC_USIC_CH_TBUF_STATUS_BUSY);
        for (volatile int i = 0; i < 50; ++i);
        XMC_GPIO_SetOutputHigh(CS_PORT, CS_PIN);
        for (volatile int i = 0; i < 50; ++i);
    }

    uint8_t spiTransfer(uint8_t tx_data) {
        XMC_USIC_CH_TXFIFO_PutDataHPCMode(XMC_SPI0_CH0, tx_data, XMC_SPI_CH_MODE_STANDARD);
        while (XMC_USIC_CH_RXFIFO_IsEmpty(XMC_SPI0_CH0));
        return static_cast<uint8_t>(XMC_SPI_CH_GetReceivedData(XMC_SPI0_CH0));
    }

    bool writeBuffer(uint16_t address, uint8_t block_select, const uint8_t* buffer, uint16_t length) {
        if (length == 0 || length > MAX_DATA_LEN) return false;

        selectCS();
        spiTransfer(static_cast<uint8_t>(address >> 8));
        spiTransfer(static_cast<uint8_t>(address & 0xFF));
        spiTransfer(((block_select << 3) & 0xF8) | 0x04);
        for (uint16_t i = 0; i < length; ++i) {
            spiTransfer(buffer[i]);
        }
        deselectCS();
        return true;
    }

    bool readBuffer(uint16_t address, uint8_t block_select, uint8_t* buffer, uint16_t length) {
        if (length == 0 || length > MAX_DATA_LEN) return false;

        selectCS();
        spiTransfer(static_cast<uint8_t>(address >> 8));
        spiTransfer(static_cast<uint8_t>(address & 0xFF));
        spiTransfer((block_select << 3) & 0xF8);
        for (uint16_t i = 0; i < length; ++i) {
            buffer[i] = spiTransfer(0x00);
        }
        deselectCS();
        return true;
    }

    bool writeRegister(uint16_t address, uint8_t block_select, uint8_t value) {
        return writeBuffer(address, block_select, &value, 1);
    }

    uint8_t readRegister(uint16_t address, uint8_t block_select) {
        uint8_t val = 0;
        readBuffer(address, block_select, &val, 1);
        return val;
    }
};

#endif // W5500_SPI_HPP_