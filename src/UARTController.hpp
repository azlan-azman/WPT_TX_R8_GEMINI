#pragma once

#include "xmc_uart.h"
#include "xmc_gpio.h"

class UARTController {
public:
    void init() {
        // 1. Configure P1.4 (RX) and P1.5 (TX) Pins
        XMC_GPIO_CONFIG_t rx_pin_config = {};
        rx_pin_config.mode             = XMC_GPIO_MODE_INPUT_TRISTATE;
        rx_pin_config.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH;
        rx_pin_config.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD;
        XMC_GPIO_Init((XMC_GPIO_PORT_t *)PORT1_BASE, 4U, &rx_pin_config);

        XMC_GPIO_CONFIG_t tx_pin_config = {};
        tx_pin_config.mode         = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2;
        tx_pin_config.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
        XMC_GPIO_Init((XMC_GPIO_PORT_t *)PORT1_BASE, 5U, &tx_pin_config);

        // 2. Configure USIC0_CH0 Channel (115,200 Baud)
        XMC_UART_CH_CONFIG_t uart_cfg = {};
        uart_cfg.baudrate     = 256000U;
        uart_cfg.data_bits    = 8U;
        uart_cfg.frame_length = 8U;
        uart_cfg.stop_bits    = 1U;
        uart_cfg.oversampling = 16U;
        uart_cfg.parity_mode  = XMC_USIC_CH_PARITY_MODE_NONE;

        XMC_UART_CH_Init(XMC_UART0_CH0, &uart_cfg);

        // 3. Set Input Paths
        XMC_USIC_CH_SetInputSource(XMC_UART0_CH0, XMC_USIC_CH_INPUT_DX0, 6U); // P1.4
        XMC_USIC_CH_SetInputSource(XMC_UART0_CH0, XMC_USIC_CH_INPUT_DX3, 5U);
        XMC_USIC_CH_SetInputSource(XMC_UART0_CH0, XMC_USIC_CH_INPUT_DX5, 4U);

        // 4. Hardware FIFO Setup
        XMC_USIC_CH_RXFIFO_Configure(XMC_UART0_CH0, 0U,  XMC_USIC_CH_FIFO_SIZE_32WORDS, 0U);
        XMC_USIC_CH_TXFIFO_Configure(XMC_UART0_CH0, 32U, XMC_USIC_CH_FIFO_SIZE_32WORDS, 1U);

        // 5. Connect RX FIFO Events to NVIC IRQ 9 (SR0)
        XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(XMC_UART0_CH0, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 0U);
        XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(XMC_UART0_CH0, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, 0U);
        XMC_USIC_CH_RXFIFO_EnableEvent(XMC_UART0_CH0, XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE);

        NVIC_SetPriority((IRQn_Type)9, 3U);
        NVIC_EnableIRQ((IRQn_Type)9);

        // 6. Start UART Channel
        XMC_UART_CH_Start(XMC_UART0_CH0);
    }
};
