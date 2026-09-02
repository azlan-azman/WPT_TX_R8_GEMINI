#pragma once

#include <stdint.h>
#include <string.h>
#include "xmc_uart.h"
#include "InverterPWM.hpp"
#include "WPTController.hpp"

#define FIRST_RX_LEN        4U
#define FIRST_TX_LEN        3U
#define BODY_RX_LEN         92U

#define FUNC_READ_TELEMETRY 0x01U
#define FUNC_WRITE_COMMAND  0x02U
#define FUNC_READ_DIAG_LOG  0x03U

enum class UART_STATE : uint8_t {
    RECEIVE_LEN = 0,
    RECEIVE_BODY,
    SILENCE
};

class UARTProtocolController {
public:
    UARTProtocolController();
    ~UARTProtocolController();

    void pushRxByte(uint8_t byte);

    void process(uint32_t currentMs,
                 InverterPWM &inverter,
                 WPT_Controller &wptCtrl,
                 const Telemetry &telemetry);

    static uint8_t calculateCRC(const uint8_t *pData, size_t len) {
        if (!pData || len == 0) return 0;
        uint8_t crc = pData[0];
        for (size_t i = 1; i < len; ++i) {
            crc ^= pData[i];
        }
        return crc;
    }

private:
    UART_STATE currentState;

    alignas(4) uint8_t rxBuffer[384];
    alignas(4) uint8_t txBuffer[320]; // Fits 263B Staging Readback, 95B Diag Log, and 95B Telemetry

    uint8_t    funcID;
    uint8_t    transID;
    uint16_t   expectedBodyLen;
    volatile size_t rxCount;

    uint32_t   lastActivityTimer;

    void resetBus(uint32_t currentMs);
};
