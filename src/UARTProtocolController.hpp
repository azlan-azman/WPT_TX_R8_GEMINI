#pragma once

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "xmc_uart.h"
#include "InverterPWM.hpp"
#include "WPTController.hpp"

// OLI Protocol Fixed Frame Lengths
#define FIRST_RX_LEN        4U   // 4-Byte RX Header: [FuncID, LenHi, LenLo, HeaderCRC]
#define FIRST_TX_LEN        3U   // 3-Byte TX Header: [0x00, BodyLen, HeaderCRC]
#define BODY_RX_LEN         92U  // Standard 92-Byte Body Payload

// Function Identifiers
#define FUNC_READ_TELEMETRY 0x01U
#define FUNC_WRITE_COMMAND  0x02U
#define FUNC_READ_DIAG_LOG  0x03U

// UART Protocol State Machine States (Aligned with UART_OLI)
enum class UART_STATE : uint8_t {
    RECEIVE_LEN = 0,
    RECEIVE_BODY,
    TRANSMIT_LEN,
    WAITING_TX_HEADER,
    TRANSMIT_BODY,
    SILENCE
};

class UARTProtocolController {
public:
    UARTProtocolController();
    ~UARTProtocolController();

    alignas(4) uint8_t rxBuffer[384];
    alignas(4) uint8_t txBuffer[320];

    // Push incoming UART bytes into RX ring/linear buffer
    void pushRxByte(uint8_t byte);

    // Main protocol processing loop
    void process(uint32_t currentMs,
                 InverterPWM &inverter,
                 WPT_Controller &wptCtrl,
                 const Telemetry &telemetry);

    // XOR-based CRC calculation matching UART_OLI::getset_crc
    static uint8_t calculateCRC(const uint8_t *pData, size_t len) {
        if (!pData || len == 0) return 0;
        uint8_t crc = pData[0];
        for (size_t i = 1; i < len; ++i) {
            crc ^= pData[i];
        }
        return crc;
    }

    // LCD text processing helper from UART_OLI
    static void process_rec(char *lcd, const uint8_t *pData);

    // Active instance pointer for ISR delegation
    static UARTProtocolController* p_instance;

private:
    UART_STATE currentState;

    uint8_t    funcID;
    uint8_t    transID;
    uint16_t   expectedBodyLen;
    volatile size_t rxCount;

    uint32_t   lastActivityTimer;
    uint32_t   silenceTimer;

    void resetBus(uint32_t currentMs);
    void transmitBuffer(const uint8_t *buffer, size_t len);
    void transmitSplitBuffer(const uint8_t *buffer, size_t totalLen);
};