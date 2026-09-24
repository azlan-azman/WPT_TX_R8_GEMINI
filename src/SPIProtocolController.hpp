#ifndef SPI_PROTOCOL_CONTROLLER_HPP_
#define SPI_PROTOCOL_CONTROLLER_HPP_

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "W5500_SPI.hpp"
#include "InverterPWM.hpp"
#include "WPTController.hpp"

#define STD_REQ_LEN         10U   // [Func, MsgID, PWM, Opcode(2), Freq/Offset(4), Flags]
#define OTA_WRITE_REQ_LEN   266U  // 10B Header + 256B Flash Page Data
#define STD_RESP_LEN        30U   // [Status, MsgID, PWM, Opcode(2), Telemetry(24), Fault]
#define OTA_READ_RESP_LEN   258U  // [Status, MsgID] + 256B Flash Page Data

class SPIProtocolController {
public:
    explicit SPIProtocolController(W5500SPI* w5500);

    bool init();

    void process(uint32_t currentMs,
                 InverterPWM &inverter,
                 WPT_Controller &wptCtrl,
                 const Telemetry &telemetry);

private:
    W5500SPI* w5500_;

    alignas(4) uint8_t rxBuffer[384];
    alignas(4) uint8_t txBuffer[320];
    volatile size_t rxCount;

    uint32_t lastReopenTimer;

    void setupNetwork();
    bool startTCPServer();
    bool waitSnCR();
    uint16_t getTxFreeSize();
    void pollTCPData();
    void transmitBuffer(const uint8_t *buffer, size_t len);
};

#endif // SPI_PROTOCOL_CONTROLLER_HPP_