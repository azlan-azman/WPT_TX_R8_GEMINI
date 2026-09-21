#ifndef SPI_PROTOCOL_CONTROLLER_HPP_
#define SPI_PROTOCOL_CONTROLLER_HPP_

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "W5500_SPI.hpp"
#include "InverterPWM.hpp"
#include "WPTController.hpp"

#define FIRST_RX_LEN        4U   // [FuncID, LenHi, LenLo, HeaderCRC]
#define BODY_RX_LEN         92U  // Standard 92-Byte Body Payload

#define FUNC_READ_TELEMETRY 0x01U
#define FUNC_WRITE_COMMAND  0x02U
#define FUNC_READ_DIAG_LOG  0x03U

enum class SPI_STATE : uint8_t {
    RECEIVE_LEN = 0,
    RECEIVE_BODY,
    SILENCE
};

class SPIProtocolController {
public:
    explicit SPIProtocolController(W5500SPI* w5500);
    ~SPIProtocolController();

    bool init();

    // Main non-blocking task called in while(1)
    void process(uint32_t currentMs,
                 InverterPWM &inverter,
                 WPT_Controller &wptCtrl,
                 const Telemetry &telemetry);

    void pushRxByte(uint8_t byte);

    static uint8_t calculateCRC(const uint8_t *pData, size_t len) {
        if (!pData || len == 0) return 0;
        uint8_t crc = pData[0];
        for (size_t i = 1; i < len; ++i) {
            crc ^= pData[i];
        }
        return crc;
    }

private:
    W5500SPI* w5500_;
    SPI_STATE currentState;

    uint8_t funcID;
    uint8_t transID;
    uint16_t expectedBodyLen;

    alignas(4) uint8_t rxBuffer[384];
    alignas(4) uint8_t txBuffer[320];
    volatile size_t rxCount;

    uint32_t lastActivityTimer;
    uint32_t silenceTimer;
    uint32_t lastReopenTimer;

    uint16_t getTxFreeSize();
    void setupNetwork();
    bool startTCPServer();
    bool waitSnCR();
    void pollTCPData(uint32_t currentMs);
    void resetBus(uint32_t currentMs);
    void transmitBuffer(const uint8_t *buffer, size_t len);
};

#endif // SPI_PROTOCOL_CONTROLLER_HPP_