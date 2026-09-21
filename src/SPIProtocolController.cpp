#include "SPIProtocolController.hpp"
#include "xmc_flash.h"
#include "xmc_wdt.h"
#include "eeprom.h"

#define STAGING_START_ADDR 0x1001B000U
#define APP_START_ADDR     0x10001000U
#define SRAM_SIZE_ADDR     (*(volatile uint32_t *)0x20003FFCU)

extern "C" void execute_sram_swap(uint32_t dstAddr, uint32_t srcAddr, uint32_t imageSize);

SPIProtocolController::SPIProtocolController(W5500SPI* w5500)
    : w5500_(w5500),
      currentState(SPI_STATE::RECEIVE_LEN),
      funcID(0),
      transID(0),
      expectedBodyLen(0),
      rxCount(0),
      lastActivityTimer(0),
      silenceTimer(0),
      lastReopenTimer(0) {
    memset(rxBuffer, 0, sizeof(rxBuffer));
    memset(txBuffer, 0, sizeof(txBuffer));
}

SPIProtocolController::~SPIProtocolController() {}

bool SPIProtocolController::waitSnCR() {
    uint32_t timeout = 50000U;
    while (w5500_->readRegister(0x0001, 0x01) != 0x00) {
        if (--timeout == 0) return false;
    }
    return true;
}

uint16_t SPIProtocolController::getTxFreeSize() {
    uint8_t fsr[2] = {0};
    w5500_->readBuffer(0x0020, 0x01, fsr, 2);
    return (static_cast<uint16_t>(fsr[0]) << 8) | fsr[1];
}

void SPIProtocolController::setupNetwork() {
    const uint8_t mac[6]     = {0x00, 0x08, 0xDC, 0x01, 0x02, 0x03};
    const uint8_t ip[4]      = {192, 168, 2, 177};                 
    const uint8_t subnet[4]  = {255, 255, 255, 0};               
    const uint8_t gateway[4] = {192, 168, 2, 253};                

    w5500_->writeBuffer(0x0001, 0x00, gateway, 4);                
    w5500_->writeBuffer(0x0005, 0x00, subnet, 4);                 
    w5500_->writeBuffer(0x0009, 0x00, mac, 6);                      
    w5500_->writeBuffer(0x000F, 0x00, ip, 4);                      

    w5500_->writeRegister(0x001E, 0x01, 2);                        
    w5500_->writeRegister(0x001F, 0x01, 2);                        
}

bool SPIProtocolController::startTCPServer() {
    w5500_->writeRegister(0x0001, 0x01, 0x10); // Sn_CR = CLOSE
    if (!waitSnCR()) return false;

    w5500_->writeRegister(0x0000, 0x01, 0x01); // Sn_MR = TCP Mode

    uint8_t port[2] = {0x13, 0x88};             // Port 5000 (0x1388)
    w5500_->writeBuffer(0x0004, 0x01, port, 2);

    w5500_->writeRegister(0x0001, 0x01, 0x01); // Sn_CR = OPEN
    if (!waitSnCR()) return false;

    w5500_->writeRegister(0x0001, 0x01, 0x02); // Sn_CR = LISTEN
    if (!waitSnCR()) return false;

    __disable_irq();
    rxCount = 0;
    __enable_irq();

    return true;
}

bool SPIProtocolController::init() {
    w5500_->init();

    // Power-on delay for W5500 SPI/PLL stabilization
    for (volatile uint32_t i = 0; i < 100000U; ++i) { __NOP(); }

    // Retry VERSIONR read up to 10 times to prevent cold-boot aborts
    uint8_t version = 0;
    uint32_t retries = 10U;
    while (retries > 0) {
        version = w5500_->readRegister(0x0039, 0x00);
        if (version == 0x04) break;
        for (volatile uint32_t i = 0; i < 50000U; ++i) { __NOP(); }
        --retries;
    }

    if (version != 0x04) {
        return false; // Hardware SPI failure
    }

    setupNetwork();
    return startTCPServer();
}

void SPIProtocolController::pushRxByte(uint8_t byte) {
    if (rxCount < sizeof(rxBuffer)) {
        rxBuffer[rxCount++] = byte;
    }
}

void SPIProtocolController::resetBus(uint32_t currentMs) {
    __disable_irq();
    if (rxCount > 0) {
        memmove(rxBuffer, &rxBuffer[1], rxCount - 1);
        rxCount--;
    }
    lastActivityTimer = currentMs;
    __enable_irq();
}

void SPIProtocolController::transmitBuffer(const uint8_t *buffer, size_t len) {
    if (!buffer || len == 0) return;

    uint32_t timeout = 10000U;
    while (getTxFreeSize() < len && --timeout) { __NOP(); }
    if (timeout == 0) return;

    uint8_t ptr_buf[2] = {0};
    w5500_->readBuffer(0x0024, 0x01, ptr_buf, 2);
    uint16_t tx_ptr = (ptr_buf[0] << 8) | ptr_buf[1];

    uint16_t offset = tx_ptr & 0x07FF; // 2KB ring buffer boundary
    if ((offset + len) > 2048U) {
        uint16_t upper_len = 2048U - offset;
        uint16_t lower_len = static_cast<uint16_t>(len) - upper_len;
        w5500_->writeBuffer(offset, 0x02, buffer, upper_len);
        w5500_->writeBuffer(0x0000, 0x02, buffer + upper_len, lower_len);
    } else {
        w5500_->writeBuffer(offset, 0x02, buffer, static_cast<uint16_t>(len));
    }

    tx_ptr += static_cast<uint16_t>(len);
    ptr_buf[0] = static_cast<uint8_t>(tx_ptr >> 8);
    ptr_buf[1] = static_cast<uint8_t>(tx_ptr & 0xFF);
    w5500_->writeBuffer(0x0024, 0x01, ptr_buf, 2);

    w5500_->writeRegister(0x0001, 0x01, 0x20); // SEND Cmd
    waitSnCR();
}

void SPIProtocolController::pollTCPData(uint32_t currentMs) {
    uint8_t sn_sr = w5500_->readRegister(0x0003, 0x01);

    if (sn_sr == 0x17) { // SOCK_ESTABLISHED
        uint8_t rx_size_buf[2] = {0};
        w5500_->readBuffer(0x0026, 0x01, rx_size_buf, 2);
        uint16_t rx_size = (rx_size_buf[0] << 8) | rx_size_buf[1];

        if (rx_size > 0) {
            lastActivityTimer = currentMs;

            uint8_t ptr_buf[2] = {0};
            w5500_->readBuffer(0x0028, 0x01, ptr_buf, 2);
            uint16_t rx_ptr = (ptr_buf[0] << 8) | ptr_buf[1];

            uint8_t tcp_buf[260] = {0};
            uint16_t read_len = (rx_size > 260) ? 260 : rx_size;

            uint16_t offset = rx_ptr & 0x07FF;
            if ((offset + read_len) > 2048U) {
                uint16_t upper_len = 2048U - offset;
                uint16_t lower_len = read_len - upper_len;
                w5500_->readBuffer(offset, 0x03, tcp_buf, upper_len);
                w5500_->readBuffer(0x0000, 0x03, tcp_buf + upper_len, lower_len);
            } else {
                w5500_->readBuffer(offset, 0x03, tcp_buf, read_len);
            }

            // Advance rx_ptr by exact byte count read
            rx_ptr += read_len;
            ptr_buf[0] = static_cast<uint8_t>(rx_ptr >> 8);
            ptr_buf[1] = static_cast<uint8_t>(rx_ptr & 0xFF);
            w5500_->writeBuffer(0x0028, 0x01, ptr_buf, 2);
            w5500_->writeRegister(0x0001, 0x01, 0x40); // RECV Cmd
            waitSnCR();

            for (uint16_t i = 0; i < read_len; ++i) {
                pushRxByte(tcp_buf[i]);
            }
        }
    } else if ((sn_sr == 0x00 || sn_sr == 0x1C) && (currentMs - lastReopenTimer >= 500U)) {
        lastReopenTimer = currentMs;
        startTCPServer();
    }
}

void SPIProtocolController::process(uint32_t currentMs,
                                     InverterPWM &inverter,
                                     WPT_Controller &wptCtrl,
                                     const Telemetry &telemetry) {
    static uint32_t lastPollMs = 0;
    if (currentMs - lastPollMs >= 2U) {
        lastPollMs = currentMs;
        pollTCPData(currentMs);
    }

    bool stateChanged = false;
    do {
        stateChanged = false;

        size_t currentRxCount;
        __disable_irq();
        currentRxCount = rxCount;
        __enable_irq();

        // ATOMIC AT-LEAST-HEADER CHECK
        if (currentRxCount < FIRST_RX_LEN) {
            break;
        }

        // Validate Header (Bytes 0..3)
        uint8_t headerCrc = calculateCRC(rxBuffer, 3);
        uint8_t rxFunc = rxBuffer[0];
        bool validFunc = (rxFunc == FUNC_READ_TELEMETRY || rxFunc == FUNC_WRITE_COMMAND || rxFunc == FUNC_READ_DIAG_LOG);

        if (rxBuffer[3] != headerCrc || !validFunc) {
            // Misaligned byte stream: slide 1 byte forward to resync
            resetBus(currentMs);
            stateChanged = true;
            continue;
        }

        uint16_t bodyLen = (static_cast<uint16_t>(rxBuffer[1]) << 8) | rxBuffer[2];
        if (bodyLen == 0 || bodyLen > 320U) {
            resetBus(currentMs);
            stateChanged = true;
            continue;
        }

        uint16_t totalFrameLen = FIRST_RX_LEN + bodyLen; // e.g. 4 + 92 = 96 Bytes

        // Wait until FULL frame (Header + Body) arrives in rxBuffer
        if (currentRxCount < totalFrameLen) {
            break;
        }

        // Validate Body CRC (Last Byte of Frame)
        uint8_t bodyCrc = calculateCRC(&rxBuffer[FIRST_RX_LEN], bodyLen - 1);
        if (rxBuffer[totalFrameLen - 1] != bodyCrc) {
            resetBus(currentMs);
            stateChanged = true;
            continue;
        }

        // ATOMIC FRAME MATCH: Parse Command
        funcID  = rxFunc;
        transID = rxBuffer[FIRST_RX_LEN];

        if (funcID == FUNC_WRITE_COMMAND) {
            uint16_t newOpcode = (static_cast<uint16_t>(rxBuffer[6]) << 8) | rxBuffer[7];
            if (newOpcode != 0x0000) wptCtrl.WPT_OPCODE = newOpcode;

            uint8_t pwmCmd = rxBuffer[5];
            if (pwmCmd == 0x01) { wptCtrl.PWM_EN_STATE = true; inverter.start(); }
            else if (pwmCmd == 0x00) { wptCtrl.PWM_EN_STATE = false; inverter.stop(); }

            uint32_t newFreq = (static_cast<uint32_t>(rxBuffer[8]) << 24) |
                               (static_cast<uint32_t>(rxBuffer[9]) << 16) |
                               (static_cast<uint32_t>(rxBuffer[10]) << 8) |
                                static_cast<uint32_t>(rxBuffer[11]);

            if (newFreq >= 20000U && newFreq <= 200000U) {
                wptCtrl.Resonant_Freq = static_cast<float>(newFreq);
                inverter.setFrequency(newFreq);
            }

            uint8_t flags = rxBuffer[12];
            if (flags & 0x02) wptCtrl.AutoTune = true;
            else if (flags & 0x04) wptCtrl.AutoTune = false;
            if (flags & 0x08) wptCtrl.saveEEPROM();
            if (flags & 0x10) { wptCtrl.AutoStartDone = true; wptCtrl.DisplayMenuID = 2; }
        }

        // Strip Processed Frame from Buffer
        __disable_irq();
        memmove(rxBuffer, &rxBuffer[totalFrameLen], rxCount - totalFrameLen);
        rxCount -= totalFrameLen;
        __enable_irq();

        // Pack Contiguous 95-Byte Response
        memset(txBuffer, 0, sizeof(txBuffer));

        txBuffer[0] = 0x00;                                   // Status: OK
        txBuffer[1] = 0x00;                                   // Length MSB
        txBuffer[2] = BODY_RX_LEN;                            // Length LSB: 92 (0x5C)
        txBuffer[3] = calculateCRC(txBuffer, 3);              // Header CRC

        txBuffer[4] = transID;                                // Trans ID
        txBuffer[5] = wptCtrl.PWM_EN_STATE ? 0x01 : 0x00;    // PWM State

        txBuffer[6] = static_cast<uint8_t>((wptCtrl.WPT_OPCODE >> 8) & 0xFF);
        txBuffer[7] = static_cast<uint8_t>(wptCtrl.WPT_OPCODE & 0xFF);

        int32_t vInScaled = static_cast<int32_t>(telemetry.vIn * 1000.0f);
        txBuffer[8]  = static_cast<uint8_t>((vInScaled >> 24) & 0xFF);
        txBuffer[9]  = static_cast<uint8_t>((vInScaled >> 16) & 0xFF);
        txBuffer[10] = static_cast<uint8_t>((vInScaled >> 8) & 0xFF);
        txBuffer[11] = static_cast<uint8_t>(vInScaled & 0xFF);

        int32_t iInScaled = static_cast<int32_t>(telemetry.iIn * 1000.0f);
        txBuffer[12] = static_cast<uint8_t>((iInScaled >> 24) & 0xFF);
        txBuffer[13] = static_cast<uint8_t>((iInScaled >> 16) & 0xFF);
        txBuffer[14] = static_cast<uint8_t>((iInScaled >> 8) & 0xFF);
        txBuffer[15] = static_cast<uint8_t>(iInScaled & 0xFF);

        int32_t iOutScaled = static_cast<int32_t>(telemetry.iOut * 1000.0f);
        txBuffer[16] = static_cast<uint8_t>((iOutScaled >> 24) & 0xFF);
        txBuffer[17] = static_cast<uint8_t>((iOutScaled >> 16) & 0xFF);
        txBuffer[18] = static_cast<uint8_t>((iOutScaled >> 8) & 0xFF);
        txBuffer[19] = static_cast<uint8_t>(iOutScaled & 0xFF);

        int32_t tempScaled = static_cast<int32_t>(telemetry.tempC * 10.0f);
        txBuffer[20] = static_cast<uint8_t>((tempScaled >> 24) & 0xFF);
        txBuffer[21] = static_cast<uint8_t>((tempScaled >> 16) & 0xFF);
        txBuffer[22] = static_cast<uint8_t>((tempScaled >> 8) & 0xFF);
        txBuffer[23] = static_cast<uint8_t>(tempScaled & 0xFF);

        int32_t pInScaled = static_cast<int32_t>((telemetry.vIn * telemetry.iIn) * 1000.0f);
        txBuffer[24] = static_cast<uint8_t>((pInScaled >> 24) & 0xFF);
        txBuffer[25] = static_cast<uint8_t>((pInScaled >> 16) & 0xFF);
        txBuffer[26] = static_cast<uint8_t>((pInScaled >> 8) & 0xFF);
        txBuffer[27] = static_cast<uint8_t>(pInScaled & 0xFF);

        uint32_t resFreqHz = static_cast<uint32_t>(wptCtrl.actualResonantFreq * 1000.0f);
        txBuffer[28] = static_cast<uint8_t>((resFreqHz >> 24) & 0xFF);
        txBuffer[29] = static_cast<uint8_t>((resFreqHz >> 16) & 0xFF);
        txBuffer[30] = static_cast<uint8_t>((resFreqHz >> 8)  & 0xFF);
        txBuffer[31] = static_cast<uint8_t>(resFreqHz & 0xFF);

        txBuffer[32] = static_cast<uint8_t>(wptCtrl.fault);

        // Trailing Body CRC at Index 94
        txBuffer[94] = calculateCRC(&txBuffer[4], 90);

        transmitBuffer(txBuffer, 95);

        lastActivityTimer = currentMs;
        stateChanged = true;
    } while (stateChanged);
}