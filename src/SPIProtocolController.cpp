#include "SPIProtocolController.hpp"
#include "xmc_flash.h"
#include "xmc_wdt.h"

#define STAGING_START_ADDR 0x1001B000U
#define APP_START_ADDR     0x10001000U
#define SRAM_SIZE_ADDR     (*(volatile uint32_t *)0x20003FFCU)

extern "C" void execute_sram_swap(uint32_t dstAddr, uint32_t srcAddr, uint32_t imageSize);

SPIProtocolController::SPIProtocolController(W5500SPI* w5500)
    : w5500_(w5500), rxCount(0), lastReopenTimer(0) {
    memset(rxBuffer, 0, sizeof(rxBuffer));
    memset(txBuffer, 0, sizeof(txBuffer));
}

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

    w5500_->writeRegister(0x001E, 0x01, 2); // 2KB RX
    w5500_->writeRegister(0x001F, 0x01, 2); // 2KB TX
}

bool SPIProtocolController::startTCPServer() {
    w5500_->writeRegister(0x0001, 0x01, 0x10); // CLOSE
    if (!waitSnCR()) return false;

    w5500_->writeRegister(0x0000, 0x01, 0x01); // TCP Mode

    uint8_t port[2] = {0x13, 0x88};             // Port 5000
    w5500_->writeBuffer(0x0004, 0x01, port, 2);

    w5500_->writeRegister(0x0001, 0x01, 0x01); // OPEN
    if (!waitSnCR()) return false;

    w5500_->writeRegister(0x0001, 0x01, 0x02); // LISTEN
    if (!waitSnCR()) return false;

    __disable_irq();
    rxCount = 0;
    __enable_irq();

    return true;
}

bool SPIProtocolController::init() {
    w5500_->init();

    for (volatile uint32_t i = 0; i < 100000U; ++i) { __NOP(); }

    uint8_t version = 0;
    uint32_t retries = 10U;
    while (retries > 0) {
        version = w5500_->readRegister(0x0039, 0x00);
        if (version == 0x04) break;
        for (volatile uint32_t i = 0; i < 50000U; ++i) { __NOP(); }
        --retries;
    }

    if (version != 0x04) return false;

    setupNetwork();
    return startTCPServer();
}

void SPIProtocolController::transmitBuffer(const uint8_t *buffer, size_t len) {
    if (!buffer || len == 0) return;

    uint32_t timeout = 10000U;
    while (getTxFreeSize() < len && --timeout) { __NOP(); }
    if (timeout == 0) return;

    uint8_t ptr_buf[2] = {0};
    w5500_->readBuffer(0x0024, 0x01, ptr_buf, 2);
    uint16_t tx_ptr = (ptr_buf[0] << 8) | ptr_buf[1];

    uint16_t offset = tx_ptr & 0x07FF;
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

    w5500_->writeRegister(0x0001, 0x01, 0x20); // SEND
    waitSnCR();
}

void SPIProtocolController::pollTCPData() {
    uint8_t sn_sr = w5500_->readRegister(0x0003, 0x01);

    if (sn_sr == 0x17) { // SOCK_ESTABLISHED
        uint8_t rx_size_buf[2] = {0};
        w5500_->readBuffer(0x0026, 0x01, rx_size_buf, 2);
        uint16_t rx_size = (rx_size_buf[0] << 8) | rx_size_buf[1];

        if (rx_size > 0) {
            uint8_t ptr_buf[2] = {0};
            w5500_->readBuffer(0x0028, 0x01, ptr_buf, 2);
            uint16_t rx_ptr = (ptr_buf[0] << 8) | ptr_buf[1];

            uint8_t tcp_buf[280] = {0};
            uint16_t read_len = (rx_size > 280) ? 280 : rx_size;

            uint16_t offset = rx_ptr & 0x07FF;
            if ((offset + read_len) > 2048U) {
                uint16_t upper_len = 2048U - offset;
                uint16_t lower_len = read_len - upper_len;
                w5500_->readBuffer(offset, 0x03, tcp_buf, upper_len);
                w5500_->readBuffer(0x0000, 0x03, tcp_buf + upper_len, lower_len);
            } else {
                w5500_->readBuffer(offset, 0x03, tcp_buf, read_len);
            }

            rx_ptr += read_len;
            ptr_buf[0] = static_cast<uint8_t>(rx_ptr >> 8);
            ptr_buf[1] = static_cast<uint8_t>(rx_ptr & 0xFF);
            w5500_->writeBuffer(0x0028, 0x01, ptr_buf, 2);
            w5500_->writeRegister(0x0001, 0x01, 0x40); // RECV
            waitSnCR();

            __disable_irq();
            for (uint16_t i = 0; i < read_len; ++i) {
                if (rxCount < sizeof(rxBuffer)) {
                    rxBuffer[rxCount++] = tcp_buf[i];
                }
            }
            __enable_irq();
        }
    }
}

void SPIProtocolController::process(uint32_t currentMs,
                                     InverterPWM &inverter,
                                     WPT_Controller &wptCtrl,
                                     const Telemetry &telemetry) {
    static uint32_t lastPollMs = 0;
    if (currentMs - lastPollMs >= 2U) {
        lastPollMs = currentMs;
        pollTCPData();

        uint8_t sn_sr = w5500_->readRegister(0x0003, 0x01);
        if ((sn_sr == 0x00 || sn_sr == 0x1C) && (currentMs - lastReopenTimer >= 500U)) {
            lastReopenTimer = currentMs;
            startTCPServer();
        }
    }

    if (rxCount < STD_REQ_LEN) return;

    uint8_t  cmdID  = rxBuffer[0];
    uint8_t  msgID = rxBuffer[1];
    uint8_t  pwmCmd  = rxBuffer[2];
    uint16_t opcode  = (static_cast<uint16_t>(rxBuffer[3]) << 8) | rxBuffer[4];
    uint32_t val32   = (static_cast<uint32_t>(rxBuffer[5]) << 24) |
                       (static_cast<uint32_t>(rxBuffer[6]) << 16) |
                       (static_cast<uint32_t>(rxBuffer[7]) << 8)  |
                        static_cast<uint32_t>(rxBuffer[8]);
    uint8_t  flags   = rxBuffer[9];

    // Handle OTA Write Chunk Frame Size Check
    if (opcode == 0xFFF1 && rxCount < OTA_WRITE_REQ_LEN) {
        return; // Wait for full 256-byte page chunk to arrive
    }

    uint16_t consumedLen = (opcode == 0xFFF1) ? OTA_WRITE_REQ_LEN : STD_REQ_LEN;

    bool isReadbackCommand = false;

    if (cmdID == 0x02) {
        // --- OTA STAGING & FLASH COMMANDS ---
        if (opcode == 0xFFF0) { // Staging Init
            inverter.stop();
            wptCtrl.PWM_EN_STATE = false;
            wptCtrl.AutoStartDone = true;
            wptCtrl.DisplayMenuID = 2;

            volatile uint32_t* pStaging = reinterpret_cast<volatile uint32_t*>(STAGING_START_ADDR);
            if (*pStaging != 0xFFFFFFFFU) {
                uint32_t addr = STAGING_START_ADDR;
                uint32_t end  = STAGING_START_ADDR + val32;
                while (addr < end) {
                    XMC_FLASH_EraseSector(reinterpret_cast<uint32_t*>(addr));
                    XMC_WDT_Service();
                    addr += 4096U;
                }
            }
            SRAM_SIZE_ADDR = val32;
        }
        else if (opcode == 0xFFF1) { // Page Flash Write
            uint32_t targetAddr = STAGING_START_ADDR + val32;
            alignas(4) uint32_t pageBuf[64];
            memcpy(pageBuf, &rxBuffer[10], 256U);

            XMC_FLASH_ClearStatus();
            XMC_FLASH_ProgramPage(reinterpret_cast<uint32_t*>(targetAddr), pageBuf);
        }
        else if (opcode == 0xFFF2) { // Swap & Execute Reboot
            inverter.stop();
            wptCtrl.PWM_EN_STATE = false;

            uint32_t totalImageSize = (val32 >= 1024U && val32 <= 102400U) ? val32 : SRAM_SIZE_ADDR;
            if (totalImageSize < 1024U || totalImageSize > 102400U) totalImageSize = 74240U;

            memset(txBuffer, 0, STD_RESP_LEN);
            txBuffer[0] = 0x00;
            txBuffer[1] = msgID;
            transmitBuffer(txBuffer, STD_RESP_LEN);

            execute_sram_swap(APP_START_ADDR, STAGING_START_ADDR, totalImageSize);
        }
        else if (opcode == 0xFFF3) { // Readback Flash
            isReadbackCommand = true;
            uint32_t srcAddr = STAGING_START_ADDR + val32;
            memset(txBuffer, 0, OTA_READ_RESP_LEN);
            txBuffer[0] = 0x00;
            txBuffer[1] = msgID;
            memcpy(&txBuffer[2], reinterpret_cast<const void*>(srcAddr), 256U);
        }
        // --- STANDARD INVERTER CONTROL ---
        else {
            if (opcode != 0x0000) wptCtrl.WPT_OPCODE = opcode;

            if (pwmCmd == 0x01)      { wptCtrl.PWM_EN_STATE = true; inverter.start(); }
            else if (pwmCmd == 0x00) { wptCtrl.PWM_EN_STATE = false; inverter.stop(); }

            if (val32 >= 20000U && val32 <= 200000U) {
                wptCtrl.Resonant_Freq = static_cast<float>(val32);
                inverter.setFrequency(val32);
            }

            if (flags & 0x02)      wptCtrl.AutoTune = true;
            else if (flags & 0x04) wptCtrl.AutoTune = false;
            if (flags & 0x08)      wptCtrl.saveEEPROM();
            if (flags & 0x10)    { wptCtrl.AutoStartDone = true; wptCtrl.DisplayMenuID = 2; }
        }
    }

    // Shift RX buffer past consumed frame
    __disable_irq();
    memmove(rxBuffer, &rxBuffer[consumedLen], rxCount - consumedLen);
    rxCount -= consumedLen;
    __enable_irq();

    // Transmit Response
    if (isReadbackCommand) {
        transmitBuffer(txBuffer, OTA_READ_RESP_LEN);
    } else {
        memset(txBuffer, 0, STD_RESP_LEN);

        txBuffer[0] = 0x00; // Status: OK
        txBuffer[1] = msgID;
        txBuffer[2] = wptCtrl.PWM_EN_STATE ? 0x01 : 0x00;

        txBuffer[3] = static_cast<uint8_t>((wptCtrl.WPT_OPCODE >> 8) & 0xFF);
        txBuffer[4] = static_cast<uint8_t>(wptCtrl.WPT_OPCODE & 0xFF);

        int32_t vInScaled  = static_cast<int32_t>(telemetry.vIn * 1000.0f);
        int32_t iInScaled  = static_cast<int32_t>(telemetry.iIn * 1000.0f);
        int32_t iOutScaled = static_cast<int32_t>(telemetry.iOut * 1000.0f);
        int32_t tempScaled = static_cast<int32_t>(telemetry.tempC * 10.0f);
        int32_t pInScaled  = static_cast<int32_t>((telemetry.vIn * telemetry.iIn) * 1000.0f);
        uint32_t resFreqHz = static_cast<uint32_t>(wptCtrl.actualResonantFreq * 1000.0f);

        txBuffer[5]  = (vInScaled >> 24) & 0xFF; txBuffer[6]  = (vInScaled >> 16) & 0xFF;
        txBuffer[7]  = (vInScaled >> 8)  & 0xFF; txBuffer[8]  = vInScaled & 0xFF;

        txBuffer[9]  = (iInScaled >> 24) & 0xFF; txBuffer[10] = (iInScaled >> 16) & 0xFF;
        txBuffer[11] = (iInScaled >> 8)  & 0xFF; txBuffer[12] = iInScaled & 0xFF;

        txBuffer[13] = (iOutScaled >> 24) & 0xFF; txBuffer[14] = (iOutScaled >> 16) & 0xFF;
        txBuffer[15] = (iOutScaled >> 8)  & 0xFF; txBuffer[16] = iOutScaled & 0xFF;

        txBuffer[17] = (tempScaled >> 24) & 0xFF; txBuffer[18] = (tempScaled >> 16) & 0xFF;
        txBuffer[19] = (tempScaled >> 8)  & 0xFF; txBuffer[20] = tempScaled & 0xFF;

        txBuffer[21] = (pInScaled >> 24) & 0xFF; txBuffer[22] = (pInScaled >> 16) & 0xFF;
        txBuffer[23] = (pInScaled >> 8)  & 0xFF; txBuffer[24] = pInScaled & 0xFF;

        txBuffer[25] = (resFreqHz >> 24) & 0xFF; txBuffer[26] = (resFreqHz >> 16) & 0xFF;
        txBuffer[27] = (resFreqHz >> 8)  & 0xFF; txBuffer[28] = resFreqHz & 0xFF;

        txBuffer[29] = static_cast<uint8_t>(wptCtrl.fault);

        transmitBuffer(txBuffer, STD_RESP_LEN);
    }
}