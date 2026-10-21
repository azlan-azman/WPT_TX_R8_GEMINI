#include "UARTProtocolController.hpp"
#include "ADCData.h"
#include "xmc_flash.h"
#include "xmc_wdt.h" // Service Watchdog Timer
#include "eeprom.h"
#include "ADCData.h"

#define STAGING_START_ADDR 0x1001B000U
#define APP_START_ADDR     0x10001000U
#define SRAM_SIZE_ADDR     (*(volatile uint32_t *)0x20003FFCU)

extern "C" void execute_sram_swap(uint32_t dstAddr, uint32_t srcAddr, uint32_t imageSize);

UARTProtocolController* UARTProtocolController::p_instance = nullptr;

UARTProtocolController::UARTProtocolController()
    : currentState(UART_STATE::RECEIVE_LEN),
      funcID(0),
      transID(0),
      expectedBodyLen(0),
      rxCount(0),
      lastActivityTimer(0),
      silenceTimer(0) {
    p_instance = this;
    memset(rxBuffer, 0, sizeof(rxBuffer));
    memset(txBuffer, 0, sizeof(txBuffer));
}

UARTProtocolController::~UARTProtocolController() {
    if (p_instance == this) {
        p_instance = nullptr;
    }
}

void UARTProtocolController::pushRxByte(uint8_t byte) {
    if (rxCount < sizeof(rxBuffer)) {
        rxBuffer[rxCount++] = byte;
    }
}

void UARTProtocolController::process_rec(char *lcd, const uint8_t *pData) {
    if (!lcd || !pData) return;
    for (int i = 0; i < 22; ++i) {
        *lcd = *pData + 48;
        lcd++;
        pData++;
    }
}

void UARTProtocolController::resetBus(uint32_t currentMs) {
    __disable_irq();
    XMC_USIC_CH_TXFIFO_Flush(XMC_UART0_CH0);
    XMC_USIC_CH_RXFIFO_Flush(XMC_UART0_CH0);

    rxCount = 0;
    lastActivityTimer = currentMs;
    silenceTimer = currentMs;
    currentState = UART_STATE::SILENCE;
    __enable_irq();
}

void UARTProtocolController::transmitBuffer(const uint8_t *buffer, size_t len) {
    if (!buffer || len == 0) return;
    for (size_t i = 0; i < len; ++i) {
        while (XMC_USIC_CH_TXFIFO_IsFull(XMC_UART0_CH0)) {
            __NOP();
        }
        XMC_UART_CH_Transmit(XMC_UART0_CH0, buffer[i]);
    }
}

void UARTProtocolController::transmitSplitBuffer(const uint8_t *buffer, size_t totalLen) {
    if (!buffer || totalLen < 3) return;

    // 1. Transmit 3-Byte Header (LenMSB, LenLSB, HeaderCRC)
    transmitBuffer(buffer, 3);

    // 2. 1.5ms Inter-chunk pause: Allows STM32 main loop to process Case 50
    //    and arm HAL_UART_Receive_IT for the body without dropping bytes.
    for (volatile uint32_t i = 0; i < 22000U; ++i) {
        __NOP();
    }

    // 3. Transmit remaining Body bytes
    if (totalLen > 3) {
        transmitBuffer(&buffer[3], totalLen - 3);
    }
}

void UARTProtocolController::process(uint32_t currentMs,
                                     InverterPWM &inverter,
                                     WPT_Controller &wptCtrl,
                                     const Telemetry1 &telemetry) {
    // Clear USIC UART line noise or stop bit errors
    uint32_t status = XMC_UART_CH_GetStatusFlag(XMC_UART0_CH0);
    if (status & (XMC_UART_CH_STATUS_FLAG_RECEIVER_NOISE_DETECTED |
                  XMC_UART_CH_STATUS_FLAG_FORMAT_ERROR_IN_STOP_BIT_0 |
                  XMC_UART_CH_STATUS_FLAG_FORMAT_ERROR_IN_STOP_BIT_1)) {
        XMC_UART_CH_ClearStatusFlag(XMC_UART0_CH0, status);
    }

    bool stateChanged = false;
    do {
        stateChanged = false;

        size_t currentRxCount;
        __disable_irq();
        currentRxCount = rxCount;
        __enable_irq();

        switch (currentState) {
            case UART_STATE::SILENCE: {
                // 500ms silence recovery window matching UART_OLI
                if (currentMs - silenceTimer >= 500U) {
                    __disable_irq();
                    XMC_USIC_CH_TXFIFO_Flush(XMC_UART0_CH0);
                    XMC_USIC_CH_RXFIFO_Flush(XMC_UART0_CH0);
                    rxCount = 0;
                    lastActivityTimer = currentMs;
                    currentState = UART_STATE::RECEIVE_LEN;
                    stateChanged = true;
                    __enable_irq();
                }
                break;
            }

            case UART_STATE::RECEIVE_LEN: {
                if (currentRxCount >= FIRST_RX_LEN) {
                    // 1. Validate 4-Byte RX Header CRC (bytes 0..2 vs byte 3)
                    uint8_t headerCrc = calculateCRC(rxBuffer, 3);
                    if (rxBuffer[3] == headerCrc) {
                        funcID          = rxBuffer[0];
                        expectedBodyLen = (static_cast<uint16_t>(rxBuffer[1]) << 8) | rxBuffer[2];

                        __disable_irq();
                        memmove(rxBuffer, &rxBuffer[FIRST_RX_LEN], rxCount - FIRST_RX_LEN);
                        rxCount -= FIRST_RX_LEN;
                        __enable_irq();

                        lastActivityTimer = currentMs;
                        currentState = UART_STATE::RECEIVE_BODY;
                        stateChanged = true;
                    } else {
                        // Header CRC mismatch -> enter SILENCE state
                        __disable_irq();
                        memmove(rxBuffer, &rxBuffer[1], rxCount - 1);
                        rxCount--;
                        __enable_irq();

                        resetBus(currentMs);
                    }
                }
                break;
            }

            case UART_STATE::RECEIVE_BODY: {
                // Timeout check
                if (currentMs - lastActivityTimer > 500U) {
                    resetBus(currentMs);
                    break;
                }

                if (expectedBodyLen == 0 || expectedBodyLen > 320U) {
                    resetBus(currentMs);
                    break;
                }

                if (currentRxCount >= expectedBodyLen) {
                    // 2. Validate Body CRC (bytes 0..expectedBodyLen-2 vs last byte)
                    uint8_t bodyCrc = calculateCRC(rxBuffer, expectedBodyLen - 1);
                    if (rxBuffer[expectedBodyLen - 1] == bodyCrc) {
                        // transID is byte 0 of the body payload (matching UART_OLI)
                        transID = rxBuffer[0];

                        bool isStagingCommand  = false;
                        bool isDiagLogRequest  = false;
                        bool isReadbackCommand = false;

                        // =========================================================
                        // COMMAND DECODING (funcID == 0x02)
                        // =========================================================
                        if (funcID == FUNC_WRITE_COMMAND) {
                            uint16_t newOpcode = (static_cast<uint16_t>(rxBuffer[2]) << 8) | rxBuffer[3];

                            // --- OPCODE 0xFFF0: ERASE STAGING FLASH ---
                            if (newOpcode == 0xFFF0) {
                                isStagingCommand = true;

                                // Disarm PWM inverter and cancel auto-start countdown
                                inverter.stop();
                                wptCtrl.PWM_EN_STATE = false;
                                wptCtrl.AutoStartDone = true;
                                wptCtrl.DisplayMenuID = 2;

                                uint32_t totalImageSize = (static_cast<uint32_t>(rxBuffer[4]) << 24) |
                                                        (static_cast<uint32_t>(rxBuffer[5]) << 16) |
                                                        (static_cast<uint32_t>(rxBuffer[6]) << 8)  |
                                                        static_cast<uint32_t>(rxBuffer[7]);

                                // Fast-path: Skip 1.1s sector erase if staging flash is already erased (0xFFFFFFFF)
                                volatile uint32_t* pStaging = reinterpret_cast<volatile uint32_t*>(STAGING_START_ADDR);
                                if (*pStaging != 0xFFFFFFFFU) {
                                    uint32_t addr = STAGING_START_ADDR;
                                    uint32_t end  = STAGING_START_ADDR + totalImageSize;
                                    while (addr < end) {
                                        XMC_FLASH_EraseSector(reinterpret_cast<uint32_t*>(addr));
                                        XMC_WDT_Service(); // Prevent watchdog reset during sector erase
                                        addr += 4096U;
                                    }
                                }

                                SRAM_SIZE_ADDR = totalImageSize;
                            }
                            // --- OPCODE 0xFFF1: WRITE STAGING PAGE ---
                            else if (newOpcode == 0xFFF1) {
                                isStagingCommand = true;
                                uint32_t offset = (static_cast<uint32_t>(rxBuffer[4]) << 24) |
                                                  (static_cast<uint32_t>(rxBuffer[5]) << 16) |
                                                  (static_cast<uint32_t>(rxBuffer[6]) << 8)  |
                                                   static_cast<uint32_t>(rxBuffer[7]);

                                uint32_t targetAddr = STAGING_START_ADDR + offset;

                                alignas(4) uint32_t pageBuf[64];
                                memcpy(pageBuf, &rxBuffer[8], 256U);

                                XMC_FLASH_ClearStatus();
                                XMC_FLASH_ProgramPage(reinterpret_cast<uint32_t*>(targetAddr), pageBuf);
                            }
                            // --- OPCODE 0xFFF2: SWAP TRIGGER & EXECUTION ---
                            else if (newOpcode == 0xFFF2) {
                                inverter.stop();
                                wptCtrl.PWM_EN_STATE = false;

                                uint32_t totalImageSize = 0;
                                if (expectedBodyLen >= 8) {
                                    totalImageSize = (static_cast<uint32_t>(rxBuffer[4]) << 24) |
                                                     (static_cast<uint32_t>(rxBuffer[5]) << 16) |
                                                     (static_cast<uint32_t>(rxBuffer[6]) << 8)  |
                                                      static_cast<uint32_t>(rxBuffer[7]);
                                }
                                if (totalImageSize < 1024U || totalImageSize > 102400U) {
                                    totalImageSize = SRAM_SIZE_ADDR;
                                }
                                if (totalImageSize < 1024U || totalImageSize > 102400U) {
                                    totalImageSize = 74240U;
                                }

                                // Send 6-byte ACK frame back to Flasher before swap
                                memset(txBuffer, 0, sizeof(txBuffer));
                                txBuffer[0] = 0x00;
                                txBuffer[1] = 0x05;
                                txBuffer[2] = calculateCRC(txBuffer, 2);
                                txBuffer[3] = transID;
                                txBuffer[4] = 0x55;
                                txBuffer[5] = calculateCRC(&txBuffer[3], 2);

                                transmitBuffer(txBuffer, 6);

                                while (!XMC_USIC_CH_TXFIFO_IsEmpty(XMC_UART0_CH0)) { __NOP(); }
                                for (volatile uint32_t i = 0; i < 200000U; ++i) { __NOP(); }

                                // Execute SRAM Flasher swap
                                execute_sram_swap(APP_START_ADDR, STAGING_START_ADDR, totalImageSize);
                            }
                            // --- OPCODE 0xFFF3: READBACK STAGING PAGE ---
                            else if (newOpcode == 0xFFF3) {
                                isReadbackCommand = true;
                                uint32_t offset = (static_cast<uint32_t>(rxBuffer[4]) << 24) |
                                                (static_cast<uint32_t>(rxBuffer[5]) << 16) |
                                                (static_cast<uint32_t>(rxBuffer[6]) << 8)  |
                                                static_cast<uint32_t>(rxBuffer[7]);

                                uint32_t srcAddr = STAGING_START_ADDR + offset;

                                memset(txBuffer, 0, sizeof(txBuffer));
                                
                                // Corrected 3-Byte UART Header for STM32 Compatibility: [LenMSB, LenLSB, HeaderCRC]
                                txBuffer[0] = 0x01; // LenMSB (0x0103 = 259 Bytes)
                                txBuffer[1] = 0x03; // LenLSB
                                txBuffer[2] = calculateCRC(txBuffer, 2); // Header CRC (0x01 ^ 0x03 = 0x02)

                                txBuffer[3] = transID;
                                txBuffer[4] = 0x55; // Status OK ACK

                                memcpy(&txBuffer[5], reinterpret_cast<const void*>(srcAddr), 256U);
                                txBuffer[261] = calculateCRC(&txBuffer[3], 258);
                            }
                            // --- LIVE CONTROL CENTER COMMANDS ---
                            else {
                                if (newOpcode != 0x0000) {
                                    wptCtrl.WPT_OPCODE = newOpcode;
                                }

                                uint8_t pwmCmd = rxBuffer[1];
                                if (pwmCmd == 0x01) {
                                    wptCtrl.PWM_EN_STATE = true;
                                    inverter.start();
                                } else if (pwmCmd == 0x00) {
                                    wptCtrl.PWM_EN_STATE = false;
                                    inverter.stop();
                                }

                                uint32_t newFreq = (static_cast<uint32_t>(rxBuffer[4]) << 24) |
                                                   (static_cast<uint32_t>(rxBuffer[5]) << 16) |
                                                   (static_cast<uint32_t>(rxBuffer[6]) << 8)  |
                                                    static_cast<uint32_t>(rxBuffer[7]);

                                if (newFreq >= 20000U && newFreq <= 200000U) {
                                    wptCtrl.Resonant_Freq = static_cast<float>(newFreq);
                                    inverter.setFrequency(newFreq);
                                }

                                uint8_t flags = rxBuffer[8];
                                if (flags & 0x02) {
                                    wptCtrl.AutoTune = true;
                                } else if (flags & 0x04) {
                                    wptCtrl.AutoTune = false;
                                }

                                if (flags & 0x08) {
                                    wptCtrl.saveEEPROM();
                                }

                                if (flags & 0x10) {
                                    wptCtrl.AutoStartDone = true;
                                    wptCtrl.DisplayMenuID = 2;
                                }
                            }
                        }
                        // =========================================================
                        // EEPROM DIAGNOSTIC LOG REQUEST (funcID == 0x03)
                        // =========================================================
                        else if (funcID == FUNC_READ_DIAG_LOG) {
                            isDiagLogRequest = true;
                            uint8_t requestedIndex = rxBuffer[1];
                            if (requestedIndex > 9) requestedIndex = 0;

                            uint16_t eepromAddr = 0x0140 + (requestedIndex * 64);

                            memset(txBuffer, 0, sizeof(txBuffer));
                            txBuffer[0] = 0x03;
                            txBuffer[1] = BODY_RX_LEN; // 92
                            txBuffer[2] = calculateCRC(txBuffer, 2);
                            txBuffer[3] = transID;
                            txBuffer[4] = requestedIndex;

                            // Read 64-byte log payload into bytes 5..68
                            for (uint16_t offset = 0; offset < 64; offset += 4) {
                                i2c_eeprom_read_long(0xA0, eepromAddr + offset, &txBuffer[5 + offset]);
                            }

                            txBuffer[94] = calculateCRC(&txBuffer[3], 91);
                        }

                        // Shift processed body bytes out of rxBuffer
                        __disable_irq();
                        memmove(rxBuffer, &rxBuffer[expectedBodyLen], rxCount - expectedBodyLen);
                        rxCount -= expectedBodyLen;
                        __enable_irq();

                        // =========================================================
                        // RESPONSE TRANSMISSION
                        // =========================================================
                        // Add a 200us guard delay so STM32 can arm UART RX before XMC transmits
                        for (volatile uint32_t i = 0; i < 3000U; ++i) {
                            __NOP();
                        }

                        if (isStagingCommand) {
                            // Standardized 95-byte ACK frame for STM32 TCP Bridge compatibility
                            memset(txBuffer, 0, sizeof(txBuffer));
                            txBuffer[0] = 0x00;
                            txBuffer[1] = BODY_RX_LEN;                 // 92 Bytes (0x5C)
                            txBuffer[2] = calculateCRC(txBuffer, 2);   // 0x5C

                            txBuffer[3] = transID;                     // Echoed Transaction ID
                            txBuffer[4] = 0x55;                        // Status OK ACK Flag

                            txBuffer[94] = calculateCRC(&txBuffer[3], 91);

                            // Send full 95-byte packet so STM32 Bridge forwards it over TCP
                            transmitBuffer(txBuffer, 95);
                        } else if (isReadbackCommand) {
                            // 263-byte Staging Readback frame
                            transmitBuffer(txBuffer, 263);
                        } else if (isDiagLogRequest) {
                            // 95-byte Diagnostic Log frame
                            transmitBuffer(txBuffer, 95);
                        } else {
                            // 95-byte Real-time Telemetry frame (3B Header + 92B Body)
                            memset(txBuffer, 0, sizeof(txBuffer));
                            txBuffer[0] = 0x00;
                            txBuffer[1] = BODY_RX_LEN; // 92
                            txBuffer[2] = calculateCRC(txBuffer, 2); // 0x5C

                            txBuffer[3] = transID; // Echoed transaction ID
                            txBuffer[4] = wptCtrl.PWM_EN_STATE ? 0x01 : 0x00;

                            txBuffer[35] = static_cast<uint8_t>((wptCtrl.WPT_OPCODE >> 8) & 0xFF);
                            txBuffer[36] = static_cast<uint8_t>(wptCtrl.WPT_OPCODE & 0xFF);

                            int32_t vInScaled = static_cast<int32_t>(telemetry.vIn * 1000.0f);
                            txBuffer[69] = static_cast<uint8_t>((vInScaled >> 24) & 0xFF);
                            txBuffer[70] = static_cast<uint8_t>((vInScaled >> 16) & 0xFF);
                            txBuffer[71] = static_cast<uint8_t>((vInScaled >> 8) & 0xFF);
                            txBuffer[72] = static_cast<uint8_t>(vInScaled & 0xFF);

                            int32_t iInScaled = static_cast<int32_t>(telemetry.iIn * 1000.0f);
                            txBuffer[73] = static_cast<uint8_t>((iInScaled >> 24) & 0xFF);
                            txBuffer[74] = static_cast<uint8_t>((iInScaled >> 16) & 0xFF);
                            txBuffer[75] = static_cast<uint8_t>((iInScaled >> 8) & 0xFF);
                            txBuffer[76] = static_cast<uint8_t>(iInScaled & 0xFF);

                            int32_t iOutScaled = static_cast<int32_t>(telemetry.iOut * 1000.0f);
                            txBuffer[77] = static_cast<uint8_t>((iOutScaled >> 24) & 0xFF);
                            txBuffer[78] = static_cast<uint8_t>((iOutScaled >> 16) & 0xFF);
                            txBuffer[79] = static_cast<uint8_t>((iOutScaled >> 8) & 0xFF);
                            txBuffer[80] = static_cast<uint8_t>(iOutScaled & 0xFF);

                            int32_t tempScaled = static_cast<int32_t>(telemetry.tempC * 10.0f);
                            txBuffer[81] = static_cast<uint8_t>((tempScaled >> 24) & 0xFF);
                            txBuffer[82] = static_cast<uint8_t>((tempScaled >> 16) & 0xFF);
                            txBuffer[83] = static_cast<uint8_t>((tempScaled >> 8) & 0xFF);
                            txBuffer[84] = static_cast<uint8_t>(tempScaled & 0xFF);

                            int32_t pInScaled = static_cast<int32_t>((telemetry.vIn * telemetry.iIn) * 1000.0f);
                            txBuffer[85] = static_cast<uint8_t>((pInScaled >> 24) & 0xFF);
                            txBuffer[86] = static_cast<uint8_t>((pInScaled >> 16) & 0xFF);
                            txBuffer[87] = static_cast<uint8_t>((pInScaled >> 8) & 0xFF);
                            txBuffer[88] = static_cast<uint8_t>(pInScaled & 0xFF);

                            uint32_t resFreqHz = static_cast<uint32_t>(wptCtrl.actualResonantFreq * 1000.0f);
                            txBuffer[89] = static_cast<uint8_t>((resFreqHz >> 24) & 0xFF);
                            txBuffer[90] = static_cast<uint8_t>((resFreqHz >> 16) & 0xFF);
                            txBuffer[91] = static_cast<uint8_t>((resFreqHz >> 8)  & 0xFF);
                            txBuffer[92] = static_cast<uint8_t>(resFreqHz & 0xFF);

                            txBuffer[93] = static_cast<uint8_t>(wptCtrl.fault);

                            txBuffer[94] = calculateCRC(&txBuffer[3], 91);

                            transmitSplitBuffer(txBuffer, 95);
                        }

                        lastActivityTimer = currentMs;
                        currentState = UART_STATE::RECEIVE_LEN;
                        stateChanged = true;
                    } else {
                        resetBus(currentMs);
                    }
                }
                break;
            }

            default:
                break;
        }
    } while (stateChanged);
}