#include "XMC1300.h"
#include "xmc_scu.h"
#include "xmc_wdt.h"
#include "xmc_gpio.h"

#include "PinDefine.h"
#include "InverterPWM.hpp"
#include "AnalogADC.hpp"
#include "AnalogACMP.hpp"
#include "HardwareERU.hpp"
#include "FrequencyCapture.hpp"
#include "UARTController.hpp"
#include "UARTProtocolController.hpp"
#include "I2CController.hpp"
#include "I2C_LCD.h"
#include "LCDOutput.h"
#include "PowerControl.hpp"
#include "WPTController.hpp"
#include "ButtonController.h"

static volatile uint32_t g_systemTicks = 0;

extern "C" {
void HardFault_Handler(void) { while (1) { __NOP(); } }
void Default_Handler(void)   { while (1) { __NOP(); } }
void SysTick_Handler(void)   { g_systemTicks++; }
}

uint32_t getTickMs(void) { return g_systemTicks; }

// Global Driver Instances
InverterPWM              inverter;
AnalogADC                adc;
AnalogACMP               acmp;
HardwareERU              hardwareERU;
FrequencyCapture         frequencyCapture;
UARTController           uartHardware;
UARTProtocolController   uartComm;
I2CController            displayBus;
PowerControl             controller;
ButtonController         buttonController;

WPT_Controller           WPT_Controller1;
Telemetry                currentTelemetry;
I2CLCD                   LCD1;
LCDOutput                lcdOutput;

// --- USIC0 RX Hardware Interrupt (IRQ 9) ---
extern "C" {
void IRQ9_Handler(void) {
    while (!XMC_USIC_CH_RXFIFO_IsEmpty(XMC_UART0_CH0)) {
        uint8_t byte = static_cast<uint8_t>(XMC_USIC_CH_RXFIFO_GetData(XMC_UART0_CH0));
        uartComm.pushRxByte(byte);
    }
}
void USIC0_0_IRQHandler(void) { IRQ9_Handler(); }
}

int main(void) {
    XMC_SCU_CLOCK_CONFIG_t clock_config = {};
    clock_config.pclk_src = XMC_SCU_CLOCK_PCLKSRC_DOUBLE_MCLK;
    clock_config.rtc_src  = XMC_SCU_CLOCK_RTCCLKSRC_DCO2;
    clock_config.fdiv     = 0U;
    clock_config.idiv     = 1U;
    XMC_SCU_CLOCK_Init(&clock_config);
    SystemCoreClockUpdate();

    SysTick_Config(SystemCoreClock / 1000U);
    __enable_irq();

    XMC_SCU_CLOCK_UngatePeripheralClock(XMC_SCU_PERIPHERAL_CLOCK_CCU40);
    XMC_SCU_CLOCK_UngatePeripheralClock(XMC_SCU_PERIPHERAL_CLOCK_CCU80);
    XMC_SCU_CLOCK_UngatePeripheralClock(XMC_SCU_PERIPHERAL_CLOCK_USIC0);
    XMC_SCU_CLOCK_UngatePeripheralClock(XMC_SCU_PERIPHERAL_CLOCK_VADC);

    Hardware_InitPins();

    inverter.init();
    adc.init();
    acmp.init();
    hardwareERU.init();
    frequencyCapture.init();
    uartHardware.init();
    displayBus.init();

	// Initialize Fan Monitor Capture Slices (P0.12 / P0.13)
	WPT_Controller1.fanMonitor.init();

    WPT_Controller1.loadEEPROM();
    inverter.setFrequency(WPT_Controller1.Resonant_Freq);

    LCD1.LCDbegin(0x40);

    WPT_Controller1.PWM_EN_STATE = false;
    WPT_Controller1.DisplayMenuID = 0;

    uint32_t lastFastTick      = getTickMs();
    uint32_t lastButtonTick    = getTickMs();
    uint32_t lastLcdTick       = getTickMs();
    uint32_t lastCountdownTick = getTickMs();
    uint32_t lastSlowTick      = getTickMs();

    while (1) {
        uint32_t now = getTickMs();

        uartComm.process(now, inverter, WPT_Controller1, currentTelemetry);

        if (WPT_Controller1.I2C_RST_required) {
            LCD1.reinit();
        }

        // 1 ms Control Loop
        if (now - lastFastTick >= 1U) {
            lastFastTick = now;

            uint16_t rawTemp = static_cast<uint16_t>(VADC_G0->RES[7]  & 0x0FFF);
            uint16_t rawIout = static_cast<uint16_t>(VADC_G0->RES[9]  & 0x0FFF);
            uint16_t rawVin  = static_cast<uint16_t>(VADC_G1->RES[11] & 0x0FFF);
            uint16_t rawVref = static_cast<uint16_t>(VADC_G1->RES[10] & 0x0FFF);
            uint16_t rawIin  = static_cast<uint16_t>(VADC_G1->RES[4]  & 0x0FFF);

            bool dip3 = DIP3_READ();
            currentTelemetry = adc.process(rawVin, rawIin, rawIout, rawVref, rawTemp, dip3);

            // Feed pre-fault RAM trend buffer
            WPT_Controller1.pushTrend(currentTelemetry);

            WPT_Controller1.update(inverter, controller, currentTelemetry);
        }

        // 10 ms Button Loop
        if (now - lastButtonTick >= 10U) {
            lastButtonTick = now;
            WPT_Controller1.DIP3_State = DIP3_READ();
            WPT_Controller1.DIP2_State = DIP2_READ();

            if (WPT_Controller1.DisplayMenuID == 2) {
                if (WPT_Controller1.DIP2_State) {
                    WPT_Controller1.DisplayOption = 2;
                } else if (WPT_Controller1.DisplayOption == 2) {
                    WPT_Controller1.DisplayOption = 0;
                }
                if (WPT_Controller1.AutoTune) {
                    WPT_Controller1.DisplayOption = 1;
                } else if (WPT_Controller1.DisplayOption == 1) {
                    WPT_Controller1.DisplayOption = 0;
                }
            }

            buttonController.ButtonPress();
        }

        // 1000 ms Countdown Loop
        if (now - lastCountdownTick >= 1000U) {
            lastCountdownTick = now;

            // Auto-Start Countdown
            if (WPT_Controller1.DisplayMenuID == 1 && !WPT_Controller1.AutoStartDone) {
                if (WPT_Controller1.tempSecond > 0) {
                    WPT_Controller1.tempSecond--;
                }

                if (WPT_Controller1.tempSecond == 0) {
                    WPT_Controller1.AutoStartDone = true;
                    WPT_Controller1.PWM_EN_STATE  = true;
                    WPT_Controller1.exitMenu();
                    LCD1.clear();
                }
            }
            // System Menu Inactivity Timeout Check
			/*else if (WPT_Controller1.In_System_Menu == 1) {
				if (WPT_Controller1.tempSecond > 0) {
					WPT_Controller1.tempSecond--;
				}

				// Timer expired without reaching the final page -> Discard changes
				if (WPT_Controller1.tempSecond == 0) {
					WPT_Controller1.discardMenuEdits(); // Revert RAM to EEPROM
					LCD1.clear();
				}
			}*/
        }

        // 25 ms LCD Loop
        if (now - lastLcdTick >= 25U) {
            lastLcdTick = now;
            WPT_Controller1.rowToUpdate = (WPT_Controller1.rowToUpdate + 1) % 4;
            lcdOutput.LcdOutput();
        }

        // 500 ms Heartbeat Loop
        if (now - lastSlowTick >= 500U) {
            lastSlowTick = now;
            XMC_GPIO_ToggleOutput(LED2_PORT, LED2_PIN);

            if (WPT_Controller1.DisplayMenuID == 0 && now >= 2000U) {
                if (!WPT_Controller1.AutoStartDone && WPT_Controller1.AutoStarTimer > 0) {
                    WPT_Controller1.DisplayMenuID = 1;
                    WPT_Controller1.tempSecond    = WPT_Controller1.AutoStarTimer;
                } else {
                    WPT_Controller1.DisplayMenuID = 2;
                }
                LCD1.clear();
            }
        }
    }
}
