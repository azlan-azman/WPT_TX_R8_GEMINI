#include "XMC1300.h"
#include "xmc_scu.h"
#include "xmc_wdt.h"
#include "xmc_gpio.h"
#include "xmc_vadc.h"

#include "W5500_SPI.hpp"
#include "SPIProtocolController.hpp"
#include "PinDefine.h"
#include "InverterPWM.hpp"
#include "AnalogADC.hpp"
#include "AnalogACMP.hpp"
#include "HardwareERU.hpp"
#include "FrequencyCapture.hpp"
#include "I2CController.hpp"
#include "I2C_LCD.h"
#include "LCDOutput.h"
#include "PowerControl.hpp"
#include "WPTController.hpp"
#include "ButtonController.h"
#include <DAVE.h>
#include "ADCData.h"

static volatile uint32_t g_systemTicks = 0;

extern "C" {
void HardFault_Handler(void) { while (1) { __NOP(); } }
void Default_Handler(void)   { while (1) { __NOP(); } }
void SysTick_Handler(void)   { g_systemTicks++; }
}

uint32_t getTickMs(void) { return g_systemTicks; }

W5500SPI                 w5500;
SPIProtocolController    spiComm(&w5500);

InverterPWM              inverter;
AnalogADC                adc;
AnalogACMP               acmp;
HardwareERU              hardwareERU;
FrequencyCapture         frequencyCapture;
I2CController            displayBus;
PowerControl             controller;
ButtonController         buttonController;

WPT_Controller           WPT_Controller1;
Telemetry1               currentTelemetry = {};
I2CLCD                   LCD1;
LCDOutput                lcdOutput;

ADCData                   ADCData1;

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

    XMC_Delay(10);
	DAVE_STATUS_t status;
	status = DAVE_Init(); /* Initialization of DAVE APPs  */
	XMC_Delay(10);

    // 1. Initialize Analog & Inverter Drivers Before Starting Network
    //adc.init();
    ADC_MEASUREMENT_StartConversion(&ADC_MEASUREMENT_0);
	ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);
	XMC_Delay(10);
    acmp.init();
    hardwareERU.init();
    frequencyCapture.init();
    displayBus.init();
    inverter.init();

    WPT_Controller1.fanMonitor.init();
    WPT_Controller1.loadEEPROM();
    WPT_Controller1.fault = static_cast<FaultReason>(0x00);
    inverter.setFrequency(WPT_Controller1.Resonant_Freq);

    // 2. Start W5500 Network Controller
    spiComm.init();

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

        // Continuous ADC Read & Filter (Runs every loop iteration, matching DAVE's ADCData)
        bool dip3 = DIP3_READ();
        currentTelemetry = ADCData1.readAndProcess1(dip3);

        // Network communication
        spiComm.process(now, inverter, WPT_Controller1, currentTelemetry);

        if (WPT_Controller1.I2C_RST_required) {
            LCD1.reinit();
        }

        // 1 ms Periodic Control & Power Ramp Loop (Requires constant dt = 1ms)
        if (now - lastFastTick >= 1U) {
            lastFastTick = now;

            WPT_Controller1.pushTrend(currentTelemetry);
            WPT_Controller1.update(inverter, controller, currentTelemetry);
        }

        // 10 ms Button Polling Loop
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

        // 1000 ms Inactivity Countdown Loop
        if (now - lastCountdownTick >= 1000U) {
            lastCountdownTick = now;

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
        }

        // 25 ms LCD Display Loop
        if (now - lastLcdTick >= 25U) {
            lastLcdTick = now;
            WPT_Controller1.rowToUpdate = (WPT_Controller1.rowToUpdate + 1) % 4;
            lcdOutput.LcdOutput();
        }

        // 500 ms LED Heartbeat Loop
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