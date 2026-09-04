#pragma once

#include "XMC1300.h"

extern "C" {
#include "xmc_common.h"
#include "xmc_gpio.h"
#include "xmc_vadc.h"

void XMC_Delay(uint32_t milliseconds);
}

uint32_t getTickMs(void);

// =============================================================================
// 1. HARDWARE INVERTER CONTROL & PWM PINS
// =============================================================================
#define PWM_EN_PORT             XMC_GPIO_PORT0
#define PWM_EN_PIN              8

// Leg A (CCU40 CC40 Output)
#define PWM_LEG_A_PORT          XMC_GPIO_PORT0
#define PWM_LEG_A_PIN           6
#define PWM_LEG_A_AF            XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT4

// Leg B (CCU80 CC81 Output)
#define PWM_LEG_B_PORT          XMC_GPIO_PORT0
#define PWM_LEG_B_PIN           7
#define PWM_LEG_B_AF            XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

// =============================================================================
// 2. USIC0 CHANNEL 1: I2C DISPLAY & EEPROM PINS
// =============================================================================
#define I2C_SDA_PORT            XMC_GPIO_PORT2
#define I2C_SDA_PIN             10
#define I2C_SDA_AF              XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT7

#define I2C_SCL_PORT            XMC_GPIO_PORT2
#define I2C_SCL_PIN             11
#define I2C_SCL_AF              XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT6

// =============================================================================
// 3. USIC0 CHANNEL 0: UART TELEMETRY PINS
// =============================================================================
#define UART_TX_PORT            XMC_GPIO_PORT1
#define UART_TX_PIN             5
#define UART_TX_AF              XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2

#define UART_RX_PORT            XMC_GPIO_PORT1
#define UART_RX_PIN             4

// =============================================================================
// 4. USER INTERFACE BUTTONS & LEDS
// =============================================================================
#define BTN1_PORT               XMC_GPIO_PORT0
#define BTN1_PIN                5
#define BTN1_READ()             XMC_GPIO_GetInput(BTN1_PORT, BTN1_PIN)
#define BTN1                    (!BTN1_READ())

#define BTN2_PORT               XMC_GPIO_PORT0
#define BTN2_PIN                4
#define BTN2_READ()             XMC_GPIO_GetInput(BTN2_PORT, BTN2_PIN)
#define BTN2                    (!BTN2_READ())

#define BTN3_PORT               XMC_GPIO_PORT0
#define BTN3_PIN                3
#define BTN3_READ()             XMC_GPIO_GetInput(BTN3_PORT, BTN3_PIN)
#define BTN3                    (!BTN3_READ())

#define LED0_PORT               XMC_GPIO_PORT1
#define LED0_PIN                1

#define LED1_PORT               XMC_GPIO_PORT1
#define LED1_PIN                2

#define LED2_PORT               XMC_GPIO_PORT1
#define LED2_PIN                3

#define DIP3_PORT               XMC_GPIO_PORT0
#define DIP3_PIN                1
#define DIP3_READ()             XMC_GPIO_GetInput(DIP3_PORT, DIP3_PIN)

#define DIP2_PORT               XMC_GPIO_PORT0
#define DIP2_PIN                0
#define DIP2_READ()             XMC_GPIO_GetInput(DIP2_PORT, DIP2_PIN)

// =============================================================================
// 5. ACMP1 / VADC SHARED ANALOG PINS (ANALOG_IO_0 = P2.7, ANALOG_IO_1 = P2.6)
// =============================================================================
#define ACMP1_INP_PORT          XMC_GPIO_PORT2
#define ACMP1_INP_PIN           7

#define ACMP1_INN_PORT          XMC_GPIO_PORT2
#define ACMP1_INN_PIN           6

// =============================================================================
// HARDWARE PIN INITIALIZATION HELPER
// =============================================================================
inline void Hardware_InitPins(void) {
    // Enable Outputs
    XMC_GPIO_SetMode(PWM_EN_PORT, PWM_EN_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetMode(LED0_PORT, LED0_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetMode(LED1_PORT, LED1_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetMode(LED2_PORT, LED2_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);

    // PWM Pin Modes
    XMC_GPIO_SetMode(PWM_LEG_A_PORT, PWM_LEG_A_PIN, PWM_LEG_A_AF);
    XMC_GPIO_SetMode(PWM_LEG_B_PORT, PWM_LEG_B_PIN, PWM_LEG_B_AF);

    // I2C Pin Modes (USIC0_CH1)
    XMC_GPIO_SetMode(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_AF);
    XMC_GPIO_SetMode(I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_AF);

    // UART Pin Modes (USIC0_CH0)
    XMC_GPIO_SetMode(UART_TX_PORT, UART_TX_PIN, UART_TX_AF);
    XMC_GPIO_SetMode(UART_RX_PORT, UART_RX_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);

    // Button Inputs
    XMC_GPIO_SetMode(BTN1_PORT, BTN1_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(BTN2_PORT, BTN2_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(BTN3_PORT, BTN3_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);

    // DIP Switch Inputs
    XMC_GPIO_SetMode(DIP2_PORT, DIP2_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(DIP3_PORT, DIP3_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);

    // ACMP1 Positive Input / Vrefin (P2.7 = ANALOG_IO_0)
    XMC_GPIO_SetMode(ACMP1_INP_PORT, ACMP1_INP_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    ACMP1_INP_PORT->HWSEL = (ACMP1_INP_PORT->HWSEL & ~(0x3U << (2U * ACMP1_INP_PIN))) |
                            (0x1U << (2U * ACMP1_INP_PIN));

    // ACMP1 Negative Input / Iout (P2.6 = ANALOG_IO_1)
    XMC_GPIO_SetMode(ACMP1_INN_PORT, ACMP1_INN_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    ACMP1_INN_PORT->HWSEL = (ACMP1_INN_PORT->HWSEL & ~(0x3U << (2U * ACMP1_INN_PIN))) |
                            (0x1U << (2U * ACMP1_INN_PIN));

    // Disable Digital Input Path on P2.6 and P2.7 for Pure Analog Mode
    XMC_GPIO_PORT2->PDISC |= (1U << ACMP1_INP_PIN) | (1U << ACMP1_INN_PIN);
}