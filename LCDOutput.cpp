#include "LCDOutput.h"
#include "I2C_LCD.h"
#include "WPTController.hpp"
#include "AnalogADC.hpp"
#include "PinDefine.h"
#include <stdio.h>
#include <string.h>

#define FIRMWARE_VERSION ("GEMINI")

extern WPT_Controller WPT_Controller1;
extern Telemetry      currentTelemetry;
extern I2CLCD         LCD1;

static char PreviousRows[4][21] = { {0}, {0}, {0}, {0} };
static uint8_t lastRenderedMenu = 0xFF;

void LCDOutput_ClearCache(void) {
    memset(PreviousRows, 0, sizeof(PreviousRows));
    lastRenderedMenu = 0xFF;
}

static const char* const MenuStrings[4][4][1] = {
    { { "Stratus Automation" }, { "WPT TX MODE" }, { nullptr }, { nullptr } },
    { { "Auto power up in" }, { nullptr }, { "BTN 3 to cancel" }, { nullptr } },
    { { nullptr }, { nullptr }, { nullptr }, { nullptr } },
    { { "      Warning !     " }, { "Following screen is " }, { "for trained personal" }, { "        only!       " } }
};

LCDOutput::LCDOutput() {}
LCDOutput::~LCDOutput() {}

void LCDOutput::LcdOutput(void) {
    Row1[0] = '\0';

    uint8_t menu = WPT_Controller1.DisplayMenuID;
    uint8_t row  = WPT_Controller1.rowToUpdate;

    // Force cache clear whenever switching menus to guarantee screen redraw
    if (menu != lastRenderedMenu) {
        memset(PreviousRows, 0, sizeof(PreviousRows));
        lastRenderedMenu = menu;
    }

    uint8_t opt = WPT_Controller1.In_System_Menu ? 0 : WPT_Controller1.DisplayOption;

    GetKey = (menu * 100) + (row * 10) + opt;

    int iIn_int  = static_cast<int>(currentTelemetry.iIn);
    int iIn_dec  = static_cast<int>((currentTelemetry.iIn - iIn_int) * 100);
    int iOut_int = static_cast<int>(currentTelemetry.iOut);
    int iOut_dec = static_cast<int>((currentTelemetry.iOut - iOut_int) * 100);
    int vIn_int  = static_cast<int>(currentTelemetry.vIn);
    int vIn_dec  = static_cast<int>((currentTelemetry.vIn - vIn_int) * 100);
    int temp_int = static_cast<int>(currentTelemetry.tempC);
    int rf_int   = static_cast<int>(WPT_Controller1.actualResonantFreq);

    switch (GetKey) {
        case 110:
            sprintf(Row1, "%lu", static_cast<unsigned long>(WPT_Controller1.tempSecond));
            break;

        // Menu 2: Row 0 (OF:xxx.xx   E :x      )
        case 200:
        case 201:
        case 202: {
            char col1[11], col2[11];
            int of_int = static_cast<int>(WPT_Controller1.Resonant_Freq / 1000.0f);
            int of_dec = static_cast<int>((WPT_Controller1.Resonant_Freq - (of_int * 1000.0f)) / 10.0f);
            if (of_dec < 0) of_dec = 0;

            snprintf(col1, sizeof(col1), "%-2s:%d.%02d", "OF", of_int, of_dec);
            snprintf(col2, sizeof(col2), "%-2s:%d", "E", (WPT_Controller1.PWM_EN_STATE ? 1 : 0));
            sprintf(Row1, "%-10s%-10s", col1, col2);
            break;
        }

        // Menu 2: Row 1 (D :xxxx     Ii:x.xx   )
        case 210:
        case 211:
        case 212: {
            char col1[11], col2[11];
            snprintf(col1, sizeof(col1), "%-2s:%d", "D", WPT_Controller1.PWMDutyCycle);
            snprintf(col2, sizeof(col2), "%-2s:%d.%02d", "Ii", iIn_int, (iIn_dec < 0 ? -iIn_dec : iIn_dec));
            sprintf(Row1, "%-10s%-10s", col1, col2);
            break;
        }

        // Menu 2: Row 2 (Vi:xx.xx    T :xxC    )
        case 220:
        case 221: {
            char col1[11], col2[11];
            snprintf(col1, sizeof(col1), "%-2s:%d.%02d", "Vi", vIn_int, (vIn_dec < 0 ? -vIn_dec : vIn_dec));
            snprintf(col2, sizeof(col2), "%-2s:%dC", "T", temp_int);
            sprintf(Row1, "%-10s%-10s", col1, col2);
            break;
        }
        case 222:
            snprintf(Row1, sizeof(Row1), "Runtime hour:");
            break;

        // Menu 2: Row 3 (Op:0xXXX    ST:OK     )
        case 230: {
            char col1[11], col2[11];
            snprintf(col1, sizeof(col1), "%-2s:0x%03X", "Op", WPT_Controller1.WPT_OPCODE);
            snprintf(col2, sizeof(col2), "%-2s:%d.%02d", "Io", iOut_int, (iOut_dec < 0 ? -iOut_dec : iOut_dec));
            sprintf(Row1, "%-10s%-10s", col1, col2);
            break;
        }
        case 231: {
            char col1[11], col2[11];
            snprintf(col1, sizeof(col1), "%-2s:%d", "RF", rf_int);
            snprintf(col2, sizeof(col2), "%-2s:%s", "ST", (WPT_Controller1.ResonantFreq_OK ? "OK" : "  "));
            sprintf(Row1, "%-10s%-10s", col1, col2);
            break;
        }
        case 232:
            sprintf(Row1, "%lu h", static_cast<unsigned long>(getTickMs() / 3600000UL));
            break;

        // --- System Menus (Reverted to Original Formatting) ---
        case 400:
            snprintf(Row1, sizeof(Row1), WPT_Controller1.AutoTune ? "Mode : Calibrate  " : "Mode : Normal     ");
            break;
        case 500:
            sprintf(Row1, "Ii :%d.%02d ", static_cast<int>(WPT_Controller1.TX_Max_Iout), static_cast<int>((WPT_Controller1.TX_Max_Iout - static_cast<int>(WPT_Controller1.TX_Max_Iout)) * 100));
            break;
        case 510:
            sprintf(Row1, "Ii :%d.%02d ", iIn_int, (iIn_dec < 0 ? -iIn_dec : iIn_dec));
            break;

        case 600:
        case 700: {
            int khz_int = static_cast<int>(WPT_Controller1.ee_Resonant_Freq / 1000.0f);
            int khz_dec = static_cast<int>((WPT_Controller1.ee_Resonant_Freq - (khz_int * 1000.0f)) / 10.0f);
            if (khz_dec < 0) khz_dec = 0;
            sprintf(Row1, "OF :%d.%02dkhz", khz_int, khz_dec);
            break;
        }

        case 610:
            snprintf(Row1, sizeof(Row1), "+/- 0.1kHz");
            break;
        case 710:
            snprintf(Row1, sizeof(Row1), "+/- 0.01kHz");
            break;
        case 800:
            sprintf(Row1, "Timer : %d    ", static_cast<int>(WPT_Controller1.AutoStarTimer));
            break;
        case 900:
        	sprintf(Row1, "Io :%6.2f ", WPT_Controller1.Iout_Limit);
            break;
        case 1000:
            sprintf(Row1, "Duty_Cycle : %d", WPT_Controller1.eePWMDutyCycle);
            break;
        case 1100:
        case 1101:
            snprintf(Row1, sizeof(Row1), "Firmware Version :");
            break;
        case 1110:
        case 1111:
            snprintf(Row1, sizeof(Row1), FIRMWARE_VERSION);
            break;
        case 1120:
        case 1121:
            snprintf(Row1, sizeof(Row1), WPT_Controller1.DIP3_State ? "48V" : "27V");
            break;
        default:
            Row1[0] = '\0';
            break;
    }

    const char* targetStr = nullptr;
    if (Row1[0] != '\0') {
        targetStr = Row1;
    } else if (menu < 4 && row < 4 && opt < 1 && MenuStrings[menu][row][opt] != nullptr) {
        targetStr = MenuStrings[menu][row][opt];
    }

    if (targetStr == nullptr) {
        targetStr = "";
    }

    char paddedBuffer[21];
    memset(paddedBuffer, ' ', 20);
    paddedBuffer[20] = '\0';

    size_t len = strlen(targetStr);
    if (len > 20) len = 20;
    memcpy(paddedBuffer, targetStr, len);

    if (strcmp(PreviousRows[row], paddedBuffer) != 0) {
        memcpy(PreviousRows[row], paddedBuffer, 21);

        LCD1.setCursor(0, row);
        LCD1.write(paddedBuffer);
    }
}
