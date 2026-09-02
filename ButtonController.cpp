#include "ButtonController.h"
#include "WPTController.hpp"
#include "InverterPWM.hpp"
#include "eeprom.h"
#include "I2C_LCD.h"
#include "PinDefine.h"

extern WPT_Controller WPT_Controller1;
extern InverterPWM    inverter;
extern I2CLCD         LCD1;

ButtonController::ButtonController()
    : Enable_BTN_Press(1), Next_Menu_ID(0), ButtonPress_SM(0), BTN_Timer(0),
      Button1Count(0), Button1Reset(0), Button2Count(0), Button2Reset(0) {}

ButtonController::~ButtonController() {}

void ButtonController::BTN_1_Press(void) {
    WPT_Controller1.DisplayOption = 0; // Clear DisplayOption on menu step

    // Force step pointer to 0 if entering from outside the menu
    if (WPT_Controller1.In_System_Menu == 0) {
        Next_Menu_ID = 0;
    }

    switch (Next_Menu_ID) {
        case 0:
        case 10:  WPT_Controller1.enterMenu(3);  Next_Menu_ID = 20; break;
        case 20:  WPT_Controller1.DisplayMenuID = 4;  Next_Menu_ID = 40; break;
        case 40:  WPT_Controller1.DisplayMenuID = 5;  Next_Menu_ID = 60; break;
        case 60:  WPT_Controller1.DisplayMenuID = 6;  Next_Menu_ID = 70; break;
        case 70:  WPT_Controller1.DisplayMenuID = 7;  Next_Menu_ID = 80; break;
        case 80:  WPT_Controller1.DisplayMenuID = 8;  Next_Menu_ID = 90; break;
        case 90:  WPT_Controller1.DisplayMenuID = 9;  Next_Menu_ID = 100; break;
        case 100: WPT_Controller1.DisplayMenuID = 10; Next_Menu_ID = 110; break;
        case 110: WPT_Controller1.DisplayMenuID = 11; Next_Menu_ID = 120; break;
        case 120:
            WPT_Controller1.confirmAndSaveMenu(inverter);
            Next_Menu_ID = 0; // Reset local navigation step
            break;
    }
}

void ButtonController::BTN_2_Press(void) {
    switch (WPT_Controller1.DisplayMenuID) {
        case 2:
            if (!WPT_Controller1.PWM_EN_STATE) WPT_Controller1.PWM_EN_STATE = true;
            break;
        case 4:
            if (WPT_Controller1.In_System_Menu) WPT_Controller1.AutoTune = 1;
            break;
        case 5:
            if (WPT_Controller1.In_System_Menu && WPT_Controller1.TX_Max_Iout <= WPT_Controller1.TX_MAX_IOUT_UPPER_LIMIT - 0.1f) {
                WPT_Controller1.TX_Max_Iout += 0.1f;
                WPT_Controller1.actual_TX_Max_Iout = WPT_Controller1.TX_Max_Iout;
            }
            break;
        case 6:
        case 7:
            if (WPT_Controller1.In_System_Menu) {
                float step = (WPT_Controller1.DisplayMenuID == 6) ? 100.0f : 10.0f;
                if (WPT_Controller1.ee_Resonant_Freq <= WPT_Controller1.MAX_RESONANT_FREQ - step) {
                    WPT_Controller1.ee_Resonant_Freq += step;
                }
                WPT_Controller1.Resonant_Freq = WPT_Controller1.ee_Resonant_Freq;
                inverter.setFrequency(WPT_Controller1.Resonant_Freq);
            }
            break;
        case 8:
            if (WPT_Controller1.In_System_Menu && WPT_Controller1.AutoStarTimer < 300) WPT_Controller1.AutoStarTimer += 1;
            break;
        case 9:
            if (WPT_Controller1.In_System_Menu && WPT_Controller1.Iout_Limit < 20.0f) WPT_Controller1.Iout_Limit += 0.1f;
            break;
        case 10:
            if (WPT_Controller1.In_System_Menu && WPT_Controller1.eePWMDutyCycle < 5000) WPT_Controller1.eePWMDutyCycle += 100;
            break;
    }
}

void ButtonController::BTN_3_Press(void) {
    switch (WPT_Controller1.DisplayMenuID) {
        case 1:
            WPT_Controller1.AutoStartDone = true;
            WPT_Controller1.PWM_EN_STATE  = false;
            WPT_Controller1.DisplayMenuID = 2;
            LCD1.clear();
            break;

        case 2:
            if (WPT_Controller1.PWM_EN_STATE) {
                WPT_Controller1.PWM_EN_STATE = false;
                WPT_Controller1.LED_R_Interval = 0;
            }
            break;
        case 4:
            if (WPT_Controller1.In_System_Menu) WPT_Controller1.AutoTune = 0;
            break;
        case 5:
            if (WPT_Controller1.In_System_Menu && WPT_Controller1.TX_Max_Iout >= WPT_Controller1.TX_MAX_IOUT_LOWER_LIMIT + 0.1f) {
                WPT_Controller1.TX_Max_Iout -= 0.1f;
                WPT_Controller1.actual_TX_Max_Iout = WPT_Controller1.TX_Max_Iout;
            }
            break;
        case 6:
        case 7:
            if (WPT_Controller1.In_System_Menu) {
                float step = (WPT_Controller1.DisplayMenuID == 6) ? 100.0f : 10.0f;
                if (WPT_Controller1.ee_Resonant_Freq >= WPT_Controller1.MIN_RESONANT_FREQ + step) {
                    WPT_Controller1.ee_Resonant_Freq -= step;
                }
                WPT_Controller1.Resonant_Freq = WPT_Controller1.ee_Resonant_Freq;
                inverter.setFrequency(WPT_Controller1.Resonant_Freq);
            }
            break;
        case 8:
            if (WPT_Controller1.In_System_Menu && WPT_Controller1.AutoStarTimer > 1) WPT_Controller1.AutoStarTimer -= 1;
            break;
        case 9:
            if (WPT_Controller1.In_System_Menu && WPT_Controller1.Iout_Limit > -20.0f) WPT_Controller1.Iout_Limit -= 0.1f;
            break;
        case 10:
            if (WPT_Controller1.In_System_Menu && WPT_Controller1.eePWMDutyCycle > 1500) WPT_Controller1.eePWMDutyCycle -= 100;
            break;
    }
}

void ButtonController::ButtonPress(void) {
    if (!Enable_BTN_Press) return;

    uint32_t now = getTickMs();

    if (BTN1 || BTN2 || BTN3) {
        WPT_Controller1.refreshInactivityTimer();
        switch (ButtonPress_SM) {
            case 0:
                BTN_Timer = now;
                ButtonPress_SM = 1;
                break;
            case 1:
                if (now - BTN_Timer > 100) {
                    if (BTN2 && BTN3 && Next_Menu_ID == 20) {
                        ButtonPress_SM = 2;
                    } else if (BTN1) {
                        if (WPT_Controller1.In_System_Menu == 0) {
                            if (now - BTN_Timer > 5000) { BTN_1_Press(); BTN_Timer = now; ButtonPress_SM = 3; }
                        } else if (Next_Menu_ID != 20) { BTN_1_Press(); BTN_Timer = now; ButtonPress_SM = 3; }
                    } else if (BTN2) {
                        BTN_2_Press(); BTN_Timer = now; ButtonPress_SM = 3;
                    } else if (BTN3) {
                        BTN_3_Press(); BTN_Timer = now; ButtonPress_SM = 3;
                        WPT_Controller1.AutoStartDone = 1;
                    }
                }
                break;
            case 3:
                if (now - BTN_Timer > 50) ButtonPress_SM = 0;
                break;
        }
    } else if (ButtonPress_SM == 1 && now - BTN_Timer > 500) {
        ButtonPress_SM = 0;
    } else if (!BTN2 && !BTN3 && ButtonPress_SM == 2) {
        BTN_1_Press();
        ButtonPress_SM = 0;
    } else if (WPT_Controller1.In_System_Menu == 1 && now - BTN_Timer > 10000) {
        LCD1.clear();
        WPT_Controller1.discardMenuEdits(); // Reverts RAM and handles exitMenu()
        Next_Menu_ID = 0;                     // Reset step pointer on timeout
    }
}
