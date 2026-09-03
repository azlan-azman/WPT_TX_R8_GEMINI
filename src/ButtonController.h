#pragma once

#include <stdint.h>

class ButtonController {
public:
    ButtonController();
    ~ButtonController();

    void BTN_1_Press(void);
    void BTN_2_Press(void);
    void BTN_3_Press(void);
    void ButtonPress(void);

    uint8_t  Enable_BTN_Press;
    uint8_t  Next_Menu_ID;
    uint8_t  ButtonPress_SM;
    uint32_t BTN_Timer;
    uint8_t  Button1Count;
    uint8_t  Button1Reset;
    uint8_t  Button2Count;
    uint8_t  Button2Reset;
};
