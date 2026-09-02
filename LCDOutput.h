#pragma once

#include <stdint.h>

class LCDOutput {
public:
    LCDOutput();
    ~LCDOutput();

    void LcdOutput(void);
    void DefineState(void);

private:
    uint32_t GetKey;
    char Row1[24];
};
