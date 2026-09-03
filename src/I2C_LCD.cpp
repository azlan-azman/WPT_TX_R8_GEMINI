#include "I2C_LCD.h"
#include "WPTController.hpp"
#include "PinDefine.h"
#include "xmc_i2c.h"
#include "xmc_gpio.h"
#include "xmc_common.h"

extern WPT_Controller WPT_Controller1;
extern void LCDOutput_ClearCache(void);

I2CLCD::I2CLCD()
    : DisplayIsOff(0),
      _cols(20),
      _rows(4),
      _charsize(LCD_5x8DOTS),
      _backlightval(LCD_BACKLIGHT),
      _addr(0x40),
      _displayfunction(0),
      _displaycontrol(0),
      _displaymode(0) {}

I2CLCD::~I2CLCD() {}

void I2CLCD::LCDbegin(unsigned char ADD) {
    _addr = ADD; // 0x40 (8-bit I2C Write Address)
    _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    if (_rows > 1) _displayfunction |= LCD_2LINE;

    reinit();
}

// Low-level raw single-nibble ping to reset HD44780 state machine
void I2CLCD::sendRawNibble(unsigned char nibble) {
    uint8_t dataHigh = nibble | _backlightval | En;
    uint8_t dataLow  = nibble | _backlightval;

    XMC_I2C_CH_MasterStart(XMC_I2C0_CH1, _addr, XMC_I2C_CH_CMD_WRITE);
    XMC_I2C_CH_MasterTransmit(XMC_I2C0_CH1, dataHigh);
    XMC_DelayUs(2);
    XMC_I2C_CH_MasterTransmit(XMC_I2C0_CH1, dataLow);
    XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);
    XMC_Delay(5);
}

// Full HD44780 + I2C Hardware Re-Initialization
void I2CLCD::reinit(void) {
    // 1. Reset USIC I2C Hardware Controller
    XMC_I2C_CH_Stop(XMC_I2C0_CH1);
    XMC_I2C0_CH1->FMR = 0x00000002U; // Flush USIC TDV Transmit Buffer
    XMC_I2C0_CH1->PSCR |= USIC_CH_PSR_IICMode_WTDF_Msk; // Clear WTDF
    XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, 0xFFFFFFFFU);
    XMC_I2C_CH_Start(XMC_I2C0_CH1);

    // 2. Official Hitachi HD44780 4-bit Resync Ping Sequence
    XMC_Delay(50);
    sendRawNibble(0x30); // 1st try: Force 8-bit state
    sendRawNibble(0x30); // 2nd try
    sendRawNibble(0x30); // 3rd try
    sendRawNibble(0x20); // Force 4-bit state

    // 3. Restore Display Mode Registers
    command(LCD_FUNCTIONSET | _displayfunction);
    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    command(LCD_DISPLAYCONTROL | _displaycontrol);

    command(LCD_CLEARDISPLAY);
    XMC_Delay(2);

    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    command(LCD_ENTRYMODESET | _displaymode);
    home();

    LCDOutput_ClearCache();
    WPT_Controller1.I2C_RST_required = 0;
}

void I2CLCD::clear(void) {
    command(LCD_CLEARDISPLAY);
    XMC_Delay(2);
    LCDOutput_ClearCache(); // Invalidate cache on physical screen clear
}

void I2CLCD::home(void) {
    command(LCD_RETURNHOME);
    XMC_Delay(2);
}

void I2CLCD::setCursor(unsigned char col, unsigned char row) {
    int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= _rows) row = _rows - 1;
    command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void I2CLCD::command(unsigned char value) {
    senddata(value, 0);
}

unsigned char I2CLCD::write(const char *value) {
    unsigned char counter = 0;
    while (*value != '\0' && counter < _cols) {
        senddata(static_cast<unsigned char>(*value++), Rs);
        counter++;
    }
    return 1;
}

void I2CLCD::senddata(unsigned char value, unsigned char mode) {
    unsigned char highnib = value & 0xF0;
    unsigned char lownib  = (value << 4) & 0xF0;
    write4bits(highnib | mode);
    write4bits(lownib | mode);
}

void I2CLCD::write4bits(unsigned char value) {
    expanderWrite(value);
    pulseEnable(value);
}

void I2CLCD::expanderWrite(unsigned char _data) {
    if (_addr == 0 || WPT_Controller1.I2C_TX_Done == 0) return;

    // Trigger recovery in main loop if reset flag was raised
    if (WPT_Controller1.I2C_RST_required) return;

    uint8_t payload = _data | _backlightval;
    uint32_t startMs;

    // 1. Send Start Condition
    XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED | XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED);
    XMC_I2C_CH_MasterStart(XMC_I2C0_CH1, _addr, XMC_I2C_CH_CMD_WRITE);

    startMs = getTickMs();
    while (!(XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & (XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED | XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED))) {
        if (getTickMs() - startMs >= 5U) {
            WPT_Controller1.I2C_RST_required = 1;
            return;
        }
    }

    if (XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED) {
        XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED);
        XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);
        WPT_Controller1.I2C_RST_required = 1;
        return;
    }
    XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

    // 2. Transmit Payload
    XMC_I2C_CH_MasterTransmit(XMC_I2C0_CH1, payload);

    startMs = getTickMs();
    while (!(XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & (XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED | XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED))) {
        if (getTickMs() - startMs >= 5U) {
            WPT_Controller1.I2C_RST_required = 1;
            return;
        }
    }

    if (XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED) {
        XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED);
        XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);
        WPT_Controller1.I2C_RST_required = 1;
        return;
    }
    XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

    // 3. Stop Condition
    XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);
}

void I2CLCD::pulseEnable(unsigned char _data) {
    expanderWrite(_data | En);
    expanderWrite(_data & ~En);
}

void I2CLCD::display()   { _displaycontrol |= LCD_DISPLAYON;  command(LCD_DISPLAYCONTROL | _displaycontrol); }
void I2CLCD::noDisplay() { _displaycontrol &= ~LCD_DISPLAYON; command(LCD_DISPLAYCONTROL | _displaycontrol); }
