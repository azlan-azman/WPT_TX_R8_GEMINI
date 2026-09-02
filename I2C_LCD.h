#pragma once

#include <stdint.h>

// LCD Commands
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

// Flags for display entry mode
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

// Flags for display/cursor shift
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

// Flags for function set
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00

// Flags for backlight control
#define LCD_BACKLIGHT           0x08
#define LCD_NOBACKLIGHT         0x00

#define En 0x04  // Enable bit
#define Rw 0x02  // Read/Write bit
#define Rs 0x01  // Register select bit

class I2CLCD {
public:
    I2CLCD();
    ~I2CLCD();

    void LCDbegin(unsigned char ADD = 0x40);
    void clear(void);
    void home(void);
    void setCursor(unsigned char col, unsigned char row);
    void command(unsigned char value);
    unsigned char write(const char *value);
    void display(void);
    void noDisplay(void);

    // Hardware Bus & HD44780 Resync Recovery Method
    void reinit(void);

    uint8_t DisplayIsOff;

private:
    void senddata(unsigned char value, unsigned char mode);
    void write4bits(unsigned char value);
    void expanderWrite(unsigned char _data);
    void pulseEnable(unsigned char _data);
    void sendRawNibble(unsigned char nibble);

    uint8_t _cols;
    uint8_t _rows;
    uint8_t _charsize;
    uint8_t _backlightval;
    uint8_t _addr;
    uint8_t _displayfunction;
    uint8_t _displaycontrol;
    uint8_t _displaymode;
};
