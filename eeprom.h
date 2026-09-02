#pragma once

#include <stdint.h>

// EEPROM Parameter Start Address Offsets
#define SETTING_1_Start_ADD 0x00
#define SETTING_2_Start_ADD 0x04
#define SETTING_3_Start_ADD 0x08
#define SETTING_4_Start_ADD 0x0C
#define SETTING_5_Start_ADD 0x10
#define SETTING_6_Start_ADD 0x14
#define SETTING_7_Start_ADD 0x18

int i2c_eeprom_write_long(unsigned int intDeviceAddress, unsigned int intEEAddress, unsigned char *pdata);
int i2c_eeprom_read_long(unsigned int intDeviceAddress, unsigned int intEEAddress, unsigned char *pdata);
