cmake_minimum_required(VERSION 3.15)

# Embedded firmware: C for drivers/CMSIS, C++ for application, ASM for startup
project(xmc1302_firmware LANGUAGES C CXX ASM)

# Standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Linker script
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/linker/xmc1302_script.ld")
add_link_options("-T${LINKER_SCRIPT}")

# ==============================================================================
# Include paths
# ==============================================================================
include_directories(
    src
    Startup
    Libraries
    Libraries/CMSIS/Include
    Libraries/CMSIS/Infineon/XMC1300_series/Include
    Libraries/XMCLib/inc
)

# ==============================================================================
# Application source files (C++)
# ==============================================================================
set(APP_SOURCES
    src/main.cpp
    src/WPTController.hpp
    src/InverterPWM.hpp
    src/PowerControl.hpp
    src/AnalogADC.hpp
    src/AnalogACMP.hpp
    src/HardwareERU.hpp
    src/FrequencyCapture.hpp
    src/UARTController.hpp
    src/UARTProtocolController.cpp
    src/UARTProtocolController.hpp
    src/I2CController.hpp
    src/I2C_LCD.cpp
    src/I2C_LCD.h
    src/LCDOutput.cpp
    src/LCDOutput.h
    src/ButtonController.cpp
    src/ButtonController.h
    src/FanMonitor.hpp
    src/PinDefine.h
    src/eeprom.cpp
    src/eeprom.h
    # Bootloader helper (execute_sram_swap used by UARTProtocolController)
    src/main_bootloader.cpp
)

# ==============================================================================
# CMSIS + System source files
# ==============================================================================
set(CMSIS_SOURCES
    Startup/startup_XMC1300.S
    Startup/system_XMC1300.c
)

# ==============================================================================
# XMCLib peripheral drivers (XMC1300 = XMC1 family)
# All files are guarded by #if UC_FAMILY == XMC1 in the library sources.
# ==============================================================================
set(XMCLIB_SOURCES
    Libraries/XMCLib/src/xmc_common.c
    Libraries/XMCLib/src/TOOLCHAIN_GCC_ARM/xmc_common_gcc.S
    Libraries/XMCLib/src/xmc1_scu.c
    Libraries/XMCLib/src/xmc1_gpio.c
    Libraries/XMCLib/src/xmc_gpio.c
    Libraries/XMCLib/src/xmc_ccu4.c
    Libraries/XMCLib/src/xmc_ccu8.c
    Libraries/XMCLib/src/xmc1_eru.c
    Libraries/XMCLib/src/xmc_usic.c
    Libraries/XMCLib/src/xmc_uart.c
    Libraries/XMCLib/src/xmc_i2c.c
    Libraries/XMCLib/src/xmc_vadc.c
    Libraries/XMCLib/src/xmc_acmp.c
    Libraries/XMCLib/src/xmc1_flash.c
    Libraries/XMCLib/src/xmc_eru.c
)

# ==============================================================================
# Collect all sources
# ==============================================================================
set(ALL_SOURCES ${APP_SOURCES} ${CMSIS_SOURCES} ${XMCLIB_SOURCES})

# ==============================================================================
# Executable target
# ==============================================================================
add_executable(${PROJECT_NAME}.elf ${ALL_SOURCES})

set_target_properties(${PROJECT_NAME}.elf PROPERTIES
    SUFFIX ""
)

# ==============================================================================
# Post-build: generate HEX, BIN and show size
# ==============================================================================
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} --format=berkeley ${PROJECT_NAME}.elf
    COMMENT "Generating auxiliary binary formats and showing size info..."
)
