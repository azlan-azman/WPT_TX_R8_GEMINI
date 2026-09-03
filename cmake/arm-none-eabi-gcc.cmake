# CMake Toolchain file for arm-none-eabi-gcc

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Skip compiler tests since we are cross-compiling
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)

# Use STATIC_LIBRARY for try_compile to avoid running cross-compiled executables
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Force the compiler to be found or use environment variables
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# Skip compiler verification as we are cross-compiling for bare-metal
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Define XMC1302 Cortex-M0 Specific Flags
set(XMC_CPU_FLAGS "-mcpu=cortex-m0 -mthumb -DXMC1302_T038x0200")

set(CMAKE_C_FLAGS "${XMC_CPU_FLAGS} -fdata-sections -ffunction-sections -Wall -O2" CACHE STRING "C compiler flags")
set(CMAKE_CXX_FLAGS "${XMC_CPU_FLAGS} -fdata-sections -ffunction-sections -fno-rtti -fno-exceptions -Wall -O2" CACHE STRING "C++ compiler flags")
set(CMAKE_ASM_FLAGS "${XMC_CPU_FLAGS} -x assembler-with-cpp" CACHE STRING "ASM compiler flags")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections --specs=nano.specs --specs=nosys.specs" CACHE STRING "Linker flags")
