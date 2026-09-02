extern "C" {
#include "XMC1300.h"
#include "xmc_scu.h"
#include "xmc_gpio.h"
#include "PinDefine.h"
#include "XMC1000_RomFunctionTable.h"
}

#define APP_START_ADDR     0x10001000U
#define STAGING_START_ADDR 0x1001B000U

extern "C" __attribute__((section(".data"), noinline))
void execute_sram_swap(uint32_t dstAddr, uint32_t srcAddr, uint32_t imageSize) {
    __disable_irq();

    uint32_t endAddr = dstAddr + imageSize;
    uint32_t curDst  = dstAddr;
    uint32_t curSrc  = srcAddr;

    alignas(4) uint32_t sramPageBuf[64];

    volatile uint32_t *wdt_srv  = (volatile uint32_t *)((0x4002U << 16) | 0x0004U); // 0x40020004 (WDT->SRV)
    volatile uint32_t *aircr    = (volatile uint32_t *)((0xE000U << 16) | 0xED0CU); // 0xE000ED0C (SCB->AIRCR)
    volatile uint32_t *nvm_stat = (volatile uint32_t *)0x40050000U;                 // NVMMEMU status register
    uint32_t reset_key          = (0x05FAU << 16) | 0x0004U;

    while (curDst < endAddr) {
        *wdt_srv  = 0x6E524635U; // Service Watchdog
        *nvm_stat = 0U;          // Clear NVM error status

        // Erase active Flash page via Silicon ROM
        XMC1000_NvmErasePage((uint32_t *)curDst);

        // Copy staged payload into SRAM buffer
        const uint32_t *pSrc = (const uint32_t *)curSrc;
        for (uint32_t i = 0; i < 64; ++i) {
            sramPageBuf[i] = pSrc[i];
        }

        *nvm_stat = 0U;
        // Program and verify SRAM buffer into active Flash
        XMC1000_NvmProgVerify(sramPageBuf, (uint32_t *)curDst);

        curSrc += 256U;
        curDst += 256U;
    }

    *aircr = reset_key;
    while (1) { __NOP(); }
}
