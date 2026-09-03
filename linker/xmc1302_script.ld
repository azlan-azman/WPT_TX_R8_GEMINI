OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(Reset_Handler)

MEMORY
{
    BOOT_FLASH (rx) : ORIGIN = 0x10001000, LENGTH = 0x4000   /* 16 KB Bootloader */
    APP_FLASH  (rx) : ORIGIN = 0x10005000, LENGTH = 0x15000  /* 84 KB Application */
    /* LOG_FLASH Sector at 0x1001A000 (4 KB) */
    /* STAGING_FLASH Sector at 0x1001B000 (84 KB) */
    SRAM      (!rx) : ORIGIN = 0x20000000, LENGTH = 0x4000   /* 16 KB SRAM */
}

stack_size = DEFINED(stack_size) ? stack_size : 1024;
no_init_size = 8; /* Reserve 8 bytes (0x20003FF8 - 0x20003FFF) from startup zeroing */

SECTIONS
{
    /* --- 1. PROTECTED BOOTLOADER REGION (0x10001000 - 0x10004FFF) --- */
    .bootloader_section :
    {
        sBoot = .;
        KEEP(*(.reset)); /* Reset vector on cold boot */
        *(.bootloader_code);
        *main_bootloader*(.text .text.* .rodata .rodata.*);
        *startup_XMC1300*(.text .text.* .rodata .rodata.*);
        *system_XMC1300*(.text .text.* .rodata .rodata.*);
        
        /* Force Flash, UART, GPIO & SCU drivers into protected BOOT_FLASH */
        *xmc1_flash*(.text .text.* .rodata .rodata.*);
        *xmc_flash*(.text .text.* .rodata .rodata.*);
        *xmc_uart*(.text .text.* .rodata .rodata.*);
        *xmc_gpio*(.text .text.* .rodata .rodata.*);
        *xmc_scu*(.text .text.* .rodata .rodata.*);
        . = ALIGN(4);
    } > BOOT_FLASH

    /* --- 2. APPLICATION VECTOR TABLE (0x10005000) --- */
    .app_vectors :
    {
        . = ALIGN(4);
        KEEP(*(.app_vector_table)); /* Vector entries for jumpToApplication() */
        . = ALIGN(4);
    } > APP_FLASH

    /* --- 3. ERASEABLE APPLICATION CODE REGION (0x10005000+) --- */
    .text : 
    {
      sText = .;
      *(.text .text.* .gnu.linkonce.t.*);

      /* C++ Support */
      KEEP(*(.init))
      KEEP(*(.fini))

      *crtbegin.o(.ctors)
      *crtbegin?.o(.ctors)
      *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
      *(SORT(.ctors.*))
      *(.ctors)

      *crtbegin.o(.dtors)
      *crtbegin?.o(.dtors)
      *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
      *(SORT(.dtors.*))
      *(.dtors)

      *(.rodata .rodata.*)
      *(.gnu.linkonce.r*)

      *(vtable)        

      . = ALIGN(4);
    } > APP_FLASH

    .eh_frame_hdr : ALIGN (4)
    {
      KEEP (*(.eh_frame_hdr))
    } > APP_FLASH

    .eh_frame : ALIGN (4)
    {
      KEEP (*(.eh_frame))
    } > APP_FLASH

    .ARM.extab : ALIGN(4)
    {
      *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > APP_FLASH

    . = ALIGN(4);
    __exidx_start = .;
    .ARM.exidx : ALIGN(4)
    {
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > APP_FLASH
    __exidx_end = .;
    . = ALIGN(4);

    .VENEER_Code ABSOLUTE(0x2000000C):
    {
        . = ALIGN(4);
        VeneerStart = .;
        KEEP(*(.XmcVeneerCode));
        . = ALIGN(4);
        VeneerEnd = .;
    } > SRAM AT > APP_FLASH
    eROData = LOADADDR (.VENEER_Code);
    VeneerSize = ABSOLUTE(VeneerEnd) - ABSOLUTE(VeneerStart);

    Stack (NOLOAD) : AT(0)
    {
        . = ALIGN(8);
        . = . + stack_size;
        __initial_sp = .;
    } > SRAM

    .data :
    {
      . = ALIGN(4);
      __data_start = .;
      * (.data);
      * (.data*);
      *(*.data);
      *(.gnu.linkonce.d*)
      
      . = ALIGN(4);
      PROVIDE_HIDDEN (__preinit_array_start = .);
      KEEP(*(.preinit_array))
      PROVIDE_HIDDEN (__preinit_array_end = .);

      . = ALIGN(4);
      PROVIDE_HIDDEN (__init_array_start = .);
      KEEP(*(SORT(.init_array.*)))
      KEEP(*(.init_array))
      PROVIDE_HIDDEN (__init_array_end = .);

      . = ALIGN(4);
      PROVIDE_HIDDEN (__fini_array_start = .);
      KEEP(*(SORT(.fini_array.*)))
      KEEP(*(.fini_array))
      PROVIDE_HIDDEN (__fini_array_end = .);

      . = ALIGN(4);
      __data_end = .;
    } > SRAM AT > APP_FLASH
    DataLoadAddr = LOADADDR (.data);
    __data_size = __data_end - __data_start;

    .ram_code :
    {
        . = ALIGN(4);
        __ram_code_start = .;
        *(.ram_code)   
        . = ALIGN(4);
        __ram_code_end = .;
    } > SRAM AT > APP_FLASH
    __ram_code_load = LOADADDR (.ram_code);
    __ram_code_size = __ram_code_end - __ram_code_start;
    
    __text_size = (__exidx_end - sText) + VeneerSize + __data_size + __ram_code_size;
    eText = sText + __text_size;

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start = .;
        * (.bss);
        * (.bss*);
        * (COMMON);
        *(.gnu.linkonce.b*)
        . = ALIGN(4);
        __bss_end = .;
        . = ALIGN(8);
        Heap_Bank1_Start = .;
    } > SRAM
    __bss_size = __bss_end - __bss_start;
    
    .no_init ORIGIN(SRAM) + LENGTH(SRAM) - no_init_size (NOLOAD) : 
    {
        Heap_Bank1_End = .;
        * (.no_init);
    } > SRAM
    
    Heap_Bank1_Size = Heap_Bank1_End - Heap_Bank1_Start;

    ASSERT(Heap_Bank1_Start <= Heap_Bank1_End, "region SRAM overflowed no_init section")

    /* End of data section - needed by newlib's _sbrk for heap allocation */
    . = ALIGN(4);
    _end = .;
    __end__ = .;
    end = .;

    /DISCARD/ :
    {
        *(.comment)
    }
}