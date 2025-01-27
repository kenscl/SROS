ENTRY(reset_handler)
_estack = 0x20020000; /* SRAM1 ends at 0x20020000 */
_Min_Leftover_RAM = 0x400;

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K  
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 112K   
    SRAM2 (rwx) : ORIGIN = 0x2001C000, LENGTH = 16K    
}

SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    .text :
    {
        . = ALIGN(4);

        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        KEEP(*(.init))
        KEEP(*(.fini))
        *(.eh_frame)
        *(.ARM.exidx)

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;

        *(.data)
        *(.data.*)
        KEEP(*(.init_array))
        KEEP(*(.fini_array))

        . = ALIGN(4);
        _edata = .;
    } > SRAM AT> FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;

        *(.bss)
        *(.bss.*)

        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > SRAM

    /* C++ specific sections */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH

    /* C++ runtime initialization */
    .init :
    {
        KEEP(*(SORT(.init.*)))
        KEEP(*(.init))
    } > FLASH

    .fini :
    {
        KEEP(*(SORT(.fini.*)))
        KEEP(*(.fini))
    } > FLASH
}
