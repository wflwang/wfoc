/*
 * linker.ld - GNU LD Linker Script for CIU32F003x5
 *
 * Memory map:
 *   Flash: 0x08000000, 16 K  (code + rodata + .data load image)
 *   RAM:   0x20000000,  3 K  (.data + .bss + stack + heap)
 *
 * The vector table is placed at the very start of flash so the CPU loads
 * the initial stack pointer and reset handler on power-on.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

/* ======================================================================== */
/* Entry Point                                                               */
/* ======================================================================== */
ENTRY(Reset_Handler)

/* ======================================================================== */
/* Memory Regions                                                            */
/* ======================================================================== */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 16K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 3K
}

/* ======================================================================== */
/* Stack Size                                                                */
/* ======================================================================== */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Minimum heap and stack sizes. The linker reserves space at the top of
 * RAM for the stack; the heap grows up from the end of .bss.               */
_Min_Heap_Size  = 0x100;   /* 256 bytes heap   */
_Min_Stack_Size = 0x200;   /* 512 bytes stack  */

/* ======================================================================== */
/* Sections                                                                  */
/* ======================================================================== */
SECTIONS
{
    /* --- Interrupt Vector Table (must be first in flash) --- */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* --- Code --- */
    .text :
    {
        . = ALIGN(4);
        *(.text)                   /* all .text sections                   */
        *(.text*)
        *(.glue_7)                 /* ARM/Thumb interworking glue         */
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
    } > FLASH

    /* --- Read-only data --- */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* --- ARM exception unwinding tables --- */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    /* --- Preinit / init / fini arrays --- */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH

    /* --- Load address of .data in flash --- */
    _sidata = LOADADDR(.data);

    /* --- Initialized data (copied from flash to RAM by Reset_Handler) --- */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;              /* start of .data in RAM                    */
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;              /* end of .data in RAM                      */
    } > RAM AT > FLASH

    /* --- Zero-initialized data (cleared by Reset_Handler) --- */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;               /* start of .bss                            */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;               /* end of .bss                              */
        __bss_end__ = _ebss;
    } > RAM

    /* --- Heap and stack reservation at the top of RAM --- */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > RAM

    /* --- Discard unneeded sections --- */
    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
        *(.ARM.attributes)
    }
}

/* ======================================================================== */
/* End of linker script                                                      */
/* ======================================================================== */
