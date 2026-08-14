/*
 * startup_ciu32f003x.s - Cortex-M0+ Startup for CIU32F003x5
 *
 * Vector table, reset handler (copy .data, zero .bss, call main), default
 * handler and weak aliases for every interrupt.  Strong overrides for
 * ADC1_IRQHandler, USART1_IRQHandler, TIM2_IRQHandler, SysTick_Handler
 * and HardFault_Handler live in interrupts.c.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

    .syntax unified
    .cpu cortex-m0plus
    .thumb

/* ======================================================================== */
/* Linker-script symbols                                                     */
/* ======================================================================== */
    .extern __estack          /* Initial stack pointer (top of RAM)         */
    .extern _sidata           /* Load address of .data in flash             */
    .extern _sdata            /* Start of .data in RAM                       */
    .extern _edata            /* End of .data in RAM                         */
    .extern _sbss             /* Start of .bss in RAM                        */
    .extern _ebss             /* End of .bss in RAM                          */
    .extern main              /* C entry point                               */

/* ======================================================================== */
/* Default Handler (infinite loop)                                           */
/* ======================================================================== */
    .section .text.Default_Handler, "ax", %progbits
    .global Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    b       Default_Handler
    .size   Default_Handler, . - Default_Handler

/* ======================================================================== */
/* Reset Handler                                                             */
/* ======================================================================== */
    .section .text.Reset_Handler, "ax", %progbits
    .global Reset_Handler
    .type   Reset_Handler, %function
Reset_Handler:
    /* --- Copy initialized data from flash to RAM --- */
    ldr     r0, =_sidata      /* source: flash load address                 */
    ldr     r1, =_sdata       /* destination: start of .data in RAM         */
    ldr     r2, =_edata       /* end of .data in RAM                        */
.Lcopy_data:
    cmp     r1, r2
    bcc     .Lcopy_word
    b       .Lzero_bss
.Lcopy_word:
    ldr     r3, [r0], #4      /* load word from flash, post-increment       */
    str     r3, [r1], #4      /* store word to RAM, post-increment          */
    b       .Lcopy_data

    /* --- Zero the .bss section --- */
.Lzero_bss:
    ldr     r1, =_sbss        /* start of .bss                              */
    ldr     r2, =_ebss        /* end of .bss                                */
    movs    r0, #0            /* zero value                                 */
.Lzero_word:
    cmp     r1, r2
    bcc     .Lstore_zero
    b       .Lcall_main
.Lstore_zero:
    str     r0, [r1], #4      /* store zero, post-increment                 */
    b       .Lzero_word

    /* --- Call main() --- */
.Lcall_main:
    bl      main

    /* If main returns, loop forever (should never happen). */
    b       Default_Handler
    .size   Reset_Handler, . - Reset_Handler

/* ======================================================================== */
/* Weak Aliases - every handler defaults to Default_Handler                  */
/* ======================================================================== */
/* Strong definitions in interrupts.c override these.                        */

    .weak   NMI_Handler
    .thumb_set NMI_Handler, Default_Handler

    .weak   HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler

    .weak   SVC_Handler
    .thumb_set SVC_Handler, Default_Handler

    .weak   PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    .weak   SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

    .weak   WWDG_IRQHandler
    .thumb_set WWDG_IRQHandler, Default_Handler

    .weak   RTC_IRQHandler
    .thumb_set RTC_IRQHandler, Default_Handler

    .weak   FLASH_IRQHandler
    .thumb_set FLASH_IRQHandler, Default_Handler

    .weak   RCC_IRQHandler
    .thumb_set RCC_IRQHandler, Default_Handler

    .weak   EXTI0_1_IRQHandler
    .thumb_set EXTI0_1_IRQHandler, Default_Handler

    .weak   EXTI2_3_IRQHandler
    .thumb_set EXTI2_3_IRQHandler, Default_Handler

    .weak   EXTI4_15_IRQHandler
    .thumb_set EXTI4_15_IRQHandler, Default_Handler

    .weak   DMA1_Channel1_IRQHandler
    .thumb_set DMA1_Channel1_IRQHandler, Default_Handler

    .weak   DMA1_Channel2_3_IRQHandler
    .thumb_set DMA1_Channel2_3_IRQHandler, Default_Handler

    .weak   DMA1_Channel4_5_IRQHandler
    .thumb_set DMA1_Channel4_5_IRQHandler, Default_Handler

    .weak   ADC1_IRQHandler
    .thumb_set ADC1_IRQHandler, Default_Handler

    .weak   TIM1_BRK_UP_TRG_COM_IRQHandler
    .thumb_set TIM1_BRK_UP_TRG_COM_IRQHandler, Default_Handler

    .weak   TIM1_CC_IRQHandler
    .thumb_set TIM1_CC_IRQHandler, Default_Handler

    .weak   TIM2_IRQHandler
    .thumb_set TIM2_IRQHandler, Default_Handler

    .weak   TIM3_IRQHandler
    .thumb_set TIM3_IRQHandler, Default_Handler

    .weak   SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler, Default_Handler

    .weak   USART1_IRQHandler
    .thumb_set USART1_IRQHandler, Default_Handler

/* ======================================================================== */
/* Vector Table                                                              */
/* ======================================================================== */
    .section .isr_vector, "a", %progbits
    .global __isr_vector
    .type   __isr_vector, %object
    .align  2
__isr_vector:
    .word   __estack                 /*  0: Initial Stack Pointer          */
    .word   Reset_Handler            /*  1: Reset                          */
    .word   NMI_Handler              /*  2: Non-Maskable Interrupt         */
    .word   HardFault_Handler        /*  3: Hard Fault                     */
    .word   0                        /*  4: Reserved (MemManage - M0+)     */
    .word   0                        /*  5: Reserved (BusFault  - M0+)     */
    .word   0                        /*  6: Reserved (UsageFault- M0+)     */
    .word   0                        /*  7: Reserved                       */
    .word   0                        /*  8: Reserved                       */
    .word   0                        /*  9: Reserved                       */
    .word   0                        /* 10: Reserved                       */
    .word   SVC_Handler              /* 11: SVCall                         */
    .word   0                        /* 12: Reserved (DebugMon - M0+)      */
    .word   0                        /* 13: Reserved                       */
    .word   PendSV_Handler           /* 14: PendSV                         */
    .word   SysTick_Handler          /* 15: SysTick                        */

    /* --- Peripheral interrupts (IRQ 0 .. 27) --- */
    .word   WWDG_IRQHandler          /*  0: Window Watchdog                */
    .word   0                        /*  1: Reserved (PVD)                 */
    .word   RTC_IRQHandler           /*  2: RTC                            */
    .word   FLASH_IRQHandler         /*  3: Flash global                   */
    .word   RCC_IRQHandler           /*  4: RCC global                     */
    .word   EXTI0_1_IRQHandler       /*  5: EXTI line 0 & 1                */
    .word   EXTI2_3_IRQHandler       /*  6: EXTI line 2 & 3                */
    .word   EXTI4_15_IRQHandler      /*  7: EXTI line 4 to 15              */
    .word   0                        /*  8: Reserved                       */
    .word   DMA1_Channel1_IRQHandler /*  9: DMA1 channel 1                 */
    .word   DMA1_Channel2_3_IRQHandler /* 10: DMA1 channel 2 & 3           */
    .word   DMA1_Channel4_5_IRQHandler /* 11: DMA1 channel 4 & 5           */
    .word   0                        /* 12: Reserved                       */
    .word   ADC1_IRQHandler          /* 13: ADC1 global                    */
    .word   0                        /* 14: Reserved                       */
    .word   0                        /* 15: Reserved                       */
    .word   0                        /* 16: Reserved                       */
    .word   TIM1_BRK_UP_TRG_COM_IRQHandler /* 17: TIM1 break/up/trig/comm  */
    .word   TIM1_CC_IRQHandler       /* 18: TIM1 capture compare           */
    .word   TIM2_IRQHandler          /* 19: TIM2 global                    */
    .word   TIM3_IRQHandler          /* 20: TIM3 global                    */
    .word   0                        /* 21: Reserved                       */
    .word   0                        /* 22: Reserved                       */
    .word   0                        /* 23: Reserved                       */
    .word   0                        /* 24: Reserved                       */
    .word   SPI1_IRQHandler          /* 25: SPI1 global                    */
    .word   0                        /* 26: Reserved                       */
    .word   USART1_IRQHandler        /* 27: USART1 global                  */
    .size   __isr_vector, . - __isr_vector

/* ======================================================================== */
/* End of file                                                               */
/* ======================================================================== */
    .end
