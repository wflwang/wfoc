/*
 * ciu32f003x.h - CIU32F003x5 MCU Register Definitions
 *
 * CIU32F003x5: ARM Cortex-M0+ core, up to 24 MHz, QFN-20 package
 *   - 16 KB Flash, 3 KB SRAM
 *   - 1x Advanced timer (TIM1) for 3-phase PWM
 *   - 2x General purpose timers (TIM2 / TIM3)
 *   - 1x 12-bit ADC (up to 8 channels, DMA support)
 *   - 1x UART, 1x SPI
 *   - GPIO: PA0-PA7, PB0-PB7, PC0-PC1
 *
 * Used by the WFOC (WFL FOC) open source FOC motor controller.
 * Style modelled after STMicroelectronics stm32f0xx.h header.
 *
 * WFOC (WFL FOC) - Open Source Low-Cost High-Performance FOC Controller
 * Copyright (c) 2026 wflwang
 * Licensed under MIT License
 */

#ifndef CIU32F003X_H
#define CIU32F003X_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CIU32F003X
  * @{
  */

/** @addtogroup Device_included
  * @{
  */
#if !defined (CIU32F003x5)
  #define CIU32F003x5          /* Target MCU variant */
#endif

/* ======================================================================== */
/* Includes                                                                  */
/* ======================================================================== */
#include <stdint.h>

/* ======================================================================== */
/* Volatile Access Type Definitions (CMSIS-style)                            */
/* ======================================================================== */
#ifdef __cplusplus
  #define __I     volatile           /*!< Read-only register (const in C++)   */
  #define __O     volatile           /*!< Write-only register                  */
  #define __IO    volatile           /*!< Read/write register                  */
#else
  #define __I     volatile const     /*!< Read-only register                   */
  #define __O     volatile           /*!< Write-only register                  */
  #define __IO    volatile           /*!< Read/write register                  */
#endif

/* ======================================================================== */
/* Memory Map Constants                                                      */
/* ======================================================================== */
#define FLASH_BASE          (0x08000000UL)  /*!< Flash memory base address   */
#define SRAM_BASE           (0x20000000UL)  /*!< SRAM base address           */
#define SRAM_BB_BASE        (SRAM_BASE  + 0x02000000UL) /*!< SRAM bit-band   */
#define PERIPH_BASE         (0x40000000UL)  /*!< Peripheral base address     */
#define PERIPH_BB_BASE      (PERIPH_BASE + 0x02000000UL) /*!< Periph bit-band */

/* AHB/APB peripheral regions (STM32F0-like topology) */
#define APBPERIPH_BASE      (PERIPH_BASE)
#define AHBPERIPH_BASE      (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE     (PERIPH_BASE + 0x08000000UL)

/* Memory sizes for CIU32F003x5 */
#define FLASH_SIZE          (16U * 1024U)   /*!< 16 KB Flash                 */
#define SRAM_SIZE            (3U * 1024U)   /*!< 3 KB SRAM                   */

/* ======================================================================== */
/* Interrupt Number Definition (IRQn)                                        */
/* ======================================================================== */
typedef enum {
    /* ---- Cortex-M0+ core exceptions (negative numbers) ---- */
    Reset_IRQn                  = -15,  /*!< Reset handler                       */
    NMI_IRQn                    = -14,  /*!< Non maskable interrupt              */
    HardFault_IRQn              = -13,  /*!< Hard fault                          */
    SVCall_IRQn                 = -5,   /*!< SVCall                              */
    PendSV_IRQn                 = -2,   /*!< PendSV                              */
    SysTick_IRQn                = -1,   /*!< System tick                         */

    /* ---- CIU32F003x5 peripheral interrupts ---- */
    WWDG_IRQn                   = 0,    /*!< Window watchdog                     */
    RTC_IRQn                    = 2,    /*!< RTC interrupt                       */
    FLASH_IRQn                  = 3,    /*!< Flash global                        */
    RCC_IRQn                    = 4,    /*!< RCC global                          */
    EXTI0_1_IRQn                = 5,    /*!< EXTI line 0 & 1                     */
    EXTI2_3_IRQn                = 6,    /*!< EXTI line 2 & 3                     */
    EXTI4_15_IRQn               = 7,    /*!< EXTI line 4 to 15                   */
    DMA1_Channel1_IRQn          = 9,    /*!< DMA1 channel 1                       */
    DMA1_Channel2_3_IRQn        = 10,   /*!< DMA1 channel 2 & 3                  */
    DMA1_Channel4_5_IRQn        = 11,   /*!< DMA1 channel 4 & 5                  */
    ADC1_IRQn                   = 13,   /*!< ADC1 global                          */
    TIM1_BRK_UP_TRG_COM_IRQn    = 17,   /*!< TIM1 break/up/trig/comm             */
    TIM1_CC_IRQn                = 18,   /*!< TIM1 capture compare                */
    TIM2_IRQn                   = 19,   /*!< TIM2 global                         */
    TIM3_IRQn                   = 20,   /*!< TIM3 global                         */
    USART1_IRQn                 = 27,   /*!< USART1 global                       */
    SPI1_IRQn                   = 25,   /*!< SPI1 global                          */
} IRQn_Type;

/* ======================================================================== */
/* Bit Manipulation Macros                                                    */
/* ======================================================================== */
/*! Generate a contiguous bit mask from bit [l] to bit [h] inclusive */
#define GENMASK(h, l)       (((1UL << ((h) - (l) + 1UL)) - 1UL) << (l))

/*! Single-bit mask at position (x) */
#define BIT(x)              (1UL << (x))

/*! Set bit(s) BIT in register REG */
#define SET_BIT(REG, BIT)   ((REG) |= (BIT))

/*! Clear bit(s) BIT in register REG */
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))

/*! Read bit(s) BIT from register REG (non-zero if set) */
#define READ_BIT(REG, BIT)  ((REG) & (BIT))

/*! Clear bits in REG based on CLEAR_MSK, then set bits SET_MSK */
#define CLEAR_REG(REG)      ((REG) = 0)

/*! Write VAL into REG */
#define WRITE_REG(REG, VAL) ((REG) = (VAL))

/*! Read REG value */
#define READ_REG(REG)       ((REG))

/*! Modify REG: keep bits outside MSK, replace bits inside MSK with VAL */
#define MODIFY_REG(REG, CLEARMASK, SETMASK) \
        WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | ((SETMASK) & (CLEARMASK))))

/*! Position of bit field BIT in a register field (value field name _Pos) */
#define POS_VAL(VAL)        (VAL)

/* ======================================================================== */
/* Peripheral Base Addresses                                                  */
/* ======================================================================== */

/* ---- AHB peripherals ---- */
#define DMA1_BASE           (AHBPERIPH_BASE + 0x00000000UL)
#define RCC_BASE            (AHBPERIPH_BASE + 0x00001000UL)
#define FLASH_R_BASE        (AHBPERIPH_BASE + 0x00002000UL)  /*!< Flash interface */
#define CRC_BASE            (AHBPERIPH_BASE + 0x00003000UL)

/* ---- APB peripherals ---- */
#define TIM2_BASE           (APBPERIPH_BASE + 0x00000000UL)
#define TIM3_BASE           (APBPERIPH_BASE + 0x00000400UL)

#define RTC_BASE            (APBPERIPH_BASE + 0x00002800UL)
#define WWDG_BASE           (APBPERIPH_BASE + 0x00002C00UL)
#define IWDG_BASE           (APBPERIPH_BASE + 0x00003000UL)

#define SPI1_BASE           (APBPERIPH_BASE + 0x00003000UL)
#define USART1_BASE         (APBPERIPH_BASE + 0x00003800UL)

#define ADC1_BASE           (APBPERIPH_BASE + 0x00012400UL)
#define TIM1_BASE           (APBPERIPH_BASE + 0x00012C00UL)

/* ---- AHB2 / IOPORT (GPIO) ---- */
#define GPIOA_BASE          (AHB2PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE          (AHB2PERIPH_BASE + 0x00000400UL)
#define GPIOC_BASE          (AHB2PERIPH_BASE + 0x00000800UL)
#define GPIOF_BASE          (AHB2PERIPH_BASE + 0x00001400UL)  /*!< For CRC/remap */

#define SYSCFG_BASE         (APBPERIPH_BASE + 0x00010000UL)
#define EXTI_BASE           (APBPERIPH_BASE + 0x00010400UL)

/* ======================================================================== */
/* Peripheral Register Access (pointer macros)                               */
/* ======================================================================== */
#define TIM1                ((TIM1_TypeDef *)  TIM1_BASE)
#define TIM2                ((TIM_TypeDef *)   TIM2_BASE)
#define TIM3                ((TIM_TypeDef *)   TIM3_BASE)
#define ADC1                ((ADC_TypeDef *)   ADC1_BASE)
#define DMA1                ((DMA_TypeDef *)  DMA1_BASE)
#define RCC                 ((RCC_TypeDef *)   RCC_BASE)
#define FLASH_REG          ((FLASH_TypeDef *) FLASH_R_BASE)
#define SPI1                ((SPI_TypeDef *)   SPI1_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *)SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *)  EXTI_BASE)
#define CRC                 ((CRC_TypeDef *)  CRC_BASE)

/* ======================================================================== */
/* ARM Cortex-M0+ Core Registers                                              */
/* ======================================================================== */
#define SCS_BASE            (0xE000E000UL)  /*!< System Control Space base      */
#define SysTick_BASE        (SCS_BASE + 0x0010UL)
#define NVIC_BASE           (SCS_BASE + 0x0100UL)
#define SCB_BASE            (SCS_BASE + 0x0D00UL)

/* ======================================================================== */
/* SysTick Register Structure                                                */
/* ======================================================================== */
typedef struct {
    __IO uint32_t CTRL;                     /*!< Offset 0x00: SysTick control   */
    __IO uint32_t LOAD;                     /*!< Offset 0x04: SysTick reload    */
    __IO uint32_t VAL;                      /*!< Offset 0x08: SysTick current   */
    __I  uint32_t CALIB;                    /*!< Offset 0x0C: SysTick calibration*/
} SysTick_Type;

/* SysTick CTRL bits */
#define SysTick_CTRL_ENABLE_Pos          0
#define SysTick_CTRL_ENABLE_Msk          (1UL << SysTick_CTRL_ENABLE_Pos)
#define SysTick_CTRL_TICKINT_Pos          1
#define SysTick_CTRL_TICKINT_Msk         (1UL << SysTick_CTRL_TICKINT_Pos)
#define SysTick_CTRL_CLKSOURCE_Pos        2
#define SysTick_CTRL_CLKSOURCE_Msk       (1UL << SysTick_CTRL_CLKSOURCE_Pos)
#define SysTick_CALIB_NOREF_Pos           31
#define SysTick_CALIB_NOREF_Msk          (1UL << SysTick_CALIB_NOREF_Pos)
#define SysTick_CALIB_SKEW_Pos           30
#define SysTick_CALIB_SKEW_Msk           (1UL << SysTick_CALIB_SKEW_Pos)

#define SysTick              ((SysTick_Type *) SysTick_BASE)

/* ======================================================================== */
/* NVIC Register Structure                                                   */
/* ======================================================================== */
typedef struct {
    __IO uint32_t ISER[1U];                /*!< Offset 0x000: Interrupt Set-Enable */
    uint32_t RESERVED0[31U];
    __IO uint32_t ICER[1U];                 /*!< Offset 0x080: Interrupt Clear-Enable*/
    uint32_t RESERVED1[31U];
    __IO uint32_t ISPR[1U];                 /*!< Offset 0x100: Interrupt Set-Pending */
    uint32_t RESERVED2[31U];
    __IO uint32_t ICPR[1U];                 /*!< Offset 0x180: Interrupt Clear-Pending*/
    uint32_t RESERVED3[31U];
    __I  uint32_t IPR[8U];                  /*!< Offset 0x300: Interrupt Priority    */
} NVIC_Type;

#define NVIC                ((NVIC_Type *) NVIC_BASE)

/* NVIC helper macros */
#define NVIC_SetPriorityGrouping(X)  /* not supported on Cortex-M0+ */
#define NVIC_GetPriorityGrouping()   (0U)

#define NVIC_EnableIRQ(IRQn)         (NVIC->ISER[0] = (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL)))
#define NVIC_DisableIRQ(IRQn)        (NVIC->ICER[0] = (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL)))
#define NVIC_GetPendingIRQ(IRQn)    ((NVIC->ISPR[0] >> ((uint32_t)(int32_t)IRQn & 0x1FUL)) & 1UL)
#define NVIC_SetPendingIRQ(IRQn)     (NVIC->ISPR[0] = (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL)))
#define NVIC_ClearPendingIRQ(IRQn)  (NVIC->ICPR[0] = (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL)))
#define NVIC_SetPriority(IRQn,prio)  (NVIC->IPR[(uint32_t)(int32_t)IRQn >> 2UL] = \
        ((uint32_t)(NVIC->IPR[(uint32_t)(int32_t)IRQn >> 2UL] & \
          (~(0xFFUL << (((uint32_t)(int32_t)IRQn) & 0x3UL) * 8UL))) | \
         (((prio) & 0xFFUL) << (((uint32_t)(int32_t)IRQn) & 0x3UL) * 8UL)))
#define NVIC_GetPriority(IRQn)        ((NVIC->IPR[(uint32_t)(int32_t)IRQn >> 2UL] >> \
                                       ((((uint32_t)(int32_t)IRQn) & 0x3UL) * 8UL)) & 0xFFUL)
#define NVIC_SystemReset()            (SCB->AIRCR = (0x05FAUL << 16) | (1UL << 2))

/* ======================================================================== */
/* SCB Register Structure                                                    */
/* ======================================================================== */
typedef struct {
    __I  uint32_t CPUID;                    /*!< Offset 0x000: CPU ID            */
    __IO uint32_t ICSR;                     /*!< Offset 0x004: Int Control State*/
    uint32_t RESERVED0;
    __IO uint32_t AIRCR;                    /*!< Offset 0x00C: App Int & Reset  */
    __IO uint32_t SCR;                      /*!< Offset 0x010: System Control   */
    __IO uint32_t CCR;                      /*!< Offset 0x014: Config & Control */
    uint32_t RESERVED1;
    __IO uint32_t SHP[2U];                  /*!< Offset 0x01C: Sys Handlers Pri */
    __IO uint32_t SHCSR;                    /*!< Offset 0x024: Sys Handler Ctrl */
} SCB_Type;

/* SCB CPUID bits */
#define SCB_CPUID_IMPLEMENTER_Pos        24
#define SCB_CPUID_IMPLEMENTER_Msk       (0xFFUL << SCB_CPUID_IMPLEMENTER_Pos)
#define SCB_CPUID_VARIANT_Pos            20
#define SCB_CPUID_VARIANT_Msk           (0xFUL << SCB_CPUID_VARIANT_Pos)
#define SCB_CPUID_ARCHITECTURE_Pos       16
#define SCB_CPUID_ARCHITECTURE_Msk      (0xFUL << SCB_CPUID_ARCHITECTURE_Pos)
#define SCB_CPUID_PARTNO_Pos             4
#define SCB_CPUID_PARTNO_Msk            (0xFFFUL << SCB_CPUID_PARTNO_Pos)
#define SCB_CPUID_REVISION_Pos           0
#define SCB_CPUID_REVISION_Msk          (0xFUL << SCB_CPUID_REVISION_Pos)

/* SCB ICSR bits */
#define SCB_ICSR_PENDSVCLR_Pos           27
#define SCB_ICSR_PENDSVCLR_Msk         (1UL << SCB_ICSR_PENDSVCLR_Pos)
#define SCB_ICSR_PENDSTSET_Pos           26
#define SCB_ICSR_PENDSTSET_Msk         (1UL << SCB_ICSR_PENDSTSET_Pos)
#define SCB_ICSR_PENDSTCLR_Pos           25
#define SCB_ICSR_PENDSTCLR_Msk         (1UL << SCB_ICSR_PENDSTCLR_Pos)
#define SCB_ICSR_ISRPENDING_Pos          22
#define SCB_ICSR_ISRPENDING_Msk        (1UL << SCB_ICSR_ISRPENDING_Pos)
#define SCB_ICSR_VECTPENDING_Pos         12
#define SCB_ICSR_VECTPENDING_Msk       (0x3FFUL << SCB_ICSR_VECTPENDING_Pos)
#define SCB_ICSR_VECTACTIVE_Pos          0
#define SCB_ICSR_VECTACTIVE_Msk         (0x1FFUL << SCB_ICSR_VECTACTIVE_Pos)

/* SCB AIRCR bits */
#define SCB_AIRCR_VECTKEY_Pos            16
#define SCB_AIRCR_VECTKEY_Msk           (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)
#define SCB_AIRCR_ENDIANESS_Pos          15
#define SCB_AIRCR_ENDIANESS_Msk         (1UL << SCB_AIRCR_ENDIANESS_Pos)
#define SCB_AIRCR_SYSRESETREQ_Pos        2
#define SCB_AIRCR_SYSRESETREQ_Msk       (1UL << SCB_AIRCR_SYSRESETREQ_Pos)

/* SCB SCR bits */
#define SCB_SCR_SEVONPEND_Pos            4
#define SCB_SCR_SEVONPEND_Msk           (1UL << SCB_SCR_SEVONPEND_Pos)
#define SCB_SCR_SLEEPDEEP_Pos            2
#define SCB_SCR_SLEEPDEEP_Msk           (1UL << SCB_SCR_SLEEPDEEP_Pos)
#define SCB_SCR_SLEEPONEXIT_Pos          1
#define SCB_SCR_SLEEPONEXIT_Msk         (1UL << SCB_SCR_SLEEPONEXIT_Pos)

/* SCB CCR bits */
#define SCB_CCR_STKALIGN_Pos             9
#define SCB_CCR_STKALIGN_Msk            (1UL << SCB_CCR_STKALIGN_Pos)
#define SCB_CCR_UNALIGN_TRP_Pos          3
#define SCB_CCR_UNALIGN_TRP_Msk        (1UL << SCB_CCR_UNALIGN_TRP_Pos)

#define SCB                 ((SCB_Type *) SCB_BASE)

/* ======================================================================== */
/* System Control (SysCfg) Register Block                                    */
/* ======================================================================== */
typedef struct {
    __IO uint32_t SCSR;                     /*!< Offset 0x000: System Control    */
    __IO uint32_t CFGR1;                    /*!< Offset 0x004: Configuration 1  */
    __IO uint32_t EXTICR[4];                /*!< Offset 0x008-0x014: EXTI conf */
    __IO uint32_t CFGR2;                    /*!< Offset 0x018: Configuration 2  */
} SYSCFG_TypeDef;

/* SYSCFG CFGR1 bits */
#define SYSCFG_CFGR1_MEM_MODE_Pos         0
#define SYSCFG_CFGR1_MEM_MODE_Msk        (0x3UL << SYSCFG_CFGR1_MEM_MODE_Pos)

/* EXTI line configuration */
typedef struct {
    __IO uint32_t IMR;                      /*!< Offset 0x000: Interrupt Mask   */
    __IO uint32_t EMR;                      /*!< Offset 0x004: Event Mask       */
    __IO uint32_t RTSR;                     /*!< Offset 0x008: Rising Trig Sel  */
    __IO uint32_t FTSR;                     /*!< Offset 0x00C: Falling Trig Sel */
    __IO uint32_t SWIER;                    /*!< Offset 0x010: Soft Int Event   */
    __IO uint32_t PR;                       /*!< Offset 0x014: Pending Register */
} EXTI_TypeDef;

#define EXTI_IMR_MR0_Pos       0
#define EXTI_IMR_MR0_Msk       (1UL << EXTI_IMR_MR0_Pos)
#define EXTI_PR_PR0_Pos        0
#define EXTI_PR_PR0_Msk        (1UL << EXTI_PR_PR0_Pos)

/* ======================================================================== */
/* GPIO Register Structure                                                   */
/* ======================================================================== */
typedef struct {
    __IO uint32_t MODER;                    /*!< Offset 0x00: Mode register     */
    __IO uint32_t OTYPER;                   /*!< Offset 0x04: Output type       */
    __IO uint32_t OSPEEDR;                  /*!< Offset 0x08: Output speed      */
    __IO uint32_t PUPDR;                    /*!< Offset 0x0C: Pull-up/pull-down */
    __I  uint32_t IDR;                      /*!< Offset 0x10: Input data        */
    __IO uint32_t ODR;                      /*!< Offset 0x14: Output data       */
    __O  uint32_t BSRR;                     /*!< Offset 0x18: Bit set/reset     */
    __O  uint32_t LCKR;                     /*!< Offset 0x1C: Lock              */
    __IO uint32_t AFRL;                     /*!< Offset 0x20: Alternate func low */
    __IO uint32_t AFRH;                      /*!< Offset 0x24: Alternate func high*/
    __IO uint32_t BRR;                       /*!< Offset 0x28: Bit reset         */
} GPIO_TypeDef;

/* GPIO MODER bit definitions */
#define GPIO_MODE_INPUT    0x00UL
#define GPIO_MODE_OUTPUT   0x01UL
#define GPIO_MODE_AF       0x02UL
#define GPIO_MODE_ANALOG  0x03UL

#define GPIO_MODEER_Pos(n) (2U * (n))
#define GPIO_MODEER_Msk(n) (0x3UL << GPIO_MODEER_Pos(n))

#define GPIO_OTYPER_PP_Pos(n) (n)
#define GPIO_OTYPER_PP_Msk(n) (1UL << GPIO_OTYPER_PP_Pos(n))

#define GPIO_OSPEEDR_LOW    0x0
#define GPIO_OSPEEDR_MED    0x1
#define GPIO_OSPEEDR_HIGH   0x3

#define GPIO_PUPDR_NONE     0x0
#define GPIO_PUPDR_PULLUP   0x1
#define GPIO_PUPDR_PULLDN   0x2

#define GPIO_BSRR_BS_Pos(n) (n)
#define GPIO_BSRR_BR_Pos(n) ((n) + 16)

/* GPIO port access helpers */
#define GPIO_PIN(n)        (1UL << (n))

/* ======================================================================== */
/* RCC (Reset and Clock Control) Registers                                   */
/* ======================================================================== */
typedef struct {
    __IO uint32_t CR;                       /*!< Offset 0x00: Clock control    */
    __IO uint32_t ICSCR;                    /*!< Offset 0x04: Internal clock    */
    __IO uint32_t CFGR;                     /*!< Offset 0x08: Clock config      */
    __IO uint32_t CIR;                      /*!< Offset 0x0C: Clock interrupt   */
    __IO uint32_t APB2RSTR;                 /*!< Offset 0x10: APB2 peripheral rst*/
    __IO uint32_t APB1RSTR;                 /*!< Offset 0x14: APB1 peripheral rst*/
    __IO uint32_t IOPENR;                    /*!< Offset 0x18: I/O port enable   */
    __IO uint32_t AHBENR;                   /*!< Offset 0x1C: AHB periph enable */
    __IO uint32_t APB2ENR;                  /*!< Offset 0x20: APB2 periph enable*/
    __IO uint32_t APB1ENR;                  /*!< Offset 0x24: APB1 periph enable*/
    __IO uint32_t IOPSMENR;                 /*!< Offset 0x28: IOP sleep mode en */
    __IO uint32_t AHBSMENR;                 /*!< Offset 0x2C: AHB sleep mode en */
    __IO uint32_t APB2SMENR;                /*!< Offset 0x30: APB2 sleep mode en*/
    __IO uint32_t APB1SMENR;                 /*!< Offset 0x34: APB1 sleep mode en*/
    __IO uint32_t CCIPR;                    /*!< Offset 0x38: Clock config      */
    uint32_t RESERVED[2];                   /*!< Reserved                       */
    __IO uint32_t CSR;                      /*!< Offset 0x44: Control/status    */
} RCC_TypeDef;

/* RCC CR bits */
#define RCC_CR_HSION_Pos                 0
#define RCC_CR_HSION_Msk                (1UL << RCC_CR_HSION_Pos)
#define RCC_CR_HSIRDY_Pos                1
#define RCC_CR_HSIRDY_Msk               (1UL << RCC_CR_HSIRDY_Pos)
#define RCC_CR_HSIDIV_Pos                2
#define RCC_CR_HSIDIV_Msk               (0x3UL << RCC_CR_HSIDIV_Pos)
#define RCC_CR_HSEON_Pos                 8
#define RCC_CR_HSEON_Msk                (1UL << RCC_CR_HSEON_Pos)
#define RCC_CR_HSERDY_Pos                9
#define RCC_CR_HSERDY_Msk               (1UL << RCC_CR_HSERDY_Pos)
#define RCC_CR_CSSON_Pos                 19
#define RCC_CR_CSSON_Msk                (1UL << RCC_CR_CSSON_Pos)

/* RCC ICSCR bits */
#define RCC_ICSCR_HSITRIM_Pos            8
#define RCC_ICSCR_HSITRIM_Msk           (0x1FUL << RCC_ICSCR_HSITRIM_Pos)
#define RCC_ICSCR_HSICAL_Pos            16
#define RCC_ICSCR_HSICAL_Msk           (0xFFUL << RCC_ICSCR_HSICAL_Pos)

/* RCC CFGR bits */
#define RCC_CFGR_SW_Pos                  0
#define RCC_CFGR_SW_Msk                 (0x3UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_HSI                 (0x0UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_HSE                 (0x1UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_PLL                 (0x2UL << RCC_CFGR_SW_Pos)

#define RCC_CFGR_SWS_Pos                 2
#define RCC_CFGR_SWS_Msk                (0x3UL << RCC_CFGR_SWS_Pos)

#define RCC_CFGR_HPRE_Pos                8
#define RCC_CFGR_HPRE_Msk               (0xFUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV1              (0x0UL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV2              (0x8UL << RCC_CFGR_HPRE_Pos)

#define RCC_CFGR_PPRE_Pos                11
#define RCC_CFGR_PPRE_Msk               (0x7UL << RCC_CFGR_PPRE_Pos)
#define RCC_CFGR_PPRE_DIV1              (0x0UL << RCC_CFGR_PPRE_Pos)

/* RCC APB2 enable bits */
#define RCC_APB2ENR_TIM1EN_Pos           11
#define RCC_APB2ENR_TIM1EN_Msk          (1UL << RCC_APB2ENR_TIM1EN_Pos)
#define RCC_APB2ENR_USART1EN_Pos         14
#define RCC_APB2ENR_USART1EN_Msk        (1UL << RCC_APB2ENR_USART1EN_Pos)
#define RCC_APB2ENR_ADC1EN_Pos           9
#define RCC_APB2ENR_ADC1EN_Msk          (1UL << RCC_APB2ENR_ADC1EN_Pos)
#define RCC_APB2ENR_SPI1EN_Pos           12
#define RCC_APB2ENR_SPI1EN_Msk          (1UL << RCC_APB2ENR_SPI1EN_Pos)
#define RCC_APB2ENR_SYSCFGEN_Pos         0
#define RCC_APB2ENR_SYSCFGEN_Msk        (1UL << RCC_APB2ENR_SYSCFGEN_Pos)
#define RCC_APB2ENR_DBGMCUEN_Pos         22
#define RCC_APB2ENR_DBGMCUEN_Msk        (1UL << RCC_APB2ENR_DBGMCUEN_Pos)

/* RCC APB1 enable bits */
#define RCC_APB1ENR_TIM2EN_Pos           0
#define RCC_APB1ENR_TIM2EN_Msk          (1UL << RCC_APB1ENR_TIM2EN_Pos)
#define RCC_APB1ENR_TIM3EN_Pos           1
#define RCC_APB1ENR_TIM3EN_Msk          (1UL << RCC_APB1ENR_TIM3EN_Pos)

/* RCC AHB enable bits */
#define RCC_AHBENR_DMA1EN_Pos            0
#define RCC_AHBENR_DMA1EN_Msk           (1UL << RCC_AHBENR_DMA1EN_Pos)
#define RCC_AHBENR_FLASHEN_Pos           8
#define RCC_AHBENR_FLASHEN_Msk          (1UL << RCC_AHBENR_FLASHEN_Pos)
#define RCC_AHBENR_CRCEN_Pos             6
#define RCC_AHBENR_CRCEN_Msk            (1UL << RCC_AHBENR_CRCEN_Pos)
#define RCC_AHBENR_SRAMEN_Pos            9
#define RCC_AHBENR_SRAMEN_Msk           (1UL << RCC_AHBENR_SRAMEN_Pos)

/* RCC IOP enable bits */
#define RCC_IOPENR_GPIOAEN_Pos           0
#define RCC_IOPENR_GPIOAEN_Msk          (1UL << RCC_IOPENR_GPIOAEN_Pos)
#define RCC_IOPENR_GPIOBEN_Pos           1
#define RCC_IOPENR_GPIOBEN_Msk          (1UL << RCC_IOPENR_GPIOBEN_Pos)
#define RCC_IOPENR_GPIOCEN_Pos           2
#define RCC_IOPENR_GPIOCEN_Msk          (1UL << RCC_IOPENR_GPIOCEN_Pos)

/* RCC APB2 reset bits */
#define RCC_APB2RSTR_TIM1RST_Pos         11
#define RCC_APB2RSTR_TIM1RST_Msk        (1UL << RCC_APB2RSTR_TIM1RST_Pos)
#define RCC_APB2RSTR_USART1RST_Pos      14
#define RCC_APB2RSTR_USART1RST_Msk      (1UL << RCC_APB2RSTR_USART1RST_Pos)
#define RCC_APB2RSTR_ADC1RST_Pos         9
#define RCC_APB2RSTR_ADC1RST_Msk        (1UL << RCC_APB2RSTR_ADC1RST_Pos)
#define RCC_APB2RSTR_SPI1RST_Pos         12
#define RCC_APB2RSTR_SPI1RST_Msk        (1UL << RCC_APB2RSTR_SPI1RST_Pos)

/* RCC APB1 reset bits */
#define RCC_APB1RSTR_TIM2RST_Pos         0
#define RCC_APB1RSTR_TIM2RST_Msk        (1UL << RCC_APB1RSTR_TIM2RST_Pos)
#define RCC_APB1RSTR_TIM3RST_Pos         1
#define RCC_APB1RSTR_TIM3RST_Msk        (1UL << RCC_APB1RSTR_TIM3RST_Pos)

/* RCC CSR bits */
#define RCC_CSR_LSIRDY_Pos              1
#define RCC_CSR_LSIRDY_Msk             (1UL << RCC_CSR_LSIRDY_Pos)
#define RCC_CSR_LSION_Pos              0
#define RCC_CSR_LSION_Msk             (1UL << RCC_CSR_LSION_Pos)
#define RCC_CSR_RMVF_Pos               23
#define RCC_CSR_RMVF_Msk             (1UL << RCC_CSR_RMVF_Pos)
#define RCC_CSR_PINRSTF_Pos            26
#define RCC_CSR_PINRSTF_Msk           (1UL << RCC_CSR_PINRSTF_Pos)
#define RCC_CSR_PORRSTF_Pos            27
#define RCC_CSR_PORRSTF_Msk           (1UL << RCC_CSR_PORRSTF_Pos)
#define RCC_CSR_SFTRSTF_Pos            28
#define RCC_CSR_SFTRSTF_Msk           (1UL << RCC_CSR_SFTRSTF_Pos)
#define RCC_CSR_IWDGRSTF_Pos           29
#define RCC_CSR_IWDGRSTF_Msk          (1UL << RCC_CSR_IWDGRSTF_Pos)
#define RCC_CSR_WWDGRSTF_Pos           30
#define RCC_CSR_WWDGRSTF_Msk          (1UL << RCC_CSR_WWDGRSTF_Pos)
#define RCC_CSR_LPWRRSTF_Pos           31
#define RCC_CSR_LPWRRSTF_Msk          (1UL << RCC_CSR_LPWRRSTF_Pos)

/* ======================================================================== */
/* Flash Controller Registers                                                */
/* ======================================================================== */
typedef struct {
    __IO uint32_t ACR;                      /*!< Offset 0x00: Access control    */
    __IO uint32_t KEYR;                    /*!< Offset 0x04: Key register      */
    __IO uint32_t OPTKEYR;                  /*!< Offset 0x08: Option key         */
    __IO uint32_t SR;                       /*!< Offset 0x0C: Status             */
    __IO uint32_t CR;                       /*!< Offset 0x10: Control           */
    __IO uint32_t AR;                       /*!< Offset 0x14: Address           */
    uint32_t RESERVED;                      /*!< Reserved                       */
    __IO uint32_t OBR;                      /*!< Offset 0x1C: Option byte       */
    __IO uint32_t WRPR;                     /*!< Offset 0x20: Write protect     */
} FLASH_TypeDef;

/* Flash ACR bits */
#define FLASH_ACR_LATENCY_Pos            0
#define FLASH_ACR_LATENCY_Msk            (1UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_PRFTBE_Pos              1
#define FLASH_ACR_PRFTBE_Msk             (1UL << FLASH_ACR_PRFTBE_Pos)
#define FLASH_ACR_PRFTBS_Pos             2
#define FLASH_ACR_PRFTBS_Msk            (1UL << FLASH_ACR_PRFTBS_Pos)

/* Flash SR bits */
#define FLASH_SR_BSY_Pos                 0
#define FLASH_SR_BSY_Msk                (1UL << FLASH_SR_BSY_Pos)
#define FLASH_SR_PGERR_Pos               2
#define FLASH_SR_PGERR_Msk             (1UL << FLASH_SR_PGERR_Pos)
#define FLASH_SR_WRPRTERR_Pos            4
#define FLASH_SR_WRPRTERR_Msk         (1UL << FLASH_SR_WRPRTERR_Pos)
#define FLASH_SR_EOP_Pos                 5
#define FLASH_SR_EOP_Msk                (1UL << FLASH_SR_EOP_Pos)

/* Flash CR bits */
#define FLASH_CR_PG_Pos                  0
#define FLASH_CR_PG_Msk                 (1UL << FLASH_CR_PG_Pos)
#define FLASH_CR_PER_Pos                 1
#define FLASH_CR_PER_Msk                (1UL << FLASH_CR_PER_Pos)
#define FLASH_CR_MER_Pos                 2
#define FLASH_CR_MER_Msk                (1UL << FLASH_CR_MER_Pos)
#define FLASH_CR_OPTPG_Pos               4
#define FLASH_CR_OPTPG_Msk             (1UL << FLASH_CR_OPTPG_Pos)
#define FLASH_CR_OPTER_Pos               5
#define FLASH_CR_OPTER_Msk             (1UL << FLASH_CR_OPTER_Pos)
#define FLASH_CR_STRT_Pos                6
#define FLASH_CR_STRT_Msk               (1UL << FLASH_CR_STRT_Pos)
#define FLASH_CR_LOCK_Pos                7
#define FLASH_CR_LOCK_Msk               (1UL << FLASH_CR_LOCK_Pos)
#define FLASH_CR_OPTWRE_Pos              9
#define FLASH_CR_OPTWRE_Msk             (1UL << FLASH_CR_OPTWRE_Pos)
#define FLASH_CR_ERRIE_Pos               10
#define FLASH_CR_ERRIE_Msk             (1UL << FLASH_CR_ERRIE_Pos)
#define FLASH_CR_EOPIE_Pos               12
#define FLASH_CR_EOPIE_Msk             (1UL << FLASH_CR_EOPIE_Pos)

/* Flash keys */
#define FLASH_KEY1                       (0x45670123UL)
#define FLASH_KEY2                       (0xCDEF89ABUL)
#define FLASH_OPTKEY1                    (0x45670123UL)
#define FLASH_OPTKEY2                    (0xCDEF89ABUL)

/* ======================================================================== */
/* ADC Register Structure (12-bit, up to 8 channels, DMA support)            */
/* ======================================================================== */
typedef struct {
    __IO uint32_t ISR;                      /*!< Offset 0x00: Int status reg    */
    __IO uint32_t IER;                      /*!< Offset 0x04: Int enable reg    */
    __IO uint32_t CR;                       /*!< Offset 0x08: Control reg       */
    __IO uint32_t CFGR1;                    /*!< Offset 0x0C: Config reg 1       */
    __IO uint32_t CFGR2;                    /*!< Offset 0x10: Config reg 2       */
    __IO uint32_t SMPR;                     /*!< Offset 0x14: Sampling time      */
    uint32_t RESERVED1[2];                  /*!< Reserved 0x18-0x1C             */
    __IO uint32_t AWD1TR;                   /*!< Offset 0x20: Watchdog threshold*/
    uint32_t RESERVED2;                     /*!< Reserved 0x24                  */
    __IO uint32_t CHSELR;                   /*!< Offset 0x28: Channel select    */
    uint32_t RESERVED3[2];                  /*!< Reserved 0x2C-0x30             */
    __IO uint32_t DR;                       /*!< Offset 0x40: Regular data       */
    uint32_t RESERVED4[6];                  /*!< Reserved 0x44-0x5B             */
    __IO uint32_t CCR;                      /*!< Offset 0x308: Common control   */
} ADC_TypeDef;

/* ADC ISR bits */
#define ADC_ISR_ADRDY_Pos                0
#define ADC_ISR_ADRDY_Msk               (1UL << ADC_ISR_ADRDY_Pos)
#define ADC_ISR_EOSMP_Pos                1
#define ADC_ISR_EOSMP_Msk               (1UL << ADC_ISR_EOSMP_Pos)
#define ADC_ISR_EOC_Pos                  2
#define ADC_ISR_EOC_Msk                 (1UL << ADC_ISR_EOC_Pos)
#define ADC_ISR_EOS_Pos                  3
#define ADC_ISR_EOS_Msk                 (1UL << ADC_ISR_EOS_Pos)
#define ADC_ISR_OVR_Pos                  4
#define ADC_ISR_OVR_Msk                 (1UL << ADC_ISR_OVR_Pos)
#define ADC_ISR_AWD1_Pos                 7
#define ADC_ISR_AWD1_Msk                (1UL << ADC_ISR_AWD1_Pos)

/* ADC IER bits (mirror ISR bits) */
#define ADC_IER_ADRDYIE_Pos              0
#define ADC_IER_ADRDYIE_Msk             (1UL << ADC_IER_ADRDYIE_Pos)
#define ADC_IER_EOSMPIE_Pos              1
#define ADC_IER_EOSMPIE_Msk             (1UL << ADC_IER_EOSMPIE_Pos)
#define ADC_IER_EOCIE_Pos                2
#define ADC_IER_EOCIE_Msk               (1UL << ADC_IER_EOCIE_Pos)
#define ADC_IER_EOSIE_Pos                3
#define ADC_IER_EOSIE_Msk               (1UL << ADC_IER_EOSIE_Pos)
#define ADC_IER_OVRIE_Pos                4
#define ADC_IER_OVRIE_Msk               (1UL << ADC_IER_OVRIE_Pos)
#define ADC_IER_AWD1IE_Pos               7
#define ADC_IER_AWD1IE_Msk              (1UL << ADC_IER_AWD1IE_Pos)

/* ADC CR bits */
#define ADC_CR_ADEN_Pos                  0
#define ADC_CR_ADEN_Msk                 (1UL << ADC_CR_ADEN_Pos)
#define ADC_CR_ADDIS_Pos                 1
#define ADC_CR_ADDIS_Msk                (1UL << ADC_CR_ADDIS_Pos)
#define ADC_CR_ADSTART_Pos               2
#define ADC_CR_ADSTART_Msk              (1UL << ADC_CR_ADSTART_Pos)
#define ADC_CR_ADSTP_Pos                 3
#define ADC_CR_ADSTP_Msk                (1UL << ADC_CR_ADSTP_Pos)
#define ADC_CR_ADVREGEN_Pos              4
#define ADC_CR_ADVREGEN_Msk             (1UL << ADC_CR_ADVREGEN_Pos)
#define ADC_CR_ADCAL_Pos                 6
#define ADC_CR_ADCAL_Msk                (1UL << ADC_CR_ADCAL_Pos)

/* ADC CFGR1 bits */
#define ADC_CFGR1_DMAEN_Pos              0
#define ADC_CFGR1_DMAEN_Msk             (1UL << ADC_CFGR1_DMAEN_Pos)
#define ADC_CFGR1_DMACFG_Pos             1
#define ADC_CFGR1_DMACFG_Msk            (1UL << ADC_CFGR1_DMACFG_Pos)
#define ADC_CFGR1_SCANDIR_Pos            2
#define ADC_CFGR1_SCANDIR_Msk           (1UL << ADC_CFGR1_SCANDIR_Pos)
#define ADC_CFGR1_RES_Pos                3
#define ADC_CFGR1_RES_Msk               (0x3UL << ADC_CFGR1_RES_Pos)
#define ADC_CFGR1_RES_12BIT             (0x0UL << ADC_CFGR1_RES_Pos)
#define ADC_CFGR1_RES_10BIT             (0x1UL << ADC_CFGR1_RES_Pos)
#define ADC_CFGR1_RES_8BIT              (0x2UL << ADC_CFGR1_RES_Pos)
#define ADC_CFGR1_RES_6BIT              (0x3UL << ADC_CFGR1_RES_Pos)
#define ADC_CFGR1_ALIGN_Pos              5
#define ADC_CFGR1_ALIGN_Msk             (1UL << ADC_CFGR1_ALIGN_Pos)
#define ADC_CFGR1_EXTSEL_Pos             6
#define ADC_CFGR1_EXTSEL_Msk            (0xFUL << ADC_CFGR1_EXTSEL_Pos)
#define ADC_CFGR1_EXTEN_Pos              10
#define ADC_CFGR1_EXTEN_Msk             (0x3UL << ADC_CFGR1_EXTEN_Pos)
#define ADC_CFGR1_OVRMOD_Pos             12
#define ADC_CFGR1_OVRMOD_Msk            (1UL << ADC_CFGR1_OVRMOD_Pos)
#define ADC_CFGR1_CONT_Pos               13
#define ADC_CFGR1_CONT_Msk              (1UL << ADC_CFGR1_CONT_Pos)
#define ADC_CFGR1_WAIT_Pos               14
#define ADC_CFGR1_WAIT_Msk              (1UL << ADC_CFGR1_WAIT_Pos)
#define ADC_CFGR1_AWDEN_Pos              23
#define ADC_CFGR1_AWDEN_Msk             (1UL << ADC_CFGR1_AWDEN_Pos)

/* ADC CFGR2 bits */
#define ADC_CFGR2_CKMODE_Pos             30
#define ADC_CFGR2_CKMODE_Msk            (0x3UL << ADC_CFGR2_CKMODE_Pos)

/* ADC SMPR bits */
#define ADC_SMPR_SMP_Pos                 0
#define ADC_SMPR_SMP_Msk                (0x7UL << ADC_SMPR_SMP_Pos)
#define ADC_SMPR_SMP_1_5                (0x0UL << ADC_SMPR_SMP_Pos)
#define ADC_SMPR_SMP_3_5                (0x1UL << ADC_SMPR_SMP_Pos)
#define ADC_SMPR_SMP_7_5                (0x2UL << ADC_SMPR_SMP_Pos)
#define ADC_SMPR_SMP_12_5               (0x3UL << ADC_SMPR_SMP_Pos)
#define ADC_SMPR_SMP_19_5               (0x4UL << ADC_SMPR_SMP_Pos)
#define ADC_SMPR_SMP_39_5               (0x5UL << ADC_SMPR_SMP_Pos)
#define ADC_SMPR_SMP_79_5               (0x6UL << ADC_SMPR_SMP_Pos)
#define ADC_SMPR_SMP_160_5              (0x7UL << ADC_SMPR_SMP_Pos)

/* ADC CHSELR bits - one bit per channel */
#define ADC_CHSELR_CHSEL0_Pos            0
#define ADC_CHSELR_CHSEL0_Msk           (1UL << ADC_CHSELR_CHSEL0_Pos)
#define ADC_CHSELR_CHSEL1_Pos            1
#define ADC_CHSELR_CHSEL1_Msk           (1UL << ADC_CHSELR_CHSEL1_Pos)
#define ADC_CHSELR_CHSEL2_Pos            2
#define ADC_CHSELR_CHSEL2_Msk           (1UL << ADC_CHSELR_CHSEL2_Pos)
#define ADC_CHSELR_CHSEL3_Pos            3
#define ADC_CHSELR_CHSEL3_Msk           (1UL << ADC_CHSELR_CHSEL3_Pos)
#define ADC_CHSELR_CHSEL4_Pos            4
#define ADC_CHSELR_CHSEL4_Msk           (1UL << ADC_CHSELR_CHSEL4_Pos)
#define ADC_CHSELR_CHSEL5_Pos            5
#define ADC_CHSELR_CHSEL5_Msk           (1UL << ADC_CHSELR_CHSEL5_Pos)
#define ADC_CHSELR_CHSEL6_Pos            6
#define ADC_CHSELR_CHSEL6_Msk           (1UL << ADC_CHSELR_CHSEL6_Pos)
#define ADC_CHSELR_CHSEL7_Pos            7
#define ADC_CHSELR_CHSEL7_Msk           (1UL << ADC_CHSELR_CHSEL7_Pos)

/* ADC CCR bits */
#define ADC_CCR_VREFEN_Pos               22
#define ADC_CCR_VREFEN_Msk              (1UL << ADC_CCR_VREFEN_Pos)
#define ADC_CCR_TSEN_Pos                 23
#define ADC_CCR_TSEN_Msk                (1UL << ADC_CCR_TSEN_Pos)
#define ADC_CCR_VBATEN_Pos               24
#define ADC_CCR_VBATEN_Msk              (1UL << ADC_CCR_VBATEN_Pos)

/* ---- ADC channel definitions (used by board_config.h) ---- */
/*! Channel index helpers used by board_config.h.
 *  Channel 0  -> PB0  (IC_ia - phase A current)
 *  Channel 1  -> PB1  (IC_ib - phase B current)
 *  Channel 2  -> PA2  (SWCLK - not used as ADC)
 *  Channel 3  -> PA3  (Vol_U)
 *  Channel 4  -> PA4  (Vol_V)
 *  Channel 5  -> PA5  (Vol_W)
 *  Channel 6  -> PA6  (MotorTemp)
 *  Channel 7  -> PA7  (Voltage/Bus)
 */
#define ADC_CH0                          ADC_CHSELR_CHSEL0_Msk
#define ADC_CH1                          ADC_CHSELR_CHSEL1_Msk
#define ADC_CH2                          ADC_CHSELR_CHSEL2_Msk
#define ADC_CH3                          ADC_CHSELR_CHSEL3_Msk
#define ADC_CH4                          ADC_CHSELR_CHSEL4_Msk
#define ADC_CH5                          ADC_CHSELR_CHSEL5_Msk
#define ADC_CH6                          ADC_CHSELR_CHSEL6_Msk
#define ADC_CH7                          ADC_CHSELR_CHSEL7_Msk

/* ADC conversion helper macros */
#define ADC_ENABLE(ADCx)                 (SET_BIT((ADCx)->CR, ADC_CR_ADEN_Msk))
#define ADC_DISABLE(ADCx)                (SET_BIT((ADCx)->CR, ADC_CR_ADDIS_Msk))
#define ADC_START_CONV(ADCx)             (SET_BIT((ADCx)->CR, ADC_CR_ADSTART_Msk))
#define ADC_STOP_CONV(ADCx)              (SET_BIT((ADCx)->CR, ADC_CR_ADSTP_Msk))

/* ======================================================================== */
/* TIM1 - Advanced Control Timer (3-phase PWM w/ complementary & dead-time)   */
/* ======================================================================== */
typedef struct {
    __IO uint32_t CR1;                      /*!< Offset 0x00: Control 1          */
    __IO uint32_t CR2;                     /*!< 0x04: Control 2                */
    __IO uint32_t SMCR;                    /*!< 0x08: Slave mode control       */
    __IO uint32_t DIER;                    /*!< 0x0C: DMA/interrupt enable     */
    __IO uint32_t SR;                       /*!< 0x10: Status                   */
    __IO uint32_t EGR;                      /*!< 0x14: Event generation         */
    __IO uint32_t CCMR1;                    /*!< 0x18: Capture/compare mode 1   */
    __IO uint32_t CCMR2;                    /*!< 0x1C: Capture/compare mode 2   */
    __IO uint32_t CCER;                     /*!< 0x20: Capture/compare enable   */
    __IO uint32_t CNT;                      /*!< 0x24: Counter                 */
    __IO uint32_t PSC;                      /*!< 0x28: Prescaler               */
    __IO uint32_t ARR;                      /*!< 0x2C: Auto-reload             */
    __IO uint32_t RCR;                      /*!< 0x30: Repetition counter      */
    __IO uint32_t CCR1;                     /*!< 0x34: Capture/compare 1       */
    __IO uint32_t CCR2;                     /*!< 0x38: Capture/compare 2       */
    __IO uint32_t CCR3;                     /*!< 0x3C: Capture/compare 3       */
    __IO uint32_t CCR4;                     /*!< 0x40: Capture/compare 4       */
    __IO uint32_t BDTR;                     /*!< 0x44: Break & dead-time        */
    __IO uint32_t DCR;                      /*!< 0x48: DMA control             */
    __IO uint32_t DMAR;                     /*!< 0x4C: DMA address for full transfer */
    __IO uint32_t OR;                      /*!< 0x50: Option register 1       */
    __IO uint32_t AF1;                      /*!< 0x60: Alternate function 1    */
    __IO uint32_t AF2;                      /*!< 0x64: Alternate function 2    */
    uint32_t RESERVED[4];                   /*!< Reserved                       */
    __IO uint32_t TISEL;                    /*!< 0x68: Input selection         */
} TIM1_TypeDef;

/* TIM1 CR1 bits */
#define TIM_CR1_CEN_Pos                  0
#define TIM_CR1_CEN_Msk                 (1UL << TIM_CR1_CEN_Pos)
#define TIM_CR1_UDIS_Pos                 1
#define TIM_CR1_UDIS_Msk                (1UL << TIM_CR1_UDIS_Pos)
#define TIM_CR1_URS_Pos                  2
#define TIM_CR1_URS_Msk                 (1UL << TIM_CR1_URS_Pos)
#define TIM_CR1_OPM_Pos                  3
#define TIM_CR1_OPM_Msk                 (1UL << TIM_CR1_OPM_Pos)
#define TIM_CR1_DIR_Pos                  4
#define TIM_CR1_DIR_Msk                 (1UL << TIM_CR1_DIR_Pos)
#define TIM_CR1_CMS_Pos                  5
#define TIM_CR1_CMS_Msk                 (0x3UL << TIM_CR1_CMS_Pos)
#define TIM_CR1_CMS_EDGE                (0x0UL << TIM_CR1_CMS_Pos)
#define TIM_CR1_CMS_CENTER1             (0x1UL << TIM_CR1_CMS_Pos)
#define TIM_CR1_CMS_CENTER2             (0x2UL << TIM_CR1_CMS_Pos)
#define TIM_CR1_CMS_CENTER3             (0x3UL << TIM_CR1_CMS_Pos)
#define TIM_CR1_ARPE_Pos                 7
#define TIM_CR1_ARPE_Msk                (1UL << TIM_CR1_ARPE_Pos)
#define TIM_CR1_CKD_Pos                  8
#define TIM_CR1_CKD_Msk                 (0x3UL << TIM_CR1_CKD_Pos)

/* TIM1 CR2 bits */
#define TIM_CR2_CCPC_Pos                 0
#define TIM_CR2_CCPC_Msk                (1UL << TIM_CR2_CCPC_Pos)
#define TIM_CR2_CCUS_Pos                 2
#define TIM_CR2_CCUS_Msk                (1UL << TIM_CR2_CCUS_Pos)
#define TIM_CR2_CCDS_Pos                 3
#define TIM_CR2_CCDS_Msk                (1UL << TIM_CR2_CCDS_Pos)
#define TIM_CR2_MMS_Pos                  4
#define TIM_CR2_MMS_Msk                 (0x7UL << TIM_CR2_MMS_Pos)
#define TIM_CR2_TI1S_Pos                 7
#define TIM_CR2_TI1S_Msk                (1UL << TIM_CR2_TI1S_Pos)
#define TIM_CR2_OIS1_Pos                 8
#define TIM_CR2_OIS1_Msk                (1UL << TIM_CR2_OIS1_Pos)
#define TIM_CR2_OIS1N_Pos                9
#define TIM_CR2_OIS1N_Msk               (1UL << TIM_CR2_OIS1N_Pos)
#define TIM_CR2_OIS2_Pos                 10
#define TIM_CR2_OIS2_Msk                (1UL << TIM_CR2_OIS2_Pos)
#define TIM_CR2_OIS2N_Pos                11
#define TIM_CR2_OIS2N_Msk               (1UL << TIM_CR2_OIS2N_Pos)
#define TIM_CR2_OIS3_Pos                 12
#define TIM_CR2_OIS3_Msk                (1UL << TIM_CR2_OIS3_Pos)
#define TIM_CR2_OIS3N_Pos                13
#define TIM_CR2_OIS3N_Msk               (1UL << TIM_CR2_OIS3N_Pos)
#define TIM_CR2_OIS4_Pos                 14
#define TIM_CR2_OIS4_Msk                (1UL << TIM_CR2_OIS4_Pos)

/* TIM1 SMCR bits */
#define TIM_SMCR_SMS_Pos                 0
#define TIM_SMCR_SMS_Msk                (0x7UL << TIM_SMCR_SMS_Pos)
#define TIM_SMCR_TS_Pos                  4
#define TIM_SMCR_TS_Msk                 (0x7UL << TIM_SMCR_TS_Pos)
#define TIM_SMCR_MSM_Pos                 7
#define TIM_SMCR_MSM_Msk                (1UL << TIM_SMCR_MSM_Pos)

/* TIM1 DIER bits */
#define TIM_DIER_UIE_Pos                 0
#define TIM_DIER_UIE_Msk                (1UL << TIM_DIER_UIE_Pos)
#define TIM_DIER_CC1IE_Pos               1
#define TIM_DIER_CC1IE_Msk              (1UL << TIM_DIER_CC1IE_Pos)
#define TIM_DIER_CC2IE_Pos               2
#define TIM_DIER_CC2IE_Msk              (1UL << TIM_DIER_CC2IE_Pos)
#define TIM_DIER_CC3IE_Pos               3
#define TIM_DIER_CC3IE_Msk              (1UL << TIM_DIER_CC3IE_Pos)
#define TIM_DIER_CC4IE_Pos               4
#define TIM_DIER_CC4IE_Msk              (1UL << TIM_DIER_CC4IE_Pos)
#define TIM_DIER_COMIE_Pos               5
#define TIM_DIER_COMIE_Msk              (1UL << TIM_DIER_COMIE_Pos)
#define TIM_DIER_TIE_Pos                 6
#define TIM_DIER_TIE_Msk                (1UL << TIM_DIER_TIE_Pos)
#define TIM_DIER_BIE_Pos                 7
#define TIM_DIER_BIE_Msk                (1UL << TIM_DIER_BIE_Pos)
#define TIM_DIER_UDE_Pos                 8
#define TIM_DIER_UDE_Msk                (1UL << TIM_DIER_UDE_Pos)
#define TIM_DIER_CC1DE_Pos               9
#define TIM_DIER_CC1DE_Msk              (1UL << TIM_DIER_CC1DE_Pos)
#define TIM_DIER_CC2DE_Pos               10
#define TIM_DIER_CC2DE_Msk              (1UL << TIM_DIER_CC2DE_Pos)
#define TIM_DIER_CC3DE_Pos               11
#define TIM_DIER_CC3DE_Msk              (1UL << TIM_DIER_CC3DE_Pos)
#define TIM_DIER_CC4DE_Pos               12
#define TIM_DIER_CC4DE_Msk              (1UL << TIM_DIER_CC4DE_Pos)
#define TIM_DIER_COMDE_Pos               13
#define TIM_DIER_COMDE_Msk              (1UL << TIM_DIER_COMDE_Pos)
#define TIM_DIER_TDE_Pos                 14
#define TIM_DIER_TDE_Msk                (1UL << TIM_DIER_TDE_Pos)

/* TIM1 SR bits */
#define TIM_SR_UIF_Pos                   0
#define TIM_SR_UIF_Msk                  (1UL << TIM_SR_UIF_Pos)
#define TIM_SR_CC1IF_Pos                 1
#define TIM_SR_CC1IF_Msk                (1UL << TIM_SR_CC1IF_Pos)
#define TIM_SR_CC2IF_Pos                 2
#define TIM_SR_CC2IF_Msk                (1UL << TIM_SR_CC2IF_Pos)
#define TIM_SR_CC3IF_Pos                 3
#define TIM_SR_CC3IF_Msk                (1UL << TIM_SR_CC3IF_Pos)
#define TIM_SR_CC4IF_Pos                 4
#define TIM_SR_CC4IF_Msk                (1UL << TIM_SR_CC4IF_Pos)
#define TIM_SR_COMIF_Pos                 5
#define TIM_SR_COMIF_Msk                (1UL << TIM_SR_COMIF_Pos)
#define TIM_SR_TIF_Pos                   6
#define TIM_SR_TIF_Msk                  (1UL << TIM_SR_TIF_Pos)
#define TIM_SR_BIF_Pos                   7
#define TIM_SR_BIF_Msk                  (1UL << TIM_SR_BIF_Pos)
#define TIM_SR_CC1OF_Pos                 9
#define TIM_SR_CC1OF_Msk                (1UL << TIM_SR_CC1OF_Pos)
#define TIM_SR_CC2OF_Pos                 10
#define TIM_SR_CC2OF_Msk                (1UL << TIM_SR_CC2OF_Pos)
#define TIM_SR_CC3OF_Pos                 11
#define TIM_SR_CC3OF_Msk                (1UL << TIM_SR_CC3OF_Pos)
#define TIM_SR_CC4OF_Pos                 12
#define TIM_SR_CC4OF_Msk                (1UL << TIM_SR_CC4OF_Pos)

/* TIM1 EGR bits */
#define TIM_EGR_UG_Pos                   0
#define TIM_EGR_UG_Msk                  (1UL << TIM_EGR_UG_Pos)
#define TIM_EGR_CC1G_Pos                 1
#define TIM_EGR_CC1G_Msk                (1UL << TIM_EGR_CC1G_Pos)
#define TIM_EGR_CC2G_Pos                 2
#define TIM_EGR_CC2G_Msk                (1UL << TIM_EGR_CC2G_Pos)
#define TIM_EGR_CC3G_Pos                 3
#define TIM_EGR_CC3G_Msk                (1UL << TIM_EGR_CC3G_Pos)
#define TIM_EGR_CC4G_Pos                 4
#define TIM_EGR_CC4G_Msk                (1UL << TIM_EGR_CC4G_Pos)
#define TIM_EGR_COMG_Pos                 5
#define TIM_EGR_COMG_Msk                (1UL << TIM_EGR_COMG_Pos)
#define TIM_EGR_TG_Pos                   6
#define TIM_EGR_TG_Msk                  (1UL << TIM_EGR_TG_Pos)
#define TIM_EGR_BG_Pos                   7
#define TIM_EGR_BG_Msk                  (1UL << TIM_EGR_BG_Pos)

/* TIM1 CCMR1 - output compare mode bits (used for PWM mode) */
#define TIM_CCMR1_CC1S_Pos               0
#define TIM_CCMR1_CC1S_Msk              (0x3UL << TIM_CCMR1_CC1S_Pos)
#define TIM_CCMR1_OC1FE_Pos              2
#define TIM_CCMR1_OC1FE_Msk             (1UL << TIM_CCMR1_OC1FE_Pos)
#define TIM_CCMR1_OC1PE_Pos              3
#define TIM_CCMR1_OC1PE_Msk             (1UL << TIM_CCMR1_OC1PE_Pos)
#define TIM_CCMR1_OC1M_Pos               4
#define TIM_CCMR1_OC1M_Msk              (0x7UL << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1M_FROZEN            (0x0UL << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1M_ACTIVE            (0x1UL << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1M_INACTIVE           (0x2UL << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1M_TOGGLE             (0x3UL << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1M_FORCE_INACTIVE     (0x4UL << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1M_FORCE_ACTIVE       (0x5UL << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1M_PWM1               (0x6UL << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1M_PWM2               (0x7UL << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1CE_Pos               7
#define TIM_CCMR1_OC1CE_Msk              (1UL << TIM_CCMR1_OC1CE_Pos)

#define TIM_CCMR1_CC2S_Pos               8
#define TIM_CCMR1_CC2S_Msk              (0x3UL << TIM_CCMR1_CC2S_Pos)
#define TIM_CCMR1_OC2FE_Pos              10
#define TIM_CCMR1_OC2FE_Msk             (1UL << TIM_CCMR1_OC2FE_Pos)
#define TIM_CCMR1_OC2PE_Pos              11
#define TIM_CCMR1_OC2PE_Msk             (1UL << TIM_CCMR1_OC2PE_Pos)
#define TIM_CCMR1_OC2M_Pos               12
#define TIM_CCMR1_OC2M_Msk              (0x7UL << TIM_CCMR1_OC2M_Pos)
#define TIM_CCMR1_OC2M_PWM1              (0x6UL << TIM_CCMR1_OC2M_Pos)
#define TIM_CCMR1_OC2M_PWM2              (0x7UL << TIM_CCMR1_OC2M_Pos)
#define TIM_CCMR1_OC2CE_Pos              15
#define TIM_CCMR1_OC2CE_Msk             (1UL << TIM_CCMR1_OC2CE_Pos)

/* TIM1 CCMR2 - output compare mode for channels 3 & 4 */
#define TIM_CCMR2_CC3S_Pos               0
#define TIM_CCMR2_CC3S_Msk              (0x3UL << TIM_CCMR2_CC3S_Pos)
#define TIM_CCMR2_OC3PE_Pos              3
#define TIM_CCMR2_OC3PE_Msk             (1UL << TIM_CCMR2_OC3PE_Pos)
#define TIM_CCMR2_OC3M_Pos               4
#define TIM_CCMR2_OC3M_Msk              (0x7UL << TIM_CCMR2_OC3M_Pos)
#define TIM_CCMR2_OC3M_PWM1              (0x6UL << TIM_CCMR2_OC3M_Pos)
#define TIM_CCMR2_OC3M_PWM2              (0x7UL << TIM_CCMR2_OC3M_Pos)

#define TIM_CCMR2_CC4S_Pos               8
#define TIM_CCMR2_CC4S_Msk              (0x3UL << TIM_CCMR2_CC4S_Pos)
#define TIM_CCMR2_OC4PE_Pos              11
#define TIM_CCMR2_OC4PE_Msk             (1UL << TIM_CCMR2_OC4PE_Pos)
#define TIM_CCMR2_OC4M_Pos               12
#define TIM_CCMR2_OC4M_Msk              (0x7UL << TIM_CCMR2_OC4M_Pos)
#define TIM_CCMR2_OC4M_PWM1              (0x6UL << TIM_CCMR2_OC4M_Pos)
#define TIM_CCMR2_OC4M_PWM2              (0x7UL << TIM_CCMR2_OC4M_Pos)

/* TIM1 CCER bits - capture/compare + complementary output enable */
#define TIM_CCER_CC1E_Pos                0
#define TIM_CCER_CC1E_Msk               (1UL << TIM_CCER_CC1E_Pos)
#define TIM_CCER_CC1P_Pos                1
#define TIM_CCER_CC1P_Msk               (1UL << TIM_CCER_CC1P_Pos)
#define TIM_CCER_CC1NE_Pos               2
#define TIM_CCER_CC1NE_Msk              (1UL << TIM_CCER_CC1NE_Pos)
#define TIM_CCER_CC1NP_Pos               3
#define TIM_CCER_CC1NP_Msk              (1UL << TIM_CCER_CC1NP_Pos)

#define TIM_CCER_CC2E_Pos                4
#define TIM_CCER_CC2E_Msk               (1UL << TIM_CCER_CC2E_Pos)
#define TIM_CCER_CC2P_Pos                5
#define TIM_CCER_CC2P_Msk               (1UL << TIM_CCER_CC2P_Pos)
#define TIM_CCER_CC2NE_Pos               6
#define TIM_CCER_CC2NE_Msk              (1UL << TIM_CCER_CC2NE_Pos)
#define TIM_CCER_CC2NP_Pos               7
#define TIM_CCER_CC2NP_Msk              (1UL << TIM_CCER_CC2NP_Pos)

#define TIM_CCER_CC3E_Pos                8
#define TIM_CCER_CC3E_Msk               (1UL << TIM_CCER_CC3E_Pos)
#define TIM_CCER_CC3P_Pos                9
#define TIM_CCER_CC3P_Msk               (1UL << TIM_CCER_CC3P_Pos)
#define TIM_CCER_CC3NE_Pos               10
#define TIM_CCER_CC3NE_Msk              (1UL << TIM_CCER_CC3NE_Pos)
#define TIM_CCER_CC3NP_Pos               11
#define TIM_CCER_CC3NP_Msk              (1UL << TIM_CCER_CC3NP_Pos)

#define TIM_CCER_CC4E_Pos                12
#define TIM_CCER_CC4E_Msk               (1UL << TIM_CCER_CC4E_Pos)
#define TIM_CCER_CC4P_Pos                13
#define TIM_CCER_CC4P_Msk               (1UL << TIM_CCER_CC4P_Pos)

/* TIM1 BDTR bits - break & dead-time (critical for 3-phase PWM) */
#define TIM_BDTR_DTG_Pos                 0
#define TIM_BDTR_DTG_Msk                (0xFFUL << TIM_BDTR_DTG_Pos)
#define TIM_BDTR_LOCK_Pos                8
#define TIM_BDTR_LOCK_Msk               (0x3UL << TIM_BDTR_LOCK_Pos)
#define TIM_BDTR_OSSI_Pos                10
#define TIM_BDTR_OSSI_Msk               (1UL << TIM_BDTR_OSSI_Pos)
#define TIM_BDTR_OSSR_Pos                11
#define TIM_BDTR_OSSR_Msk               (1UL << TIM_BDTR_OSSR_Pos)
#define TIM_BDTR_BKE_Pos                 12
#define TIM_BDTR_BKE_Msk                (1UL << TIM_BDTR_BKE_Pos)
#define TIM_BDTR_BKP_Pos                 13
#define TIM_BDTR_BKP_Msk                (1UL << TIM_BDTR_BKP_Pos)
#define TIM_BDTR_AOE_Pos                 14
#define TIM_BDTR_AOE_Msk                (1UL << TIM_BDTR_AOE_Pos)
#define TIM_BDTR_MOE_Pos                 15
#define TIM_BDTR_MOE_Msk                (1UL << TIM_BDTR_MOE_Pos)
#define TIM_BDTR_BKF_Pos                 16
#define TIM_BDTR_BKF_Msk                (0xFUL << TIM_BDTR_BKF_Pos)

/* TIM1 DCR/DMAR bits */
#define TIM_DCR_DBA_Pos                  0
#define TIM_DCR_DBA_Msk                 (0x1FUL << TIM_DCR_DBA_Pos)
#define TIM_DCR_DBL_Pos                  8
#define TIM_DCR_DBL_Msk                 (0x1FUL << TIM_DCR_DBL_Pos)

/* TIM1 enable helper macros */
#define TIM1_ENABLE()                     (SET_BIT(TIM1->CR1, TIM_CR1_CEN_Msk))
#define TIM1_DISABLE()                    (CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN_Msk))
#define TIM1_ENABLE_PWM_OUTPUT()          (SET_BIT(TIM1->BDTR, TIM_BDTR_MOE_Msk))
#define TIM1_DISABLE_PWM_OUTPUT()         (CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE_Msk))

/* ======================================================================== */
/* TIM2 / TIM3 - General Purpose Timers (used for PPM capture, etc.)         */
/* ======================================================================== */
typedef struct {
    __IO uint32_t CR1;                      /*!< Offset 0x00: Control 1          */
    __IO uint32_t CR2;                      /*!< 0x04: Control 2                */
    __IO uint32_t SMCR;                     /*!< 0x08: Slave mode control       */
    __IO uint32_t DIER;                     /*!< 0x0C: DMA/interrupt enable     */
    __IO uint32_t SR;                       /*!< 0x10: Status                   */
    __IO uint32_t EGR;                      /*!< 0x14: Event generation         */
    __IO uint32_t CCMR1;                    /*!< 0x18: Capture/compare mode 1   */
    __IO uint32_t CCMR2;                    /*!< 0x1C: Capture/compare mode 2   */
    __IO uint32_t CCER;                     /*!< 0x20: Capture/compare enable   */
    __IO uint32_t CNT;                      /*!< 0x24: Counter                 */
    __IO uint32_t PSC;                      /*!< 0x28: Prescaler               */
    __IO uint32_t ARR;                      /*!< 0x2C: Auto-reload             */
    __IO uint32_t CCR1;                     /*!< 0x30: Capture/compare 1       */
    __IO uint32_t CCR2;                     /*!< 0x34: Capture/compare 2       */
    __IO uint32_t CCR3;                     /*!< 0x38: Capture/compare 3       */
    __IO uint32_t CCR4;                     /*!< 0x3C: Capture/compare 4       */
    __IO uint32_t DCR;                      /*!< 0x40: DMA control             */
    __IO uint32_t DMAR;                     /*!< 0x44: DMA address for full transfer */
    __IO uint32_t OR;                       /*!< 0x48: Option register         */
    __IO uint32_t AF1;                      /*!< 0x50: Alternate function 1    */
    __IO uint32_t TISEL;                    /*!< 0x68: Input selection         */
} TIM_TypeDef;

/* TIM2/3 use same CR1/CR2/DIER/SR/EGR/CCER bit definitions as TIM1.
 * Generic macros (named TIM_xxx) are shared with TIM1 by design. */

/* TIM2/3 capture/compare input filter helpers (for PPM capture) */
#define TIM_CCMR1_CC1S_INPUT_TI1        (0x1UL << TIM_CCMR1_CC1S_Pos)
#define TIM_CCMR1_CC1S_INPUT_TI2        (0x2UL << TIM_CCMR1_CC1S_Pos)
#define TIM_CCMR1_IC1F_Pos              4
#define TIM_CCMR1_IC1F_Msk             (0xFUL << TIM_CCMR1_IC1F_Pos)
#define TIM_CCMR1_IC1PSC_Pos            2
#define TIM_CCMR1_IC1PSC_Msk          (0x3UL << TIM_CCMR1_IC1PSC_Pos)

/* TIM2/3 enable helper macros */
#define TIM2_ENABLE()                    (SET_BIT(TIM2->CR1, TIM_CR1_CEN_Msk))
#define TIM2_DISABLE()                   (CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN_Msk))
#define TIM3_ENABLE()                    (SET_BIT(TIM3->CR1, TIM_CR1_CEN_Msk))
#define TIM3_DISABLE()                   (CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN_Msk))

/* ======================================================================== */
/* DMA Controller Registers                                                  */
/* ======================================================================== */
typedef struct {
    __IO uint32_t ISR;                     /*!< Offset 0x00: Int status        */
    __IO uint32_t IFCR;                     /*!< Offset 0x04: Int flag clear    */
} DMA_Common_TypeDef;

typedef struct {
    __IO uint32_t CCR;                      /*!< Offset 0x00: Channel config   */
    __IO uint32_t CNDTR;                    /*!< 0x04: Number of data to transfer*/
    __IO uint32_t CPAR;                     /*!< 0x08: Peripheral address      */
    __IO uint32_t CMAR;                     /*!< 0x0C: Memory address          */
} DMA_Channel_TypeDef;

typedef struct {
    DMA_Common_TypeDef Common;             /*!< Common status registers       */
    DMA_Channel_TypeDef Channel1;          /*!< Channel 1 (typically ADC)    */
    DMA_Channel_TypeDef Channel2;          /*!< Channel 2                    */
    DMA_Channel_TypeDef Channel3;          /*!< Channel 3                    */
    DMA_Channel_TypeDef Channel4;          /*!< Channel 4                    */
    DMA_Channel_TypeDef Channel5;          /*!< Channel 5                    */
} DMA_TypeDef;

/* DMA ISR/IFCR bits */
#define DMA_ISR_GIF1_Pos                0
#define DMA_ISR_GIF1_Msk               (1UL << DMA_ISR_GIF1_Pos)
#define DMA_ISR_TCIF1_Pos               1
#define DMA_ISR_TCIF1_Msk              (1UL << DMA_ISR_TCIF1_Pos)
#define DMA_ISR_HTIF1_Pos               2
#define DMA_ISR_HTIF1_Msk              (1UL << DMA_ISR_HTIF1_Pos)
#define DMA_ISR_TEIF1_Pos               3
#define DMA_ISR_TEIF1_Msk              (1UL << DMA_ISR_TEIF1_Pos)

#define DMA1_Channel1    ((DMA_Channel_TypeDef *) (DMA1_BASE + 0x08UL))
#define DMA1_Channel2    ((DMA_Channel_TypeDef *) (DMA1_BASE + 0x1CUL))
#define DMA1_Channel3    ((DMA_Channel_TypeDef *) (DMA1_BASE + 0x30UL))
#define DMA1_Channel4    ((DMA_Channel_TypeDef *) (DMA1_BASE + 0x44UL))
#define DMA1_Channel5    ((DMA_Channel_TypeDef *) (DMA1_BASE + 0x58UL))

/* DMA CCR bits */
#define DMA_CCR_EN_Pos                  0
#define DMA_CCR_EN_Msk                 (1UL << DMA_CCR_EN_Pos)
#define DMA_CCR_TCIE_Pos                1
#define DMA_CCR_TCIE_Msk               (1UL << DMA_CCR_TCIE_Pos)
#define DMA_CCR_HTIE_Pos                2
#define DMA_CCR_HTIE_Msk               (1UL << DMA_CCR_HTIE_Pos)
#define DMA_CCR_TEIE_Pos                3
#define DMA_CCR_TEIE_Msk               (1UL << DMA_CCR_TEIE_Pos)
#define DMA_CCR_DIR_Pos                 4
#define DMA_CCR_DIR_Msk                (1UL << DMA_CCR_DIR_Pos)
#define DMA_CCR_CIRC_Pos                5
#define DMA_CCR_CIRC_Msk               (1UL << DMA_CCR_CIRC_Pos)
#define DMA_CCR_PINC_Pos                6
#define DMA_CCR_PINC_Msk               (1UL << DMA_CCR_PINC_Pos)
#define DMA_CCR_MINC_Pos                7
#define DMA_CCR_MINC_Msk               (1UL << DMA_CCR_MINC_Pos)
#define DMA_CCR_PSIZE_Pos               8
#define DMA_CCR_PSIZE_Msk              (0x3UL << DMA_CCR_PSIZE_Pos)
#define DMA_CCR_MSIZE_Pos               10
#define DMA_CCR_MSIZE_Msk              (0x3UL << DMA_CCR_MSIZE_Pos)
#define DMA_CCR_PL_Pos                  12
#define DMA_CCR_PL_Msk                 (0x3UL << DMA_CCR_PL_Pos)
#define DMA_CCR_MEM2MEM_Pos             14
#define DMA_CCR_MEM2MEM_Msk            (1UL << DMA_CCR_MEM2MEM_Pos)

/* DMA size values */
#define DMA_CCR_PSIZE_8BIT              (0x0UL << DMA_CCR_PSIZE_Pos)
#define DMA_CCR_PSIZE_16BIT             (0x1UL << DMA_CCR_PSIZE_Pos)
#define DMA_CCR_PSIZE_32BIT             (0x2UL << DMA_CCR_PSIZE_Pos)
#define DMA_CCR_MSIZE_8BIT              (0x0UL << DMA_CCR_MSIZE_Pos)
#define DMA_CCR_MSIZE_16BIT             (0x1UL << DMA_CCR_MSIZE_Pos)
#define DMA_CCR_MSIZE_32BIT             (0x2UL << DMA_CCR_MSIZE_Pos)

/* DMA priority levels */
#define DMA_CCR_PL_LOW                  (0x0UL << DMA_CCR_PL_Pos)
#define DMA_CCR_PL_MEDIUM               (0x1UL << DMA_CCR_PL_Pos)
#define DMA_CCR_PL_HIGH                 (0x2UL << DMA_CCR_PL_Pos)
#define DMA_CCR_PL_VERY_HIGH            (0x3UL << DMA_CCR_PL_Pos)

/* DMA1 channel mapping for CIU32F003x5 */
#define DMA1_REQ_ADC1                    0U   /*!< ADC1 on DMA1 channel 1        */
#define DMA1_REQ_TIM1_UP                 13U  /*!< TIM1 update                  */
#define DMA1_REQ_TIM1_CH1                14U  /*!< TIM1 CC1                     */
#define DMA1_REQ_TIM2_UP                 3U   /*!< TIM2 update                  */
#define DMA1_REQ_USART1_TX               9U
#define DMA1_REQ_USART1_RX               10U
#define DMA1_REQ_SPI1_TX                 11U
#define DMA1_REQ_SPI1_RX                 10U

/* DMA address for ADC1 regular data register (for FOC ADC DMA) */
#define ADC1_DR_ADDR                     ((uint32_t)(&(ADC1->DR)))

/* ======================================================================== */
/* USART Register Structure                                                  */
/* ======================================================================== */
typedef struct {
    __IO uint32_t CR1;                      /*!< Offset 0x00: Control 1          */
    __IO uint32_t CR2;                      /*!< 0x04: Control 2                */
    __IO uint32_t CR3;                      /*!< 0x08: Control 3                */
    __IO uint32_t BRR;                      /*!< 0x0C: Baud rate                */
    uint32_t RESERVED;                      /*!< Reserved                       */
    __IO uint32_t RQR;                      /*!< 0x14: Request                   */
    __IO uint32_t ISR;                      /*!< 0x18: Int status                */
    __IO uint32_t ICR;                      /*!< 0x1C: Int flag clear             */
    __IO uint32_t RDR;                      /*!< 0x20: Receive data              */
    __IO uint32_t TDR;                      /*!< 0x24: Transmit data             */
    __IO uint32_t PRESC;                    /*!< 0x28: Prescaler                */
} USART_TypeDef;

/* USART CR1 bits */
#define USART_CR1_UE_Pos                 0
#define USART_CR1_UE_Msk                (1UL << USART_CR1_UE_Pos)
#define USART_CR1_RE_Pos                 2
#define USART_CR1_RE_Msk                (1UL << USART_CR1_RE_Pos)
#define USART_CR1_TE_Pos                 3
#define USART_CR1_TE_Msk                (1UL << USART_CR1_TE_Pos)
#define USART_CR1_IDLEIE_Pos             4
#define USART_CR1_IDLEIE_Msk            (1UL << USART_CR1_IDLEIE_Pos)
#define USART_CR1_RXNEIE_Pos             5
#define USART_CR1_RXNEIE_Msk            (1UL << USART_CR1_RXNEIE_Pos)
#define USART_CR1_TCIE_Pos               6
#define USART_CR1_TCIE_Msk              (1UL << USART_CR1_TCIE_Pos)
#define USART_CR1_TXEIE_Pos              7
#define USART_CR1_TXEIE_Msk             (1UL << USART_CR1_TXEIE_Pos)
#define USART_CR1_PEIE_Pos               8
#define USART_CR1_PEIE_Msk              (1UL << USART_CR1_PEIE_Pos)
#define USART_CR1_PS_Pos                 9
#define USART_CR1_PS_Msk                (1UL << USART_CR1_PS_Pos)
#define USART_CR1_PCE_Pos                10
#define USART_CR1_PCE_Msk               (1UL << USART_CR1_PCE_Pos)
#define USART_CR1_WAKE_Pos               11
#define USART_CR1_WAKE_Msk              (1UL << USART_CR1_WAKE_Pos)
#define USART_CR1_M_Pos                  12
#define USART_CR1_M_Msk                 (1UL << USART_CR1_M_Pos)
#define USART_CR1_MME_Pos                13
#define USART_CR1_MME_Msk               (1UL << USART_CR1_MME_Pos)
#define USART_CR1_OVER8_Pos              15
#define USART_CR1_OVER8_Msk             (1UL << USART_CR1_OVER8_Pos)
#define USART_CR1_DEDT_Pos                16
#define USART_CR1_DEDT_Msk              (0x1FUL << USART_CR1_DEDT_Pos)
#define USART_CR1_DEAT_Pos                21
#define USART_CR1_DEAT_Msk              (0x1FUL << USART_CR1_DEAT_Pos)

/* USART CR2 bits */
#define USART_CR2_SLVEN_Pos              0
#define USART_CR2_SLVEN_Msk             (1UL << USART_CR2_SLVEN_Pos)
#define USART_CR2_DISCARD_Pos            5
#define USART_CR2_DISCARD_Msk           (1UL << USART_CR2_DISCARD_Pos)
#define USART_CR2_STOP_Pos               12
#define USART_CR2_STOP_Msk              (0x3UL << USART_CR2_STOP_Pos)
#define USART_CR2_STOP_1BIT             (0x0UL << USART_CR2_STOP_Pos)
#define USART_CR2_STOP_2BIT             (0x2UL << USART_CR2_STOP_Pos)
#define USART_CR2_SWAP_Pos               15
#define USART_CR2_SWAP_Msk              (1UL << USART_CR2_SWAP_Pos)
#define USART_CR2_RXINV_Pos              16
#define USART_CR2_RXINV_Msk             (1UL << USART_CR2_RXINV_Pos)
#define USART_CR2_TXINV_Pos              17
#define USART_CR2_TXINV_Msk             (1UL << USART_CR2_TXINV_Pos)
#define USART_CR2_DATAINV_Pos            18
#define USART_CR2_DATAINV_Msk           (1UL << USART_CR2_DATAINV_Pos)
#define USART_CR2_MSBFIRST_Pos           19
#define USART_CR2_MSBFIRST_Msk          (1UL << USART_CR2_MSBFIRST_Pos)
#define USART_CR2_ABREN_Pos              20
#define USART_CR2_ABREN_Msk             (1UL << USART_CR2_ABREN_Pos)

/* USART CR3 bits */
#define USART_CR3_EIE_Pos                0
#define USART_CR3_EIE_Msk               (1UL << USART_CR3_EIE_Pos)
#define USART_CR3_DMAR_Pos               6
#define USART_CR3_DMAR_Msk              (1UL << USART_CR3_DMAR_Pos)
#define USART_CR3_DMAT_Pos               7
#define USART_CR3_DMAT_Msk              (1UL << USART_CR3_DMAT_Pos)
#define USART_CR3_RTSE_Pos               8
#define USART_CR3_RTSE_Msk              (1UL << USART_CR3_RTSE_Pos)
#define USART_CR3_CTSE_Pos               9
#define USART_CR3_CTSE_Msk              (1UL << USART_CR3_CTSE_Pos)
#define USART_CR3_CTSIE_Pos              10
#define USART_CR3_CTSIE_Msk             (1UL << USART_CR3_CTSIE_Pos)
#define USART_CR3_ONEBIT_Pos             11
#define USART_CR3_ONEBIT_Msk            (1UL << USART_CR3_ONEBIT_Pos)
#define USART_CR3_OVRDIS_Pos             12
#define USART_CR3_OVRDIS_Msk            (1UL << USART_CR3_OVRDIS_Pos)

/* USART ISR bits */
#define USART_ISR_PE_Pos                 0
#define USART_ISR_PE_Msk                (1UL << USART_ISR_PE_Pos)
#define USART_ISR_FE_Pos                 1
#define USART_ISR_FE_Msk                (1UL << USART_ISR_FE_Pos)
#define USART_ISR_NE_Pos                 2
#define USART_ISR_NE_Msk                (1UL << USART_ISR_NE_Pos)
#define USART_ISR_ORE_Pos                3
#define USART_ISR_ORE_Msk               (1UL << USART_ISR_ORE_Pos)
#define USART_ISR_IDLE_Pos               4
#define USART_ISR_IDLE_Msk              (1UL << USART_ISR_IDLE_Pos)
#define USART_ISR_RXNE_Pos                5
#define USART_ISR_RXNE_Msk               (1UL << USART_ISR_RXNE_Pos)
#define USART_ISR_TC_Pos                 6
#define USART_ISR_TC_Msk                (1UL << USART_ISR_TC_Pos)
#define USART_ISR_TXE_Pos                7
#define USART_ISR_TXE_Msk               (1UL << USART_ISR_TXE_Pos)
#define USART_ISR_CMF_Pos                17
#define USART_ISR_CMF_Msk               (1UL << USART_ISR_CMF_Pos)
#define USART_ISR_BUSY_Pos               25
#define USART_ISR_BUSY_Msk              (1UL << USART_ISR_BUSY_Pos)
#define USART_ISR_ABRF_Pos               15
#define USART_ISR_ABRF_Msk              (1UL << USART_ISR_ABRF_Pos)

/* USART helper macros */
#define USART_ENABLE(USARTx)            (SET_BIT((USARTx)->CR1, USART_CR1_UE_Msk))
#define USART_DISABLE(USARTx)           (CLEAR_BIT((USARTx)->CR1, USART_CR1_UE_Msk))
#define USART_CLEAR_FLAG(USARTx, FLAG)  (WRITE_REG((USARTx)->ICR, (FLAG)))

/* ======================================================================== */
/* SPI Register Structure (for future use)                                   */
/* ======================================================================== */
typedef struct {
    __IO uint32_t CR1;                      /*!< Offset 0x00: Control 1          */
    __IO uint32_t CR2;                      /*!< 0x04: Control 2                */
    __IO uint32_t SR;                       /*!< 0x08: Status                   */
    __IO uint32_t DR;                       /*!< 0x0C: Data                     */
    __IO uint32_t CRCPR;                    /*!< 0x10: CRC polynomial           */
    __IO uint32_t RXCRCR;                   /*!< 0x14: RX CRC                   */
    __IO uint32_t TXCRCR;                   /*!< 0x18: TX CRC                   */
    __IO uint32_t I2SCFGR;                  /*!< 0x1C: I2S config               */
    __IO uint32_t I2SPR;                     /*!< 0x20: I2S prescaler            */
} SPI_TypeDef;

/* SPI CR1 bits */
#define SPI_CR1_CPHA_Pos                 0
#define SPI_CR1_CPHA_Msk                (1UL << SPI_CR1_CPHA_Pos)
#define SPI_CR1_CPOL_Pos                 1
#define SPI_CR1_CPOL_Msk                (1UL << SPI_CR1_CPOL_Pos)
#define SPI_CR1_MSTR_Pos                 2
#define SPI_CR1_MSTR_Msk                (1UL << SPI_CR1_MSTR_Pos)
#define SPI_CR1_BR_Pos                   3
#define SPI_CR1_BR_Msk                  (0x7UL << SPI_CR1_BR_Pos)
#define SPI_CR1_SPE_Pos                  6
#define SPI_CR1_SPE_Msk                 (1UL << SPI_CR1_SPE_Pos)
#define SPI_CR1_LSBFIRST_Pos             7
#define SPI_CR1_LSBFIRST_Msk            (1UL << SPI_CR1_LSBFIRST_Pos)
#define SPI_CR1_SSI_Pos                  8
#define SPI_CR1_SSI_Msk                 (1UL << SPI_CR1_SSI_Pos)
#define SPI_CR1_SSM_Pos                  9
#define SPI_CR1_SSM_Msk                 (1UL << SPI_CR1_SSM_Pos)
#define SPI_CR1_RXONLY_Pos               10
#define SPI_CR1_RXONLY_Msk              (1UL << SPI_CR1_RXONLY_Pos)
#define SPI_CR1_DFF_Pos                  11
#define SPI_CR1_DFF_Msk                 (1UL << SPI_CR1_DFF_Pos)
#define SPI_CR1_CRCNEXT_Pos              12
#define SPI_CR1_CRCNEXT_Msk             (1UL << SPI_CR1_CRCNEXT_Pos)
#define SPI_CR1_CRCEN_Pos                13
#define SPI_CR1_CRCEN_Msk               (1UL << SPI_CR1_CRCEN_Pos)
#define SPI_CR1_BIDIOE_Pos               14
#define SPI_CR1_BIDIOE_Msk              (1UL << SPI_CR1_BIDIOE_Pos)
#define SPI_CR1_BIDIMODE_Pos             15
#define SPI_CR1_BIDIMODE_Msk            (1UL << SPI_CR1_BIDIMODE_Pos)

/* SPI CR2 bits */
#define SPI_CR2_RXDMAEN_Pos              0
#define SPI_CR2_RXDMAEN_Msk             (1UL << SPI_CR2_RXDMAEN_Pos)
#define SPI_CR2_TXDMAEN_Pos              1
#define SPI_CR2_TXDMAEN_Msk             (1UL << SPI_CR2_TXDMAEN_Pos)
#define SPI_CR2_SSOE_Pos                 2
#define SPI_CR2_SSOE_Msk                (1UL << SPI_CR2_SSOE_Pos)
#define SPI_CR2_NSSP_Pos                 3
#define SPI_CR2_NSSP_Msk                (1UL << SPI_CR2_NSSP_Pos)
#define SPI_CR2_FRF_Pos                  4
#define SPI_CR2_FRF_Msk                 (1UL << SPI_CR2_FRF_Pos)
#define SPI_CR2_ERRIE_Pos                5
#define SPI_CR2_ERRIE_Msk               (1UL << SPI_CR2_ERRIE_Pos)
#define SPI_CR2_RXNEIE_Pos               6
#define SPI_CR2_RXNEIE_Msk              (1UL << SPI_CR2_RXNEIE_Pos)
#define SPI_CR2_TXEIE_Pos                7
#define SPI_CR2_TXEIE_Msk               (1UL << SPI_CR2_TXEIE_Pos)
#define SPI_CR2_DS_Pos                   8
#define SPI_CR2_DS_Msk                  (0xFUL << SPI_CR2_DS_Pos)
#define SPI_CR2_FRXTH_Pos                12
#define SPI_CR2_FRXTH_Msk               (1UL << SPI_CR2_FRXTH_Pos)
#define SPI_CR2_LDMA_RX_Pos              13
#define SPI_CR2_LDMA_RX_Msk             (1UL << SPI_CR2_LDMA_RX_Pos)
#define SPI_CR2_LDMA_TX_Pos              14
#define SPI_CR2_LDMA_TX_Msk             (1UL << SPI_CR2_LDMA_TX_Pos)

/* SPI SR bits */
#define SPI_SR_RXNE_Pos                  0
#define SPI_SR_RXNE_Msk                 (1UL << SPI_SR_RXNE_Pos)
#define SPI_SR_TXE_Pos                   1
#define SPI_SR_TXE_Msk                  (1UL << SPI_SR_TXE_Pos)
#define SPI_SR_CHSIDE_Pos                2
#define SPI_SR_CHSIDE_Msk               (1UL << SPI_SR_CHSIDE_Pos)
#define SPI_SR_UDR_Pos                   3
#define SPI_SR_UDR_Msk                  (1UL << SPI_SR_UDR_Pos)
#define SPI_SR_CRCERR_Pos                4
#define SPI_SR_CRCERR_Msk               (1UL << SPI_SR_CRCERR_Pos)
#define SPI_SR_MODF_Pos                  5
#define SPI_SR_MODF_Msk                 (1UL << SPI_SR_MODF_Pos)
#define SPI_SR_OVR_Pos                   6
#define SPI_SR_OVR_Msk                  (1UL << SPI_SR_OVR_Pos)
#define SPI_SR_BSY_Pos                   7
#define SPI_SR_BSY_Msk                  (1UL << SPI_SR_BSY_Pos)
#define SPI_SR_FRE_Pos                   8
#define SPI_SR_FRE_Msk                  (1UL << SPI_SR_FRE_Pos)
#define SPI_SR_FRLVL_Pos                 9
#define SPI_SR_FRLVL_Msk               (0x3UL << SPI_SR_FRLVL_Pos)
#define SPI_SR_FTLVL_Pos                 11
#define SPI_SR_FTLVL_Msk               (0x3UL << SPI_SR_FTLVL_Pos)

/* SPI helper macros */
#define SPI_ENABLE(SPIx)                (SET_BIT((SPIx)->CR1, SPI_CR1_SPE_Msk))
#define SPI_DISABLE(SPIx)               (CLEAR_BIT((SPIx)->CR1, SPI_CR1_SPE_Msk))

/* ======================================================================== */
/* CRC Register Structure                                                    */
/* ======================================================================== */
typedef struct {
    __IO uint32_t DR;                      /*!< Offset 0x00: Data register      */
    __IO uint32_t IDR;                     /*!< 0x04: Independent data         */
    __IO uint32_t CR;                       /*!< 0x08: Control                  */
    uint32_t RESERVED;                      /*!< Reserved                       */
    __IO uint32_t INIT;                    /*!< 0x10: Initial CRC value        */
    __IO uint32_t POL;                     /*!< 0x14: Polynomial               */
} CRC_TypeDef;

/* ======================================================================== */
/* IRQ Handler prototypes (weak references)                                  */
/* ======================================================================== */
/* Core exception handlers */
void NMI_Handler(void)         __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)         __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)     __attribute__((weak, alias("Default_Handler")));

/* Peripheral interrupt handlers */
void WWDG_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void RTC_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void EXTI0_1_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void EXTI2_3_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void EXTI4_15_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel2_3_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel4_5_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void ADC1_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));

void Default_Handler(void);              /*!< Default handler (infinite loop) */

/* ======================================================================== */
/* FOC-Specific Pin Mapping Aliases (from board_config.h)                    */
/* ======================================================================== */
/* These aliases document the WFOC V1 board mapping used in the firmware.    */
/* PWM channels on TIM1:                                                     */
/*   - TIM1_CH1  / TIM1_CH1N : Phase C  (PA0/PA1, PWM_CL/PWM_CH)            */
/*   - TIM1_CH2  / TIM1_CH2N : Phase A  (PB2/PB3, PWM_AH/PWM_AL)            */
/*   - TIM1_CH3  / TIM1_CH3N : Phase B  (PC1/PB7, PWM_BH/PWM_BL)           */
/* ADC channels (current sensing):                                          */
/*   - ADC_CH0 (PB0) : Phase A current (IC_ia)                              */
/*   - ADC_CH1 (PB1) : Phase B current (IC_ib)                              */
/* ======================================================================== */

/* External crystal / clock configuration (CIU32F003x5 default) */
#define HSI_VALUE            (8000000UL)   /*!< Internal 8 MHz RC oscillator  */
#define HSE_VALUE            (24000000UL)  /*!< External 24 MHz crystal (opt) */
#define LSI_VALUE            (40000UL)     /*!< Internal 40 kHz low-speed RC   */
#define LSE_VALUE            (32768UL)     /*!< External 32.768 kHz crystal    */

/* System clock frequency */
#define SystemCoreClock      (24000000UL)  /*!< 24 MHz system clock            */

/* Convenience: number of interrupt vector entries in the vector table.
 * The 16+32 entries below reflect the Cortex-M0+ vector table size used
 * by CIU32F003x5. Startup code may use this to size the vector table. */
#define CIU32_VECTOR_TABLE_SIZE  (32U)

/** @} */ /* end of group CIU32F003X */

#ifdef __cplusplus
}
#endif

#endif /* CIU32F003X_H */
