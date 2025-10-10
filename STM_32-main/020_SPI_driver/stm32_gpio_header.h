#ifndef STM32L7XX_GPIO_H
#define STM32L7XX_GPIO_H

#include <stdint.h>

#define __vo volatile

typedef struct
{
    __vo uint32_t MODER;    /*               offset 0x00 */
    __vo uint32_t OTYPER;   /*               offset 0x04 */
    __vo uint32_t OSPEEDR;  /*               offset 0x08 */
    __vo uint32_t PUPDR;    /*               offset 0x0C */
    __vo uint32_t IDR;      /*               offset 0x10 */
    __vo uint32_t ODR;      /*               offset 0x14 */
    __vo uint32_t BSRR;     /*               offset 0x18 */
    __vo uint32_t LCKR;     /*               offset 0x1C */
    __vo uint32_t AFRL;     /*               offset 0x20 */
    __vo uint32_t AFRH;     /*               offset 0x24 */
    __vo uint32_t BRR;      /*               offset 0x28 */
} GPIO_RegDef_t;

#ifndef STM32L4_RCC_DEFS_H
#define STM32L4_RCC_DEFS_H

#include <stdint.h>
#define __vo volatile

#define PERIPH_BASE       0x40000000UL
#define AHB1PERIPH_BASE   (PERIPH_BASE + 0x00020000UL)   // 0x40020000
#define RCC_BASE          (AHB1PERIPH_BASE + 0x00001000UL) // 0x40021000

typedef struct {
    __vo uint32_t CR;            // 0x00 Clock control register
    __vo uint32_t ICSCR;         // 0x04 Internal clock sources calibration register
    __vo uint32_t CFGR;          // 0x08 Clock configuration register
    __vo uint32_t PLLCFGR;       // 0x0C PLL configuration register
    __vo uint32_t PLLSAI1CFGR;   // 0x10 PLL SAI1 register
    __vo uint32_t PLLSAI2CFGR;   // 0x14 PLL SAI2 register
    uint32_t RESERVED0;          // 0x18 Reserved
    __vo uint32_t CIER;          // 0x18 Clock interrupt enable register
    __vo uint32_t CIFR;          // 0x1C Clock interrupt flag register
    __vo uint32_t CICR;          // 0x20 Clock interrupt clear register
    uint32_t RESERVED1;          // 0x24 Reserved
    __vo uint32_t AHB1RSTR;      // 0x28 AHB1 peripheral reset register
    __vo uint32_t AHB2RSTR;      // 0x2C AHB2 peripheral reset register
    __vo uint32_t AHB3RSTR;      // 0x30 AHB3 peripheral reset register
    uint32_t RESERVED2;          // 0x34 Reserved
    __vo uint32_t APB1RSTR1;     // 0x38 APB1 reset register 1
    __vo uint32_t APB1RSTR2;     // 0x3C APB1 reset register 2
    __vo uint32_t APB2RSTR;      // 0x40 APB2 reset register
    uint32_t RESERVED3;          // 0x44 Reserved
    __vo uint32_t AHB1ENR;       // 0x48 AHB1 clock enable register
    __vo uint32_t AHB2ENR;       // 0x4C AHB2 clock enable register
    __vo uint32_t AHB3ENR;       // 0x50 AHB3 clock enable register
    uint32_t RESERVED4;          // 0x54 Reserved
    __vo uint32_t APB1ENR1;      // 0x58 APB1 clock enable register 1
    __vo uint32_t APB1ENR2;      // 0x5C APB1 clock enable register 2
    __vo uint32_t APB2ENR;       // 0x60 APB2 clock enable register
    uint32_t RESERVED5;          // 0x64 Reserved
    __vo uint32_t AHB1SMENR;     // 0x68 AHB1 clocks enable in Sleep/Stop
    __vo uint32_t AHB2SMENR;     // 0x6C AHB2 clocks enable in Sleep/Stop
    __vo uint32_t AHB3SMENR;     // 0x70 AHB3 clocks enable in Sleep/Stop
    uint32_t RESERVED6;          // 0x74 Reserved
    __vo uint32_t APB1SMENR1;    // 0x78 APB1 clocks enable in Sleep/Stop 1
    __vo uint32_t APB1SMENR2;    // 0x7C APB1 clocks enable in Sleep/Stop 2
    __vo uint32_t APB2SMENR;     // 0x80 APB2 clocks enable in Sleep/Stop
    uint32_t RESERVED7;          // 0x84 Reserved
    __vo uint32_t CCIPR1;        // 0x88 Peripherals independent clock configuration 1
    __vo uint32_t BDCR;          // 0x90 Backup domain control register
    __vo uint32_t CSR;           // 0x94 Control/status register
    __vo uint32_t CRRCR;         // 0x98 Clock recovery RC register
    __vo uint32_t CCIPR2;        // 0x9C Peripherals independent clock configuration 2
} RCC_RegDef_t;

#define RCC  ((RCC_RegDef_t *) RCC_BASE)

#define RCC_AHB1ENR_GPIOAEN   (1 << 0)
#define RCC_AHB1ENR_GPIOBEN   (1 << 1)
#define RCC_AHB1ENR_GPIOCEN   (1 << 2)
#define RCC_AHB1ENR_GPIODEN   (1 << 3)
#define RCC_AHB1ENR_GPIOEEN   (1 << 4)
#define RCC_AHB1ENR_GPIOFEN   (1 << 5)
#define RCC_AHB1ENR_GPIOGEN   (1 << 6)
#define RCC_AHB1ENR_GPIOHEN   (1 << 7)
#define RCC_AHB1ENR_GPIOIEN   (1 << 8)

#define RCC_APB1ENR1_USART2EN (1 << 17)
#define RCC_APB2ENR_USART1EN  (1 << 14)
#define RCC_APB2ENR_SPI1EN    (1 << 12)

#define RCC_AHB1RSTR_GPIOARST   (1 << 0)
#define RCC_AHB1RSTR_GPIOBRST   (1 << 1)
#define RCC_AHB1RSTR_GPIOCRST   (1 << 2)
#define RCC_AHB1RSTR_GPIODRST   (1 << 3)
#define RCC_AHB1RSTR_GPIOERST   (1 << 4)

#endif /* STM32L4_RCC_DEFS_H */


#define PERIPH_BASE          (0x40000000UL)
#define AHB1PERIPH_BASE      (PERIPH_BASE + 0x00020000UL)

#define GPIOA_BASE           (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE           (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE           (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE           (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE           (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE           (AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE           (AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE           (AHB1PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASE           (AHB1PERIPH_BASE + 0x2000UL)

#define GPIOA                ((GPIO_RegDef_t *) GPIOA_BASE)
#define GPIOB                ((GPIO_RegDef_t *) GPIOB_BASE)
#define GPIOC                ((GPIO_RegDef_t *) GPIOC_BASE)
#define GPIOD                ((GPIO_RegDef_t *) GPIOD_BASE)
#define GPIOE                ((GPIO_RegDef_t *) GPIOE_BASE)
#define GPIOF                ((GPIO_RegDef_t *) GPIOF_BASE)
#define GPIOG                ((GPIO_RegDef_t *) GPIOG_BASE)
#define GPIOH                ((GPIO_RegDef_t *) GPIOH_BASE)
#define GPIOI                ((GPIO_RegDef_t *) GPIOI_BASE)


/* MODER bits (2 bits per pin) */
#define GPIO_MODER_INPUT     (0x0U)
#define GPIO_MODER_OUTPUT    (0x1U)
#define GPIO_MODER_ALT       (0x2U)
#define GPIO_MODER_ANALOG    (0x3U)

/* OTYPER bits */
#define GPIO_OTYPER_PP       (0x0U)
#define GPIO_OTYPER_OD       (0x1U)

/* OSPEEDR bits */
#define GPIO_OSPEED_LOW      (0x0U)
#define GPIO_OSPEED_MEDIUM   (0x1U)
#define GPIO_OSPEED_HIGH     (0x2U)
#define GPIO_OSPEED_VERYHIGH (0x3U)

/* PUPDR bits */
#define GPIO_PUPDR_NONE      (0x0U)
#define GPIO_PUPDR_PU        (0x1U)
#define GPIO_PUPDR_PD        (0x2U)

/* BSRR bit macros */
#define GPIO_BSRR_SET(pin)   (1U << (pin))
#define GPIO_BSRR_RESET(pin) (1U << ((pin) + 16U))

#endif /* STM32L7XX_GPIO_H */
