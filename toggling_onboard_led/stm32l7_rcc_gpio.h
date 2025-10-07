#ifndef STM32L7_RCC_GPIO_H
#define STM32L7_RCC_GPIO_H

#include <stdint.h>
#define __vo volatile
#define ENABLE 1
#define DISABLE 0

#define PERIPH_BASE       0x40000000UL
#define AHB1PERIPH_BASE   (PERIPH_BASE + 0x00020000UL)   // 0x40020000
#define RCC_BASE          (AHB1PERIPH_BASE + 0x00001000UL) // 0x40021000

typedef struct
{
    __vo uint32_t CR;            // 0x00 Clock control register
    __vo uint32_t ICSCR;         // 0x04 Internal clock calibration
    __vo uint32_t CFGR;          // 0x08 Clock config register
    __vo uint32_t PLLCFGR;       // 0x0C PLL config register
    __vo uint32_t PLLSAI1CFGR;   // 0x10 PLL SAI1 config
    __vo uint32_t PLLSAI2CFGR;   // 0x14 PLL SAI2 config
    uint32_t RESERVED0;          
    __vo uint32_t CIER;          
    __vo uint32_t CIFR;          
    __vo uint32_t CICR;          
    uint32_t RESERVED1;          
    __vo uint32_t AHB1RSTR;      
    __vo uint32_t AHB2RSTR;      
    __vo uint32_t AHB3RSTR;      
    uint32_t RESERVED2;          
    __vo uint32_t APB1RSTR1;     
    __vo uint32_t APB1RSTR2;     
    __vo uint32_t APB2RSTR;      
    uint32_t RESERVED3;          
    __vo uint32_t AHB1ENR;       // 0x48 AHB1 clock enable
    __vo uint32_t AHB2ENR;       
    __vo uint32_t AHB3ENR;       
    uint32_t RESERVED4;          
    __vo uint32_t APB1ENR1;      
    __vo uint32_t APB1ENR2;      
    __vo uint32_t APB2ENR;       
} RCC_RegDef_t;

#define RCC ((RCC_RegDef_t *) RCC_BASE)

/* RCC AHB1ENR bits for GPIO (AHB1) */
#define RCC_AHB1ENR_GPIOAEN   (1 << 0)
#define RCC_AHB1ENR_GPIOBEN   (1 << 1)
#define RCC_AHB1ENR_GPIOCEN   (1 << 2)
#define RCC_AHB1ENR_GPIODEN   (1 << 3)
#define RCC_AHB1ENR_GPIOEEN   (1 << 4)
#define RCC_AHB1ENR_GPIOFEN   (1 << 5)
#define RCC_AHB1ENR_GPIOGEN   (1 << 6)
#define RCC_AHB1ENR_GPIOHEN   (1 << 7)
#define RCC_AHB1ENR_GPIOIEN   (1 << 8)

/* Enable macros */
#define GPIOA_CLK_EN() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define GPIOB_CLK_EN() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN)
#define GPIOC_CLK_EN() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN)
#define GPIOD_CLK_EN() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN)
#define GPIOE_CLK_EN() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN)
#define GPIOF_CLK_EN() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN)
#define GPIOG_CLK_EN() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN)
#define GPIOH_CLK_EN() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN)
#define GPIOI_CLK_EN() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN)


/* GPIO Register Structure */
typedef struct
{
    __vo uint32_t MODER;
    __vo uint32_t OTYPER;
    __vo uint32_t OSPEEDR;
    __vo uint32_t PUPDR;
    __vo uint32_t IDR;
    __vo uint32_t ODR;
    __vo uint32_t BSRR;
    __vo uint32_t LCKR;
    __vo uint32_t AFRL;
    __vo uint32_t AFRH;
    __vo uint32_t BRR;
} GPIO_RegDef_t;

/* GPIO Base Addresses (AHB1) */
#define GPIOA_BASE  0x40020000UL
#define GPIOB_BASE  0x40020400UL
#define GPIOC_BASE  0x40020800UL
#define GPIOD_BASE  0x40020C00UL
#define GPIOE_BASE  0x40021000UL
#define GPIOF_BASE  0x40021400UL
#define GPIOG_BASE  0x40021800UL
#define GPIOH_BASE  0x40021C00UL
#define GPIOI_BASE  0x40022000UL

/* GPIO Peripheral Definitions */
#define GPIOA   ((GPIO_RegDef_t *) GPIOA_BASE)
#define GPIOB   ((GPIO_RegDef_t *) GPIOB_BASE)
#define GPIOC   ((GPIO_RegDef_t *) GPIOC_BASE)
#define GPIOD   ((GPIO_RegDef_t *) GPIOD_BASE)
#define GPIOE   ((GPIO_RegDef_t *) GPIOE_BASE)
#define GPIOF   ((GPIO_RegDef_t *) GPIOF_BASE)
#define GPIOG   ((GPIO_RegDef_t *) GPIOG_BASE)
#define GPIOH   ((GPIO_RegDef_t *) GPIOH_BASE)
#define GPIOI   ((GPIO_RegDef_t *) GPIOI_BASE)

/* GPIO Modes */
#define GPIO_MODE_INPUT     0x0U
#define GPIO_MODE_OUTPUT    0x1U
#define GPIO_MODE_ALT       0x2U
#define GPIO_MODE_ANALOG    0x3U

/* GPIO Output Types */
#define GPIO_OTYPE_PP       0x0U
#define GPIO_OTYPE_OD       0x1U

/* GPIO Speed */
#define GPIO_SPEED_LOW      0x0U
#define GPIO_SPEED_MEDIUM   0x1U
#define GPIO_SPEED_HIGH     0x2U
#define GPIO_SPEED_VERYHIGH 0x3U

/* GPIO Pull-up / Pull-down */
#define GPIO_NO_PUPD        0x0U
#define GPIO_PULLUP         0x1U
#define GPIO_PULLDOWN       0x2U

/* GPIO Helper Macros */
#define GPIO_PIN(n)         (1U << (n))
#define GPIO_MODER_MASK(n)  (0x3U << ((n)*2))
#define GPIO_OTYPER_MASK(n) (1U << (n))
#define GPIO_PUPDR_MASK(n)  (0x3U << ((n)*2))

#define GPIO_WRITE_PIN(GPIOx, PIN, VALUE)        \
    do {                                        \
        if ((GPIOx) != NULL) {                  \
            if (VALUE)                          \
                (GPIOx)->ODR |= (1U << (PIN)); \
            else                                \
                (GPIOx)->ODR &= ~(1U << (PIN));\
        }                                       \
    } while(0)

#define GPIO_READ_PIN(GPIOx, PIN)           (((GPIOx)->IDR >> (PIN)) & 0x1U)
#define GPIO_TOGGLE_PIN(GPIOx, PIN)         ((GPIOx)->ODR ^= GPIO_PIN(PIN))

#endif /* STM32L7_RCC_GPIO_H */
