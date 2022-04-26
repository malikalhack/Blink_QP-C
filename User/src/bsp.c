#include "bsp.h"
#include "stm32f10x.h"

#define BIT(offset)         (1 << (offset))
#define SYSCLK_FREQ_24MHz   (24000000)
#define TICKS_PER_SEC       (100U)

/**
 * @brief Static function for configuring Clock Control Registers group (RCC)
 */
static void RCCconfig(void) {
    RCC->CR |= RCC_CR_HSEON; /* Set HSE ON bit */
    while(!(RCC->CR & RCC_CR_HSERDY));
    RCC->CFGR = 0x00100402;
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
}

/**
 * @brief Static function for configuring General Purpose Input Output
 * register group (GPIO)
 */
static void GPIOconfig(void) {
    GPIOC->BSRR = GPIO_ODR_ODR13;
    GPIOC->CRH |= GPIO_CRH_MODE13_1;
}

/**
 * @brief Static function for configuring 24-bits SysTick timer.
 */
static void SYSTICKconfig (uint32_t ticks) {
    if ((ticks - 1UL) <= SysTick_LOAD_RELOAD_Msk) {
        SysTick->LOAD  = (uint32_t)(ticks - 1UL);
    }
    NVIC_SetPriority (SysTick_IRQn, 3); /* set Priority for Systick Interrupt */
    SysTick->VAL   = 0UL;               /* Load the SysTick Counter Value */
    /* Enable SysTick IRQ and SysTick Timer */
    SysTick->CTRL  = SysTick_CTRL_ENABLE + 
                     SysTick_CTRL_TICKINT + 
                     SysTick_CTRL_CLKSOURCE;
    /* assing all priority bits for preemption-prio. and none to sub-prio. */
    NVIC_SetPriorityGrouping(0U);
    __enable_irq();  /* enable interrupts */
}

/**
 * @brief The function is exported to the startup file.
 */
void SystemInit (void) {
    RCCconfig();
    GPIOconfig();
    SYSTICKconfig(SYSCLK_FREQ_24MHz/TICKS_PER_SEC);
}
  
/** @fn board_led_off */
void board_led_off(void) {
    GPIOC->BSRR = GPIO_ODR_ODR13;
}

/** @fn board_led_on */
void board_led_on(void) {
    GPIOC->BRR = GPIO_ODR_ODR13;
}
