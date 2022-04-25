#include "bsp.h"

void board_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    //GPIOC->BRR = BIT(13);
    GPIOC->BSRR = GPIO_ODR_ODR13;
    GPIOC->CRH |= GPIO_CRH_MODE13_1;
}

void board_led_off(void) {
    GPIOC->BSRR = GPIO_ODR_ODR13;
}

void board_led_on(void) {
    GPIOC->BRR = GPIO_ODR_ODR13;
}
