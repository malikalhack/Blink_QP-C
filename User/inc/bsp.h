// The Board Support Package (BSP).

#ifndef BSP_H_
#define BSP_H_

#include "stm32f10x.h"

#define BIT(offset) (1 << (offset))

/**
 * @details Board initialisation function. It allows you to configure
 * the control registers of counters, ports, timers, etc.
 */
void board_init(void);

/**
 * @details The function of turning on the LED on the board. It allows you
 * to light the only controllable LED on the board, located on port C, pin 13.
 */
void board_led_on(void);

/**
 * @details The function of turning off the LED on the board. It allows you
 * to turn off the only controllable LED on the board, located on port C, pin 13.
 */
void board_led_off(void);

#endif /* !BSP_H_ */
