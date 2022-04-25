// The Board Support Package (BSP).

#ifndef BSP_H_
#define BSP_H_

#include "stm32f10x.h"

#define BIT(offset) (1 << (offset))

void board_init(void);
void board_led_on(void);
void board_led_off(void);

#endif /* !BSP_H_ */
