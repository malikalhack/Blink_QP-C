// The Board Support Package (BSP).

#ifndef BSP_H_
#define BSP_H_

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


__declspec(noreturn) void SystemReset(void);
#endif /* !BSP_H_ */
