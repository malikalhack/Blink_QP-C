/**
 * @brief The Board Support Package (BSP)
 * @autor Anton Chernov
 * @date  23/03/2025
 */

#ifndef BSP_H_
#define BSP_H_

/**
 * @brief Creates actor objects and starts threads.
 */
void bsp_start(void);

/**
 * @brief Resets the MCU/board.
 */
void bsp_system_reset(void);

/**
 * @brief Turns off the LED on the board. It allows you to turn off the only LED
 * on the board, connected to Port C, pin 13.
 */
inline void bsp_led_off(void);

/**
 * @brief Turns on the LED on the board. It allows you to turn on the only LED
 * on the board, connected to Port C, pin 13.
 */
inline void bsp_led_on(void);

/**
 * @brief Toggles the LED on the board. Allows you to toggle the only LED on
 * the board, which is connected to Port C, pin 13.
 */
inline void bsp_led_toggle(void);

#endif /* !BSP_H_ */
