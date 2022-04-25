#include "bsp.h"

#define STANDARD_TIME (3000000)

int blink(void) {
    unsigned int time;
    while(1) {
        time = STANDARD_TIME;
        while(time--);
        board_led_on();
        time = STANDARD_TIME;
        while(time--);
        board_led_off();
    }
}

int main(void) {
    board_init();
    return blink();
}
