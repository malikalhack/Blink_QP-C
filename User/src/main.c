/**
 * @brief Entry point
 * @autor Anton Chernov
 * @date  25/04/2022
 */

/******************************** Included files ******************************/
#include "blink.h"
#include "bsp.h"
/********************************* Entry point ********************************/
int main(void) {
    QF_init();       // initialize the framework and the underlying RT kernel
    bsp_start();     // create the AOs and start Threads
    return QF_run(); // run the QF application
}
/******************************************************************************/
