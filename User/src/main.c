#include "blink.h"
#include "bsp.h"
#include <stdio.h>  /* for printf()/fprintf() */
#include <stdlib.h> /* for exit() */

Q_DEFINE_THIS_FILE

int main(void) {
    static QEvt const *blink_queueSto[10];
    extern QActive * const AO_Blink;
    QF_init();
    Blink_ctor();

    QACTIVE_START(
        AO_Blink,
        1U,                     /* priority */
        blink_queueSto,         /* queue storage */
        Q_DIM(blink_queueSto),  /* queue storage size */
        (void *)0, 0U,          /* no stack */
        (QEvt *)0               /* no initialization event */
    );
    return QF_run();
}

void QF_onStartup(void) {}

void QXK_onIdle(void) {}

void QF_onCleanup(void) {
    // do nothing
}

void Q_onAssert(char const * const module, int loc) {
    fprintf(stderr, "Assertion failed in %s:%d", module, loc);
    exit(-1);
}

void SysTick_Handler(void) {
    QXK_ISR_ENTRY();   /* inform QXK about entering an ISR */
    QF_TICK_X(0U, (void *)0);  /* perform clock processing QF */
    QXK_ISR_EXIT();  /* inform QXK about exiting an ISR */
}
