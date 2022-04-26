#include "blink.h"
#include "bsp.h"

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

void QF_onStartup(void) {
}

void QF_onCleanup(void) {
    /* do nothing */
}

void Q_onAssert(char const * const module, int loc) {
    QS_ASSERTION(module, loc, 10000U); /* report assertion to QS */
    SystemReset();
}


#ifdef Q_SPY
    void QS_onReset(void) {
        SystemReset();
    }
    
    void QS_onCleanup(void) {
    
    }

    void QS_onCommand(
        uint8_t cmdId,
        uint32_t param1,
        uint32_t param2,
        uint32_t param3
    ) {
/*
        (void)param1;
        (void)param2;
        (void)param3;

        switch(cmdId) {
            default: break;
        }
*/
    QS_BEGIN_ID(0U, 0U) /* app-specific record */
        QS_U8(2, cmdId);
        QS_U32(8, param1);
        QS_U32(8, param2);
        QS_U32(8, param3);
    QS_END()
    }
    
#endif
