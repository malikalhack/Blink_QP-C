#include "blink.h"
#include "bsp.h"

Q_DEFINE_THIS_FILE

#ifdef Q_SPY

typedef enum {
    dummy = 0,
    print_param_1,
    print_param_2,
    print_param_3,
    print_all_param,
    throw_error
} qspy_commands_t;

typedef enum {
    BLINK = QS_USER
} qspy_trace_records_t;

#endif

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
    QS_USR_DICTIONARY(BLINK);
    QS_GLB_FILTER(QS_UA_RECORDS);
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
        switch(cmdId) {    
            case dummy: break;
            case print_param_1:
                QS_BEGIN_ID(BLINK, 0U)
                    QS_STR("Print parameter #1:");
                    QS_U32(8, param1);
                QS_END()
                break;
            case print_param_2:
                QS_BEGIN_ID(BLINK, 0U)
                    QS_STR("Print parameter #2:");
                    QS_U32(8, param2);
                QS_END()
                break;
            case print_param_3:
                QS_BEGIN_ID(BLINK, 0U)
                    QS_STR("Print parameter #3:");
                    QS_U32(8, param3);
                QS_END()
                break;
            case print_all_param:
                QS_BEGIN_ID(BLINK, 0U)
                    QS_STR("Print all parameters:\n");
                    QS_U32(8, param1);
                    QS_U32(8, param2);
                    QS_U32(8, param3);
                QS_END()
                break;
            case throw_error:
                QS_BEGIN_ID(BLINK, 0U)
                    QS_STR("Throw ERROR!\n");
                QS_END()
            default: Q_ERROR();
        }
    }
    
#endif
