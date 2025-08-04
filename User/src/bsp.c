/**
 * @brief The Board Support Package (BSP)
 * @autor Anton Chernov
 * @date  23/03/2025
 */

/******************************** Included files ******************************/
#include "bsp.h"
#include "qpc.h"
#include "blink.h"
#include "stm32f10x.h"

/********************************* Definitions ********************************/

Q_DEFINE_THIS_FILE


#define SYSCLK_FREQ_24MHz   (24000000)
#define TICKS_PER_SEC       (100U)

#define BIT(offset)         (1 << (offset))
#define UNUSED(x)           (void)(x)
#define DISCARD_RETURN(x)   UNUSED(x)
#define VOID_PTR_CAST(x)    (void*)(x)

enum EActorPrio{
    eFORBIDDEN_PRIO = 0,
    eBLINK_PRIO,
    eACTOR_PRIO_MAX
};

#ifdef Q_SPY
enum qspy_commands {
    eDUMMY = 0,
    ePRINT_PARAM_1,
    ePRINT_PARAM_2,
    ePRINT_PARAM_3,
    ePRINT_PARAM_ALL,
    eTHROW_ERROR
};

enum EQspyTraceRecords{
    eBLINK = QS_USER
};
#endif
/****************************** Private  variables ****************************/
#ifdef Q_SPY
static QSTimeCtr QS_tickTime_;
static QSTimeCtr QS_tickPeriod_;
#endif
/****************************** Private prototypes ****************************/

/**
 * @brief Resets Clock Control Registers group (RCC).
 */
static void rcc_reset(void);

/**
 * @brief Configurs flash.
 */
static void flash_config(void);

/**
 * @brief Configurs Clock Control Registers group (RCC).
 */
static void rcc_config(void);

/**
 * @brief Configurs General Purpose Input Output register group (GPIO).
 */
static void gpio_config(void);

/**
 * @brief Configurs 24-bits SysTick timer.
 * @param[in] ticks - ticks per second.
 */
static void systick_config (uint32_t ticks);

#ifdef Q_SPY
/**
 * @brief Configurs USART3.
 */
static void uart_config (void);

/**
 * @brief Interrupt request handler for USART3.
 */
void USART3_IRQHandler(void);
#endif

/********************* Application Programming Interface **********************/

/**
 * @brief The function is exported to the startup file.
 */
void SystemInit (void) {
    rcc_reset();
    flash_config();
    rcc_config();
    gpio_config();
    systick_config(SYSCLK_FREQ_24MHz/TICKS_PER_SEC);
#ifdef Q_SPY
    uart_config();
    if (QS_INIT((void *)0) != 0U) { /* initialize the QS software tracing */
        Q_ERROR();
    }
#endif
    __enable_irq();  /* enable interrupts */
}
/******************************************************************************/

__INLINE void bsp_led_off(void) {
    GPIOC->BSRR = GPIO_ODR_ODR13;
}
/******************************************************************************/

__INLINE void bsp_led_on(void) {
    GPIOC->BRR = GPIO_ODR_ODR13;
}
/******************************************************************************/

__INLINE void bsp_led_toggle(void) {
    GPIOC->ODR ^= GPIO_ODR_ODR13;
}
/******************************************************************************/

void __NO_RETURN bsp_system_reset(void) {
    __NVIC_SystemReset();
}
/******************************************************************************/

void bsp_start(void) {
    //static QF_MPOOL_EL(Clock) blink_queueSto[10];
    static QEvt const *blink_queueSto[10];
    extern QActive * const AO_Blink;

    Blink_ctor();
    QACTIVE_START(
        AO_Blink,
        eBLINK_PRIO,            /* priority */
        blink_queueSto,         /* queue storage */
        Q_DIM(blink_queueSto),  /* queue storage size */
        (void *)0, 0U,          /* no stack */
        (QEvt *)0               /* no initialization event */
    );
}
/****************************** Private functions *****************************/

static void rcc_reset(void) {
    RCC->CR |= (uint32_t)0x00000001;    /* HSION = 1; */
/*---------------------------- Reset defined bits ----------------------------*/
    RCC->CFGR &= (uint32_t)0xF8FF0000;
    RCC->CR   &= (uint32_t)0xFEF6FFFF;
    RCC->CR   &= (uint32_t)0xFFFBFFFF;
    RCC->CFGR &= (uint32_t)0xFF80FFFF;
/*----------------------------------------------------------------------------*/
    RCC->CIR = 0x009F0000;   /* Disable all interrupts and clear pending bits */
}
/******************************************************************************/

static void flash_config(void) {
    FLASH->ACR |= FLASH_ACR_PRFTBE;     // Enable Prefetch Buffer

    /* Flash 1 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_1;
}
/******************************************************************************/

static void rcc_config(void) {
/*---------------------------------- 24 MHz ----------------------------------*/
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);        /* HSE=ON */
    while(!(RCC->CR & RCC_CR_HSERDY));

    RCC->CFGR = 0x00050402; //PLLMUL=3; PLLSRC=HSE;

    RCC->CR   |= RCC_CR_PLLON;                  /* PLL=ON */
    while(!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;     /* SW=PLL */
    RCC->CR &= ~RCC_CR_HSION;                   /* HSI=OFF; */
    RCC->APB2ENR = RCC_APB2ENR_IOPCEN + RCC_APB2ENR_IOPBEN;

#ifdef Q_SPY
    RCC->APB1ENR = RCC_APB1ENR_USART3EN;
#endif
}
/******************************************************************************/

static void gpio_config(void) {
#ifdef Q_SPY
    GPIOB->CRH = 0x44444B44;        //PIN10(TX): Alternate, 10MHz, push-pull
                                    //PIN11(RX): Input
#endif
/********************************** PC13 LED **********************************/
   GPIOC->BSRR = GPIO_BSRR_BS13;      /* PIN13 = 1 (LED off) */
   GPIOC->CRH |= GPIO_CRH_MODE13_1;   /* PIN13: output, 2MHz, open-drain, gen */
}
/******************************************************************************/
#ifdef Q_SPY
static void uart_config (void) {
    USART3->BRR = 0x00000068;
    USART3->CR1 = BIT(3) + BIT(2); // enable RX, TX, RX interrupt
    USART3->CR1 |= BIT(13); // enable USART3
    USART3->CR1 |= BIT(5);

    NVIC_EnableIRQ(USART3_IRQn); /* USART3 interrupt used for QS-RX */

    QS_tickPeriod_ = SYSCLK_FREQ_24MHz / TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_; /* to start the timestamp at zero */
}
#endif
/******************************************************************************/

static void systick_config (uint32_t ticks) {
    if ((ticks - 1UL) <= SysTick_LOAD_RELOAD_Msk) {
        SysTick->LOAD  = (uint32_t)(ticks - 1UL);
    }
    NVIC_SetPriority (SysTick_IRQn, 3); /* set Priority for Systick Interrupt */
    SysTick->VAL   = 0UL;               /* Load the SysTick Counter Value */
    /* Enable SysTick IRQ and SysTick Timer */
    SysTick->CTRL  = SysTick_CTRL_ENABLE +
                     SysTick_CTRL_TICKINT +
                     SysTick_CTRL_CLKSOURCE;
    /* assing all priority bits for preemption-prio. and none to sub-prio. */
    NVIC_SetPriorityGrouping(0U);
}
/******************************************************************************/

/**
 * @brief Interrupt request handler for SysTick timer.
 */
void SysTick_Handler(void) {
    QXK_ISR_ENTRY();   /* inform QXK about entering an ISR */
#ifdef Q_SPY
    QS_tickTime_ += QS_tickPeriod_; /* account for the clock rollover */
#endif
    QF_TICK_X(0U, (void *)0);  /* perform clock processing QF */
    QXK_ISR_EXIT();  /* inform QXK about exiting an ISR */
}
/******************************************************************************/

/**
 * @brief Interrupt request handler for Hard fault.
 */
void __NO_RETURN HardFault_Handler (void) {
    __disable_irq();    /* disable interrupts */
    GPIOC->BRR = GPIO_ODR_ODR13;
    while(1);
}
/******************************************************************************/
#ifdef QXK_PRJ
/**
 * @brief Actions performed by the QXK kernel when it is idle.
 */
void QXK_onIdle(void) {
#ifdef Q_SPY
    QS_rxParse(); /* parse all the received bytes */
    if ((USART3->SR & BIT(7)) != 0) { // TXE empty?
        uint16_t b;

        QF_INT_DISABLE();
        b = QS_getByte();
        QF_INT_ENABLE();

        if (b != QS_EOD) {  // not End-Of-Data?
            USART3->DR  = (b & 0xFFU);  // put into the DR register
        }
    }
#endif
}
#endif
/******************************************************************************/

Q_NORETURN Q_onError(char const * const module, int_t const id) {
    QS_ASSERTION(module, id, 10000U);

#ifndef NDEBUG
    while(1); /* for debugging, hang on in an endless loop... */
#else
    NVIC_SystemReset();
#endif
}
/******************************************************************************/

void QF_onCleanup(void) {
    /* do nothing */
}
/******************************************************************************/

void QF_onStartup(void) {
    QS_USR_DICTIONARY(eBLINK);
    QS_GLB_FILTER(QS_UA_RECORDS);
}
/******************************************************************************/

void Q_onAssert(char const * const module, int loc) {
    QS_ASSERTION(module, loc, 10000U); /* report assertion to QS */
    __NVIC_SystemReset();
}
/******************************************************************************/
#ifdef Q_SPY

uint8_t QS_onStartup(void const *arg) {
    UNUSED(arg); /* avoid the "unused parameter" compiler warning */
    static uint8_t qsTxBuf[2*1024]; /* buffer for QS-TX channel */
    static uint8_t qsRxBuf[100];  /* buffer for QS-RX channel */
    QS_initBuf(qsTxBuf, sizeof(qsTxBuf));
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));
    return 0;
}
/******************************************************************************/

void USART3_IRQHandler(void){
    if(USART3->SR & USART_SR_RXNE) {
        uint32_t b = USART3->DR;
        QS_RX_PUT(b);
    }
}
/******************************************************************************/

void QS_onFlush(void) {
    uint16_t b;

    QF_INT_DISABLE();
    while ((b = QS_getByte()) != QS_EOD) { // while not End-Of-Data...
        QF_INT_ENABLE();
        while ((USART3->SR & BIT(7)) == 0); // while TXE not empty
        USART3->DR = (b & 0xFFU); // put into the DR register
        QF_INT_DISABLE();
    }
    QF_INT_ENABLE();
}
/******************************************************************************/

QSTimeCtr QS_onGetTime(void) {
    QSTimeCtr result;
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {
        result = QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else { // the rollover occured, but the SysTick_ISR did not run yet
        result = QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
    return result;
}
/******************************************************************************/

void QS_onReset(void) {
    __NVIC_SystemReset();
}
/******************************************************************************/

void QS_onCleanup(void) {

}
/******************************************************************************/

void QS_onCommand(
    uint8_t cmdId,
    uint32_t param1,
    uint32_t param2,
    uint32_t param3
) {
    switch(cmdId) {
        case eDUMMY: break;
        case ePRINT_PARAM_1:
            QS_BEGIN_ID(eBLINK, eBLINK_PRIO)
                QS_STR("Print parameter #1:");
                QS_U32(8, param1);
            QS_END()
            break;
        case ePRINT_PARAM_2:
            QS_BEGIN_ID(eBLINK, eBLINK_PRIO)
                QS_STR("Print parameter #2:");
                QS_U32(8, param2);
            QS_END()
            break;
        case ePRINT_PARAM_3:
            QS_BEGIN_ID(eBLINK, eBLINK_PRIO)
                QS_STR("Print parameter #3:");
                QS_U32(8, param3);
            QS_END()
            break;
        case ePRINT_PARAM_ALL:
            QS_BEGIN_ID(eBLINK, eBLINK_PRIO)
                QS_STR("Print all parameters:\n");
                QS_U32(8, param1);
                QS_U32(8, param2);
                QS_U32(8, param3);
            QS_END()
            break;
        case eTHROW_ERROR:
            QS_BEGIN_ID(eBLINK, eBLINK_PRIO)
                QS_STR("Throw ERROR!\n");
            QS_END()
        default: Q_ERROR();
    }
}
#endif
/******************************************************************************/
