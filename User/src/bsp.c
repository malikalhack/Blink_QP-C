#include "bsp.h"
#include "qpc.h"
#include "stm32f10x.h"
#include "standard.h"

Q_DEFINE_THIS_FILE

#define SYSCLK_FREQ_24MHz   (24000000)
#define TICKS_PER_SEC       (100U)

void SysTick_Handler(void);
void USART3_IRQHandler(void);

/**
 * @brief Static function for configuring Clock Control Registers group (RCC)
 */
static void RCCconfig(void) {
    RCC->CR |= RCC_CR_HSEON; //HSE=ON
    while(!(RCC->CR & RCC_CR_HSERDY));

    RCC->CFGR = 0x00050402; //PLLMUL=3; PLLSRC=HSE;
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= 0x00000002; //SW=PLL;
    RCC->CR &= ~RCC_CR_HSION; //HSI=OFF;
    RCC->APB2ENR = RCC_APB2ENR_IOPCEN + RCC_APB2ENR_IOPBEN;

#ifdef Q_SPY
    RCC->APB1ENR = RCC_APB1ENR_USART3EN + RCC_APB1ENR_PWREN;
#else
    RCC->APB1ENR = RCC_APB1ENR_PWREN;
#endif
}

/**
 * @brief Static function for configuring General Purpose Input Output
 * register group (GPIO)
 */
static void GPIOconfig(void) {
#ifdef Q_SPY
    //GPIOB->BSRR = GPIO_BSRR_BS11;   //PIN11(RX): Input, pull-up
    //GPIOB->CRH = 0x44448944;        //PIN10(TX): Alternate, 10MHz, push-pull
    GPIOB->CRH = 0x44444B44;
#endif
    GPIOC->BSRR = GPIO_BSRR_BS13; //PIN13 = 1 (LED off)
    GPIOC->CRH |= GPIO_CRH_MODE13_1;//PIN13: output, 2MHz, open-drain, general
}

/**
 * @brief Static function for configuring 24-bits SysTick timer.
 */
static void SYSTICKconfig (uint32_t ticks) {
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

void QXK_onIdle(void) {
#ifdef Q_SPY
    QS_rxParse(); /* parse all the received bytes */
    if ((USART3->SR & BIT(7)) != 0) { // TXE empty?
        uint16_t b;

        QF_INT_DISABLE();
        b = QS_getByte();
        QF_INT_ENABLE();

        if (b != QS_EOD) {  // not End-Of-Data?
            USART2->DR  = (b & 0xFFU);  // put into the DR register
        }
    }
#endif
}

#ifdef Q_SPY
    static QSTimeCtr QS_tickTime_;
    static QSTimeCtr QS_tickPeriod_;

    uint8_t QS_onStartup(void const *arg) {
        UNUSED_PARAM(arg); /* avoid the "unused parameter" compiler warning */
        static uint8_t qsBuf[2*1024]; /* buffer for QS-TX channel */
        static uint8_t qsRxBuf[100];  /* buffer for QS-RX channel */
        QS_initBuf(qsBuf, sizeof(qsBuf));
        QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

        return 0;
    }

    void USART3_IRQHandler(void){
        if(USART3->SR & USART_SR_RXNE) {
            uint32_t b = USART3->DR;
            QS_RX_PUT(b);
        }
    }

    static void UARTconfig (void) {
        USART3->BRR = 0x00000068;
        USART3->CR1 = BIT(3) + BIT(2); // enable RX, TX, RX interrupt
        USART3->CR1 |= BIT(13); // enable USART3
        USART3->CR1 |= BIT(5);

        NVIC_EnableIRQ(USART3_IRQn); /* USART3 interrupt used for QS-RX */

        QS_tickPeriod_ = SYSCLK_FREQ_24MHz / TICKS_PER_SEC;
        QS_tickTime_ = QS_tickPeriod_; /* to start the timestamp at zero */
    }

    void QS_onFlush(void) {
    /*uint16_t b;

    QF_INT_DISABLE();
    while ((b = QS_getByte()) != QS_EOD) { // while not End-Of-Data...
        QF_INT_ENABLE();
        while ((USART2->SR & USART_FLAG_TXE) == 0) { // while TXE not empty 
        }
        USART2->DR = (b & 0xFFU); // put into the DR register 
        QF_INT_DISABLE();
    }
    QF_INT_ENABLE();*/
    }

    QSTimeCtr QS_onGetTime(void) {
    /*
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) { 
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else { // the rollover occured, but the SysTick_ISR did not run yet
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
    */
        return 0;
    }
#endif

/**
 * @brief The function is exported to the startup file.
 */
void SystemInit (void) {
    RCCconfig();
    GPIOconfig();
    SYSTICKconfig(SYSCLK_FREQ_24MHz/TICKS_PER_SEC);
#ifdef Q_SPY
    UARTconfig();
    if (QS_INIT((void *)0) != 0U) { /* initialize the QS software tracing */
        Q_ERROR();
    }
#endif
    __enable_irq();  /* enable interrupts */
}

void SysTick_Handler(void) {
    QXK_ISR_ENTRY();   /* inform QXK about entering an ISR */
#ifdef Q_SPY
    QS_tickTime_ += QS_tickPeriod_; /* account for the clock rollover */
#endif
    QF_TICK_X(0U, (void *)0);  /* perform clock processing QF */
    QXK_ISR_EXIT();  /* inform QXK about exiting an ISR */
}

/** @fn board_led_off */
void board_led_off(void) {
    GPIOC->BSRR = GPIO_ODR_ODR13;
}

/** @fn board_led_on */
void board_led_on(void) {
    GPIOC->BRR = GPIO_ODR_ODR13;
}

void SystemReset(void) {
    __NVIC_SystemReset();
}
