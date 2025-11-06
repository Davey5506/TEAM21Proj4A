#include "hat.h"
#define PWN_OUT_PIN 6 // for PC6
#define FEEDBACK_IN_PIN 7 //for PC7
#define TIM3_FREQ_HZ 1000000 //1Mhz timer clock
#define PWM_FREQ_HZ 50 //PWM frequency
#define SERVO_NEUTRAL_PULSE_WIDTH 1500 //1.5ms pulse width for neutral position

void init_sys_tick(uint32_t ticks){
    // Runs at 16 MHz
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    return;
}

void delay_us(uint32_t us){
    uint32_t start = SysTick->VAL;
    uint32_t ticks = us * (SYSTEM_FREQ / 1000000);
    while((start - SysTick->VAL) < ticks){};
    return;
}

void init_adv_timer(TIM_TypeDef* TIMx, uint32_t freq, uint32_t arr, uint8_t cnt_enable, uint8_t pwm_enable){
    if(TIMx->CR1 & TIM_CR1_CEN){
        return;
    }
    switch((int)TIMx){
        case (int)TIM1:
            RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
            break;
        case (int)TIM8:
            RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
            break;
        default:
            return;
    }

    TIMx->PSC = (SYSTEM_FREQ / freq) - 1;
    TIMx->ARR = arr;
    TIMx->CCMR1 |= pwm_enable ? TIM_CCMR1_OC1M_1: 0;
    TIMx->CNT = 0;
    TIMx->CR1 |= cnt_enable ? TIM_CR1_CEN : 0;
    return;
}

void init_gp_timer(TIM_TypeDef* TIMx, uint32_t freq, uint32_t arr, uint8_t enable){
    if(TIMx->CR1 & TIM_CR1_CEN){
        return;
    }
    switch((int)TIMx){
        case (int)TIM2:
            RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
            break;
        case (int)TIM3:
            RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
            break;
        case (int)TIM4:
            RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
            break;
        case (int)TIM5:
            RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
            break;
        case (int)TIM9:
            RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
            break;
        case (int)TIM10:
            RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
            break;
        case (int)TIM11:
            RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
            break;
        case (int)TIM12:
            RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
            break;
        case (int)TIM13:
            RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
            break;
        case (int)TIM14:
            RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
            break;
        default:
            return;
    }

    TIMx->PSC = (SYSTEM_FREQ / freq) - 1;
    TIMx->ARR = arr;
    TIMx->CNT = 0;
    TIMx->CR1 |= (TIM_CR1_CEN & enable);
    return;
}

void init_timer_IRQ(TIM_TypeDef* TIMx, uint16_t priority){
    TIMx->DIER |= TIM_DIER_UIE;
    switch((int)TIMx){
        case (int)TIM2:
            NVIC_EnableIRQ(TIM2_IRQn);
            NVIC_SetPriority(TIM2_IRQn, priority);
            break;
        case (int)TIM3:
            NVIC_EnableIRQ(TIM3_IRQn);
            NVIC_SetPriority(TIM3_IRQn, priority);
            break;
        case (int)TIM4:
            NVIC_EnableIRQ(TIM4_IRQn);
            NVIC_SetPriority(TIM4_IRQn, priority);
            break;
        case (int)TIM5:
            NVIC_EnableIRQ(TIM5_IRQn);
            NVIC_SetPriority(TIM5_IRQn, priority);
            break;
        case (int)TIM9:
            NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
            NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, priority);
            break;
        case (int)TIM10:
            NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
            NVIC_SetPriority(TIM1_UP_TIM10_IRQn, priority);
            break;
        case (int)TIM11:
            NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
            NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, priority);
            break;
        case (int)TIM12:
            NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
            NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, priority);
            break;
        case (int)TIM13:
            NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
            NVIC_SetPriority(TIM8_UP_TIM13_IRQn, priority);
            break;
        case (int)TIM14:
            NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
            NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, priority);
            break;
        default:
            break;
    }
}