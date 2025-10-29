#include "hat.h"
#include <stdio.h>

#define PWM_FREQ_HZ 50
#define TIM3_FREQ_HZ 1000 // 1kHz timer clock for 1us resolution
#define TIM3_ARR (TIM3_FREQ_HZ * PWM_FREQ_HZ) // Auto-reload value for 50Hz
#define SERVO_NEUTRAL_PULSE_WIDTH 1500 // 1.5ms pulse width for neutral position

volatile float rpm = 0; //Rotational Speed in RPM

void print_data(void){
    float rpm_local = rpm;
    display_num((uint16_t)(rpm_local * 10), 1); //Display RPM with one decimal place
}

void PWM_PC6_INIT(void){
    init_gpio(GPIOC);
    set_pin_mode(GPIOC, 6, AF);
    GPIOC->AFR[0] |= (2 << (6*4)); //AF2 for TIM3_CH1

    // Initialize TIM3 for 50Hz PWM. Timer will be enabled by this function.
    init_gp_timer(TIM3, TIM3_FREQ_HZ, TIM3_ARR, 1);

    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //PWM mode 1
    TIM3->CCER |= TIM_CCER_CC1E; //Enable capture/compare 1 output
    TIM3->CCR1 = SERVO_NEUTRAL_PULSE_WIDTH; //Set pulse width for neutral position (1.5ms)
    TIM3->CR1 |= TIM_CR1_CEN; //Enable TIM3
}

int main(void) {
    init_ssd(10);
    display_num(0, 1);
    PWM_PC6_INIT();

    while(1){};
    return 0;
}