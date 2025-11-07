#include "hat.h"
#include <stdio.h>

#define ADC_CHANNEL 10 // PC0
#define PWM_FREQ_HZ 50
#define TIM3_FREQ_HZ 1000 // 1kHz timer clock for 1us resolution
#define TIM3_ARR (TIM3_FREQ_HZ * PWM_FREQ_HZ) // Auto-reload value for 50Hz
#define SERVO_NEUTRAL_PULSE_WIDTH 1500 // 1.5ms pulse width for neutral position

#define CW_MAX_PULSE 1480
#define CW_MIN_PULSE 1280
#define CCW_MAX_PULSE 1520
#define CCW_MIN_PULSE 1720

SERVO_t wheel = {
    .SERVO_PIN_PORT = GPIOC,
    .SERVO_PWM_PIN = 6,
    .SERVO_FEEDBACK_PIN = 7
};

volatile uint32_t pulse_width = 0; // Pulse width in microseconds
volatile uint32_t period = 0; 
volatile uint8_t direction = 0; // 0 for CW, 1 for CCW

volatile float rpm = 0; //Rotational Speed in RPM
volatile uint16_t adc_value = 0; //ADC Value
volatile uint8_t value_ready = 0; //Flag to indicate new ADC value is ready

void print_data(void){
    if(value_ready){
        char string[35];
        sprintf(string, "Pulse Width: %lu\tDirection: %s\r\n", pulse_width, direction ? "CCW" : "CW");
        send_string(USART2, string);
        display_num(adc_value, 0);
        value_ready = 0;
    }
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
}

void TIM3_INIT(void){
    GPIOC->AFR[0] &= ~((0xF << (wheel.SERVO_PWM_PIN * 4)) | (0xF << (wheel.SERVO_FEEDBACK_PIN * 4)));
    GPIOC->AFR[0] |= (2 << (wheel.SERVO_PWM_PIN *4)) | (2<< (wheel.SERVO_FEEDBACK_PIN *4)); //AF2 for TIM3_CH1 and TIM3_CH2

    const uint32_t TIM3_ARR_VALUE= TIM3_FREQ_HZ / PWM_FREQ_HZ; //20000 for 50Hz
    const uint16_t NEUTRAL_PULSE_VALUE= SERVO_NEUTRAL_PULSE_WIDTH; //1500 for 1.5ms pulse width

    init_gp_timer(TIM3, TIM3_FREQ_HZ, TIM3_ARR_VALUE, 1); //used helper to PSC,ARR,Counter

    TIM3->CCMR1 &= ~TIM_CCMR1_CC1S_Msk; //CC1 channel is output
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //PWM mode 
    TIM3->CCER |= TIM_CCER_CC1E; //enabled capture/compare 1 output
    TIM3->CCR1 = NEUTRAL_PULSE_VALUE; //set the pulse width for neutral position

    TIM3->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE; //Enable interrupt for capture/compare 1 and 2
    init_timer_IRQ(TIM3, 2);//used helper to setup NVIC
}

uint32_t lvl_to_pulse(uint16_t lvl, uint8_t direction){
    if(direction == 0){ //Clockwise
        return umap(lvl, 0,4095, CW_MIN_PULSE, CW_MAX_PULSE);
    } else { //Counter-Clockwise
        return umap(lvl, 0, 4095, CCW_MIN_PULSE, CCW_MAX_PULSE);
    }
}

void servo_speed_set(uint32_t pulse_width){
    TIM3->CCR1= pulse_width;
}

void TIM3_IRQHandler(void){ //meaures pulse width and period of feedback signal
    if(TIM3->SR & TIM_SR_CC1IF){

        uint32_t ccr2= TIM3->CCR2;
        uint32_t ccr1= TIM3->CCR1;

        pulse_width=ccr2;
        period=ccr1;

        TIM3->SR &= ~TIM_SR_CC1IF;
    }
    if (TIM3->SR & TIM_SR_UIF){
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

void SysTick_Handler(void){
    // Start ADC conversion, but don't wait for it in the ISR
    adc_swtstart(ADC1);
}

void ADC_IRQHandler(void){
    adc_value = read_adc(ADC1); // Clear EOC flag by reading ADC value
    pulse_width = lvl_to_pulse(adc_value, direction);
    servo_speed_set(pulse_width);
    print_data();
    value_ready = 1;
}

void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR13){
        EXTI->PR |= EXTI_PR_PR13; //Clear pending register
        direction ^= 1; //Toggle direction on each button press
    }
}

void lvl_to_pulse(uint16_t lvl, uint8_t direction){
    if(direction == 0){ //Clockwise
        pulse_width = CW_MIN_PULSE + ((CW_MAX_PULSE - CW_MIN_PULSE) * lvl) / 100;
    } else { //Counter-Clockwise
        pulse_width = CCW_MIN_PULSE + ((CCW_MAX_PULSE - CCW_MIN_PULSE) * lvl) / 100;
    }
}

int main(void) {
    init_usart(115200);
    init_sys_tick(SYSTEM_FREQ); // 1s tick
    init_ssd(10);
    display_num(0, 1);
    PWM_PC6_INIT();
    set_pin_mode(GPIOC, 0, ANALOG); //PC0 as analog input for ADC
    init_adc(ADC1, 10); // Initialize ADC1 on channel 10 (PC0)
    init_adc_interrupt(ADC1, 2); // Enable ADC interrupt with priority 2

    while(1){};
    return 0;
}