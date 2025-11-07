#include "hat.h"
#include <stdio.h>
#include <string.h>

#define ADC_CHANNEL 10 // PC0
#define PWM_FREQ_HZ 50 // Standard servo PWM frequency
#define TIM3_FREQ_HZ 1000000 // 1MHz timer clock for 1us resolution
#define TIM3_ARR ((TIM3_FREQ_HZ / PWM_FREQ_HZ) - 1) // Auto-reload value for 50Hz (20000 - 1 = 19999)
#define FEEDBACK_PERIOD_US 1100
#define FEEDBACK_MAX_DUTY_CYCLE 0.971f
#define FEEDBACK_MIN_DUTY_CYCLE 0.029f
#define SERVO_NEUTRAL_PULSE_WIDTH 1500 // 1.5ms pulse width for neutral position

#define CW_MAX_PULSE 1480
#define CW_MIN_PULSE 1280
#define CCW_MIN_PULSE 1520
#define CCW_MAX_PULSE 1720

SERVO_t wheel = {
    .SERVO_PIN_PORT = GPIOC,
    .SERVO_PWM_PIN = 6,
    .SERVO_FEEDBACK_PIN = 7
};

volatile uint32_t pulse_width = 0; // Pulse width in microseconds
volatile uint8_t direction = 0; // 0 for CW, 1 for CCW
volatile uint8_t stop = 0;
volatile uint32_t feedback_timings[2]; // To store rising and falling edge times
volatile uint32_t pulse_start_times[2]; // To store pulse start times
volatile float angular_positions[2]; //Angular position in degrees
volatile float rpm = 0; //Rotational Speed in RPM
volatile uint16_t adc_value = 0; //ADC Value
volatile uint8_t value_ready = 0; //Flag to indicate new ADC value is ready

void print_data(void){
    if(value_ready){
        char string[80];
        char dchar[5];
        if(stop){
            strcpy(dchar, "STOP");
        }else if(direction){
            strcpy(dchar, "CCW");
        }else{
            strcpy(dchar, "CW");
        }
        sprintf(string, "ADC value: %u, dir: %s, servo (us):%lu, rpm: %.3f\r\n",
                adc_value, dchar, pulse_width, rpm);

        send_string(USART2, string);
        display_num(rpm, 0);
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
    pulse_width = stop ? SERVO_NEUTRAL_PULSE_WIDTH : lvl_to_pulse(adc_value, direction);
    servo_speed_set(pulse_width);
    print_data();
    value_ready = 1;
}

void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR13){
        EXTI->PR |= EXTI_PR_PR13; //Clear pending register
        if(!stop){
            stop = !stop;
            pulse_width = SERVO_NEUTRAL_PULSE_WIDTH; //Stop the wheel
        }else{
            stop = !stop;
            direction = !direction;
        }
    }
}

void EXTI9_5_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR7){
        EXTI->PR |= EXTI_PR_PR7; //Clear pending register
        if(read_pin(GPIOC, 7)){ //Rising edge
            feedback_timings[0] = TIM3->CNT;

        } else { //Falling edge
            feedback_timings[1] = TIM3->CNT;
            uint32_t width = feedback_timings[1] >= feedback_timings[0] ? (feedback_timings[1] - feedback_timings[0]) : (TIM3->ARR + 1 - feedback_timings[0] + feedback_timings[1]);
            float duty_cycle = (float)width / FEEDBACK_PERIOD_US;
            angular_positions[1] = angular_positions[0];
            angular_positions[0] = (duty_cycle - FEEDBACK_MIN_DUTY_CYCLE) * (360.0f / (FEEDBACK_MAX_DUTY_CYCLE - FEEDBACK_MIN_DUTY_CYCLE + 1));
            rpm = (angular_positions[0] - angular_positions[1]) * (60.0f / 360.0f) * (1000000.0f / FEEDBACK_PERIOD_US); //RPM calculation
        }
    }
}

int main(void) {
    // Initialize USART2 for serial communication at 115200 baud
    init_usart(115200);
    
    init_sys_tick(SYSTEM_FREQ); // 1s tick
    
    // Initialize SSD
    init_ssd(10);
    
    PWM_PC6_INIT(); // Initialize PWM on PC6
    
    set_pin_mode(GPIOC, 0, ANALOG); //PC0 as analog input for ADC
    
    // Configure EXTI for button on PC13
    set_pin_mode(GPIOC, 13, INPUT); //PC13 as input for button
    set_pin_pull(GPIOC, 13, PULL_UP); //Enable pull-up resistor
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk; //Enable SYSCFG clock
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;//Map EXTI13 to PC13
    NVIC_EnableIRQ(EXTI15_10_IRQn); //Enable EXTI15_10 interrupt
    NVIC_SetPriority(EXTI15_10_IRQn, 10); //Set priority to 1
    EXTI->IMR |= EXTI_IMR_MR13; //Unmask EXTI13
    EXTI->RTSR |= EXTI_RTSR_TR13; //Rising trigger

    // Configure EXTI for PC7
    set_pin_mode(GPIOC, 7, INPUT); //PC7 as input for feedback signal
    set_pin_pull(GPIOC, 7, PULL_DOWN); //Enable pull-down resistor
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PC; //Map EXTI7 to PC7
    EXTI->IMR |= EXTI_IMR_MR7; //Unmask EXTI7
    EXTI->RTSR |= EXTI_RTSR_TR7; //Rising trigger
    EXTI->FTSR |= EXTI_FTSR_TR7; //Falling trigger
    NVIC_EnableIRQ(EXTI9_5_IRQn); //Enable EXTI9_5 interrupt
    NVIC_SetPriority(EXTI9_5_IRQn, 0); //Set priority to 1

    // Initialize ADC1
    init_adc(ADC1, 10); // Initialize ADC1 on channel 10 (PC0)
    init_adc_interrupt(ADC1, 2); // Enable ADC interrupt with priority 2

    while(1){};
    return 0;
}