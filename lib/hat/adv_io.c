#include "hat.h"

volatile const ULTRA_SOUND_t ULTRA_SOUND = {
    .TRIG_PORT = GPIOA,
    .TRIG_PIN = 4,
    .ECHO_PORT = GPIOB,
    .ECHO_PIN = 0
};

void init_usart(const uint32_t baudrate){
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    init_gpio(GPIOA);
    set_pin_mode(GPIOA, 2, AF);
    set_pin_mode(GPIOA, 3, AF);
    GPIOA->AFR[0] |= (7 << (2 * 4)) | (7 << (3 * 4));
    USART2->BRR = SystemCoreClock / baudrate;
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    return;
}

void send_char(USART_TypeDef* USARTx, const char c){
    while(!(USARTx->SR & USART_SR_TXE));
    USARTx->DR = c;
    return;
}

void send_string(USART_TypeDef* USARTx, const char* str){
    int i = 0;
    while(str[i] != '\0'){
        send_char(USARTx, str[i]);
        i++;
    }
    return;
}

void init_ultrasound(void){
    init_gpio(ULTRA_SOUND.TRIG_PORT);
    set_pin_mode(ULTRA_SOUND.TRIG_PORT, ULTRA_SOUND.TRIG_PIN, OUTPUT);
    write_pin(ULTRA_SOUND.TRIG_PORT, ULTRA_SOUND.TRIG_PIN, LOW);

    init_gpio(ULTRA_SOUND.ECHO_PORT);
    set_pin_mode(ULTRA_SOUND.ECHO_PORT, ULTRA_SOUND.ECHO_PIN, INPUT);
    set_pin_pull(ULTRA_SOUND.ECHO_PORT, ULTRA_SOUND.ECHO_PIN, PULL_DOWN);
}

void init_servo(SERVO_t* servo){
    init_gpio(servo->SERVO_PIN_PORT);
    set_pin_mode(servo->SERVO_PIN_PORT, servo->SERVO_PWM_PIN, AF);
    if (servo->SERVO_PWM_PIN >= 8) {
        servo->SERVO_PIN_PORT->AFR[1] |= (3 << ((servo->SERVO_PWM_PIN - 8) * 4));
    } else {
        servo->SERVO_PIN_PORT->AFR[0] |= (3 << (servo->SERVO_PWM_PIN * 4));
    }
}

void init_adc(ADC_TypeDef* ADCx, uint8_t channel){
    // Enable ADC clock
    switch((uint32_t)ADCx){
        case (uint32_t)ADC1:
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
            break;
        case (uint32_t)ADC2:
            RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
            break;
        case (uint32_t)ADC3:
            RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
            break;
        default:
            return;
    };
    ADCx->SQR1 &= ~ADC_SQR1_L; // Set number of active channels to 1
    ADCx->SQR3 = (channel << ADC_SQR3_SQ1_Pos); // Set channel to read
    ADCx->SMPR1 |= ADC_SMPR1_SMP10_2 | ADC_SMPR1_SMP10_1 | ADC_SMPR1_SMP10_0; // 480 cycles sample time for channel 10
    ADCx->CR1 &= ~ADC_CR1_RES; // 12-bit resolution
    ADCx->CR2 &= ~ADC_CR2_ALIGN; // Right data alignment
    ADCx->CR2 |= ADC_CR2_ADON; // Enable ADC
}

void adc_swtstart(ADC_TypeDef* ADCx){
    ADCx->CR2 |= ADC_CR2_SWSTART;
}

void init_adc_interrupt(ADC_TypeDef* ADCx, uint8_t priority){
    ADCx->CR1 |= ADC_CR1_EOCIE; // Enable End of Conversion interrupt
    if (ADCx == ADC1) {
        NVIC_EnableIRQ(ADC_IRQn);
        NVIC_SetPriority(ADC_IRQn, priority);
    }
}

uint16_t read_adc(ADC_TypeDef* ADCx){
    return (uint16_t)(ADCx->DR);
}