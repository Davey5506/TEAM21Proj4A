#include "hat.h"

const PMOD_t PMOD_A = {
    .GPIO_PORTS = {GPIOA, GPIOB, GPIOC, 0},
    .PIN_PORTS = {GPIOC, GPIOB, GPIOA, GPIOA, GPIOB, GPIOA, GPIOA, GPIOA},
    .PIN_NUMS = {7, 12, 11, 12, 6, 7, 6, 5},
};
const PMOD_t PMOD_B = {
    .GPIO_PORTS = {GPIOA, GPIOB, GPIOC, GPIOD},
    .PIN_PORTS = {GPIOA, GPIOA, GPIOC, GPIOC, GPIOA, GPIOB, GPIOD, GPIOC},
    .PIN_NUMS = {1, 15, 12, 10, 0, 7, 2, 11}
};
const PMOD_t PMOD_C = {
    .GPIO_PORTS = {GPIOC, 0, 0, 0},
    .PIN_PORTS = {GPIOC, GPIOC, GPIOC, GPIOC},
    .PIN_NUMS = {0, 1, 2, 3, 0xFF, 0xFF, 0xFF, 0xFF}
};

volatile const ULTRA_SOUND_t ULTRA_SOUND = {
    .TRIG_PORT = GPIOA,
    .TRIG_PIN = 4,
    .ECHO_PORT = GPIOB,
    .ECHO_PIN = 0
};

volatile const SSD_t SSD = {
    .GPIO_PORTS = {GPIOA, GPIOB, GPIOC},
    .DATA_PIN_PORTS = {GPIOB, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOB},
    .DATA_PINs = {10, 9, 13, 14, 4, 1, 10, 5},
    .SELECT_PIN_PORTS = {GPIOB, GPIOA, GPIOB, GPIOC},
    .SELECT_PINs = {2, 8, 15, 4},
};

volatile const uint8_t digits[10] = {
    0b01111110, // 0 (A,B,C,D,E,F)
    0b00001100, // 1 (B,C)
    0b10110110, // 2 (A,B,D,E,G)
    0b10011110, // 3 (A,B,C,D,G)
    0b11001100, // 4 (B,C,F,G)
    0b11011010, // 5 (A,C,D,F,G)
    0b11111010, // 6 (A,C,D,E,F,G)
    0b00001110, // 7 (A,B,C)
    0b11111110, // 8 (A,B,C,D,E,F,G)
    0b11011110  // 9 (A,B,C,D,F,G)
};
volatile uint8_t ssd_out[4] = {0, 0, 0, 0};
volatile uint8_t active_digit = 0;

void init_gpio(GPIO_TypeDef* GPIOx){
    switch((unsigned int)GPIOx){
        case (unsigned int)GPIOA:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
            break;
        case (unsigned int)GPIOB:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
            break;
        case (unsigned int)GPIOC:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
            break;
        case (unsigned int)GPIOD:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
            break;
        case (unsigned int)GPIOE:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
            break;
        case (unsigned int)GPIOF:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
            break;
        case (unsigned int)GPIOG:
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
            break;
        default:
            break;
    }
    return;
}

void init_pmod(PMOD_t pmod){
    for(int i = 0; i < 4; i++){
        if(pmod.GPIO_PORTS[i] != 0){
            init_gpio(pmod.GPIO_PORTS[i]);
        }else{
            break;
        }
    }
}

void init_usart(uint32_t baudrate){
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    init_gpio(GPIOA);
    set_pin_mode(GPIOA, 2, AF);
    set_pin_mode(GPIOA, 3, AF);
    GPIOA->AFR[0] |= (7 << (2 * 4)) | (7 << (3 * 4));
    USART2->BRR = SystemCoreClock / baudrate;
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    return;
}

void send_char(USART_TypeDef* USARTx, char c){
    while(!(USARTx->SR & USART_SR_TXE));
    USARTx->DR = c;
    return;
}

void set_pin_mode(GPIO_TypeDef* GPIOx, uint8_t pin, PIN_MODE mode){
    GPIOx->MODER &= ~(0x3 << (pin * 2));
    GPIOx->MODER |= (mode << (pin * 2));
    return;
}

void set_pin_pull(GPIO_TypeDef* GPIOx, uint8_t pin, PIN_PULL pull){
    GPIOx->PUPDR &= ~(0x3 << (pin * 2));
    GPIOx->PUPDR |= (pull << (pin * 2));
    return;
}

void set_output_type(GPIO_TypeDef* GPIOx, uint8_t pin, PIN_OUTPUT_TYPE type){
    GPIOx->OTYPER &= ~(0x1 << pin);
    GPIOx->OTYPER |= (type << pin);
    return;
}

void write_pin(GPIO_TypeDef* GPIOx, uint8_t pin, PIN_VALUE value){
    if(value){
        GPIOx->BSRR |= (1 << pin);
    }else{
        GPIOx->BSRR |= (1 << (pin + 16));
    }
    return;
}

uint8_t read_pin(GPIO_TypeDef* GPIOx, uint8_t pin){
    return (GPIOx->IDR >> pin) & 0x1;
}

void toggle_pin(GPIO_TypeDef* GPIOx, uint8_t pin){
    GPIOx->ODR ^= (1 << pin);
    return;
}

void init_sys_tick(uint32_t ticks){
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
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
            break;
    }

    TIMx->PSC = (SYSTEM_FREQ / freq) - 1;
    TIMx->ARR = arr;
    TIMx->CNT = 0;
    if(enable){
        TIMx->CR1 |= TIM_CR1_CEN;
    }
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

void init_ssd( uint16_t reload_time){
    for(int i = 0; i < 3; i++){
        init_gpio(SSD.GPIO_PORTS[i]);
    }
    for(int i = 0; i < 8; i++){
        set_pin_mode(SSD.DATA_PIN_PORTS[i], SSD.DATA_PINs[i], OUTPUT);
    }
    for(int i = 0; i < 4; i++){
        set_pin_mode(SSD.SELECT_PIN_PORTS[i], SSD.SELECT_PINs[i], OUTPUT);
    }

    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->DIER |= TIM_DIER_UIE;
    TIM7->PSC = SYSTEM_FREQ / (160000 - 1);
    TIM7->ARR = reload_time;
    NVIC_EnableIRQ(TIM7_IRQn);
    NVIC_SetPriority(TIM7_IRQn, 20); 
    TIM7->CR1 |= TIM_CR1_CEN;
}

void display_num(uint16_t num, uint8_t decimal_place){
    uint16_t temp_num = num;
    for(int i = 0; i < 4; i++){
        uint8_t digit = temp_num % 10;
        if (i > 0 && num < (i == 1 ? 10 : (i == 2 ? 100 : 1000))) {
            // Blank leading zeros, but always show the first digit (i=0)
            ssd_out[i] = 0;
        } else {
            ssd_out[i] = digits[digit];
        }
        temp_num /= 10;
    }
    if (decimal_place < 4) ssd_out[decimal_place] |= 1;
}

void init_ultrasound(void){
    init_gpio(ULTRA_SOUND.TRIG_PORT);
    set_pin_mode(ULTRA_SOUND.TRIG_PORT, ULTRA_SOUND.TRIG_PIN, OUTPUT);
    write_pin(ULTRA_SOUND.TRIG_PORT, ULTRA_SOUND.TRIG_PIN, LOW);

    init_gpio(ULTRA_SOUND.ECHO_PORT);
    set_pin_mode(ULTRA_SOUND.ECHO_PORT, ULTRA_SOUND.ECHO_PIN, INPUT);
    set_pin_pull(ULTRA_SOUND.ECHO_PORT, ULTRA_SOUND.ECHO_PIN, PULL_DOWN);
}

void int_to_string(int num, char* str, uint16_t len){
    str[len - 1] = '\0';
    len--;
    while(num / 10){
        str[len--] = (num % 10) + '0';
        num /= 10;
    }
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

// local functions
void select_active_digit(void){
    switch(active_digit){
        case 0:
            write_pin(SSD.SELECT_PIN_PORTS[0], SSD.SELECT_PINs[0], LOW);
            write_pin(SSD.SELECT_PIN_PORTS[1], SSD.SELECT_PINs[1], HIGH);
            write_pin(SSD.SELECT_PIN_PORTS[2], SSD.SELECT_PINs[2], HIGH);
            write_pin(SSD.SELECT_PIN_PORTS[3], SSD.SELECT_PINs[3], HIGH);
            break;
        case 1:
            write_pin(SSD.SELECT_PIN_PORTS[0], SSD.SELECT_PINs[0], HIGH);
            write_pin(SSD.SELECT_PIN_PORTS[1], SSD.SELECT_PINs[1], LOW);
            write_pin(SSD.SELECT_PIN_PORTS[2], SSD.SELECT_PINs[2], HIGH);
            write_pin(SSD.SELECT_PIN_PORTS[3], SSD.SELECT_PINs[3], HIGH);
            break;
        case 2:
            write_pin(SSD.SELECT_PIN_PORTS[0], SSD.SELECT_PINs[0], HIGH);
            write_pin(SSD.SELECT_PIN_PORTS[1], SSD.SELECT_PINs[1], HIGH);
            write_pin(SSD.SELECT_PIN_PORTS[2], SSD.SELECT_PINs[2], LOW);
            write_pin(SSD.SELECT_PIN_PORTS[3], SSD.SELECT_PINs[3], HIGH);
            break;
        case 3:
            write_pin(SSD.SELECT_PIN_PORTS[0], SSD.SELECT_PINs[0], HIGH);
            write_pin(SSD.SELECT_PIN_PORTS[1], SSD.SELECT_PINs[1], HIGH);
            write_pin(SSD.SELECT_PIN_PORTS[2], SSD.SELECT_PINs[2], HIGH);
            write_pin(SSD.SELECT_PIN_PORTS[3], SSD.SELECT_PINs[3], LOW);
            break;
        default:
            break;
    }
}
// ISRs
void TIM7_IRQHandler(void){
    TIM7->SR &= ~TIM_SR_UIF;

    // Clear previous digit
    for(int i = 0; i < 8; i++){
        write_pin(SSD.DATA_PIN_PORTS[i], SSD.DATA_PINs[i], 0);
    }

    // Select active digit
    select_active_digit();

    // Rotate active digit
    active_digit = (active_digit + 1) % 4;

    // Write ssd_out to GPIO
    for(int i = 0; i < 8; i++){
        write_pin(SSD.DATA_PIN_PORTS[i], SSD.DATA_PINs[i], ssd_out[active_digit] >> (i+1) & 1);
    }
    write_pin(SSD.DATA_PIN_PORTS[7], SSD.DATA_PINs[7], ssd_out[active_digit] & 1);
    return;
}