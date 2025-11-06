#include "hat.h"

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

void init_ssd(uint16_t reload_time){
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
    TIM7->PSC = (SYSTEM_FREQ / 160000) - 1;
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