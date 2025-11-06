#ifndef HAT_H
#define HAT_H

#include "stm32f4xx.h"
#define SYSTEM_FREQ 16000000U

typedef struct{
    GPIO_TypeDef* GPIO_PORTS[4];
    GPIO_TypeDef* PIN_PORTS[8];
    uint8_t PIN_NUMS[8];
}PMOD_t;

typedef struct{
    GPIO_TypeDef* GPIO_PORTS[3];
    GPIO_TypeDef* DATA_PIN_PORTS[8];
    uint8_t DATA_PINs[8];
    GPIO_TypeDef* SELECT_PIN_PORTS[4];
    uint8_t SELECT_PINs[4];
    uint8_t ACTIVE_DIGIT;
}SSD_t;

typedef struct{
    GPIO_TypeDef* TRIG_PORT;
    uint8_t TRIG_PIN;
    GPIO_TypeDef* ECHO_PORT;
    uint8_t ECHO_PIN;
}ULTRA_SOUND_t;

typedef struct{
    GPIO_TypeDef* SERVO_PIN_PORT;
    uint8_t SERVO_PWM_PIN;
    uint8_t SERVO_FEEDBACK_PIN;
}SERVO_t;

typedef enum{
    INPUT = 0U,
    OUTPUT = 1U,
    AF = 2U,
    ANALOG = 3U
}PIN_MODE;

typedef enum{
    NO_PULL = 0U,
    PULL_UP = 1U,
    PULL_DOWN = 2U
}PIN_PULL;

typedef enum{
    PUSH_PULL = 0U,
    OPEN_DRAIN = 1U
}PIN_OUTPUT_TYPE;

typedef enum{
    LOW = 0U,
    HIGH = 1U
}PIN_VALUE;

extern const PMOD_t PMOD_A;
extern const PMOD_t PMOD_B;
extern const PMOD_t PMOD_C;

extern volatile const ULTRA_SOUND_t ULTRA_SOUND;

// GPIO pin functions
void init_gpio(GPIO_TypeDef* GPIOx);
void init_pmod(PMOD_t pmod);
void set_pin_mode(GPIO_TypeDef* GPIOx, uint8_t pin, PIN_MODE mode);
void set_pin_pull(GPIO_TypeDef* GPIOx, uint8_t pin, PIN_PULL pull);
void set_output_type(GPIO_TypeDef* GPIOx, uint8_t pin, PIN_OUTPUT_TYPE type);
void write_pin(GPIO_TypeDef* GPIOx, uint8_t pin, PIN_VALUE value);
uint8_t read_pin(GPIO_TypeDef* GPIOx, uint8_t pin);
void toggle_pin(GPIO_TypeDef* GPIOx, uint8_t pin);

// SysTick and Timer functions
void init_sys_tick(uint32_t ticks);
void delay_us(uint32_t us);
void init_adv_timer(TIM_TypeDef* TIMx, uint32_t freq, uint32_t arr, uint8_t cnt_enable, uint8_t pwm_enable);
void init_gp_timer(TIM_TypeDef* TIMx, uint32_t freq, uint32_t arr, uint8_t enable);
void init_timer_IRQ(TIM_TypeDef* TIMx, uint16_t priority);

// Seven Segment Display functions
void init_ssd (uint16_t reload_time);
void display_num(uint16_t num, uint8_t decimal_place);

// Advanced IO functions
void init_usart(const uint32_t baudrate);
void send_char(USART_TypeDef* USARTx, const char c);
void send_string(USART_TypeDef* USARTx, const char* str);
void init_adc_interrupt(ADC_TypeDef* ADCx, uint8_t priority);
void init_ultrasound(void);
void int_to_string(int num, char* str, uint16_t len);
void init_servo(SERVO_t* servo);
void init_adc(ADC_TypeDef* ADCx, uint8_t channel);
void adc_swtstart(ADC_TypeDef* ADCx);
void init_adc_interrupt(ADC_TypeDef* ADCx, uint8_t priority);
uint16_t read_adc(ADC_TypeDef* ADCx);

// Utility functions
uint32_t umap(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
#endif //HAT_H