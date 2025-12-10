/*
    Final Project: Occupancy Detection System
    - Sensors: DHT22(PB0), LDR(PC0), CO2(Fixed/USART3)
    - Actuators: Servo(PB1), Fan(PA4)
    - Model: Decision Tree (Integrated)
*/

#include "stm32f10x.h"
#include "stdio.h"
#include "math.h"

/* --- Pin Definitions --- */
#define DHT22_PORT      GPIOB
#define DHT22_PIN       GPIO_Pin_0
#define LIGHT_PORT      GPIOC
#define LIGHT_PIN       GPIO_Pin_0
#define SERVO_PORT      GPIOB
#define SERVO_PIN       GPIO_Pin_1    // PB1 (TIM3_CH4)
#define FAN_PORT        GPIOA
#define FAN_PIN         GPIO_Pin_4    // PA4 (Fan Control)

/* --- Global & SysTick --- */
volatile uint32_t g_ms_tick = 0;

void SysTick_Handler(void) {
    g_ms_tick++;
}

void delay_ms(uint32_t ms) {
    uint32_t start = g_ms_tick;
    while ((g_ms_tick - start) < ms);
}

void delay_us(uint32_t us) {
    TIM_SetCounter(TIM4, 0);
    while (TIM_GetCounter(TIM4) < us);
}

/* --- Initialization --- */
void RCC_Configuration(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO  |
                           RCC_APB2Periph_ADC1  | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM3 | 
                           RCC_APB1Periph_TIM4, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
}

void GPIO_Configuration(void) {
    GPIO_InitTypeDef gpio;

    // USART1 (Debug)
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin   = GPIO_Pin_9;  GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Pin   = GPIO_Pin_10; GPIO_Init(GPIOA, &gpio);

    // USART3 (CO2 Sensor)
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin   = GPIO_Pin_10; GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Pin   = GPIO_Pin_11; GPIO_Init(GPIOB, &gpio);

    // LDR (PC0)
    gpio.GPIO_Mode  = GPIO_Mode_AIN;
    gpio.GPIO_Pin   = LIGHT_PIN;   GPIO_Init(LIGHT_PORT, &gpio);

    // DHT22 (PB0)
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin   = DHT22_PIN;   GPIO_Init(DHT22_PORT, &gpio);
    GPIO_SetBits(DHT22_PORT, DHT22_PIN);

    // Fan Control (PA4) - Transistor Switching
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin   = FAN_PIN;     GPIO_Init(FAN_PORT, &gpio);
    GPIO_ResetBits(FAN_PORT, FAN_PIN); // Fan OFF initially
}

void Servo_Init_Config(void) {
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    // PB1 (TIM3_CH4)
    gpio.GPIO_Pin = SERVO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_PORT, &gpio);

    // TIM3 (50Hz)
    tim.TIM_Prescaler = 72 - 1;
    tim.TIM_Period = 20000 - 1; 
    tim.TIM_ClockDivision = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tim);

    // PWM Channel 4
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 1500; 
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM3, &oc); 
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_Cmd(TIM3, ENABLE);
}

void USART_Init_Config(void) {
    USART_InitTypeDef us;
    // USART1 (115200)
    us.USART_BaudRate = 115200;
    us.USART_WordLength = USART_WordLength_8b;
    us.USART_StopBits = USART_StopBits_1;
    us.USART_Parity = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &us);
    USART_Cmd(USART1, ENABLE);

    // USART3 (9600)
    us.USART_BaudRate = 9600;
    USART_Init(USART3, &us);
    USART_Cmd(USART3, ENABLE);
}

void ADC_TIM_Config(void) {
    ADC_InitTypeDef adc;
    TIM_TimeBaseInitTypeDef tim;
    // ADC1
    ADC_DeInit(ADC1);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    // TIM4 (Delay)
    tim.TIM_Period = 0xFFFF;
    tim.TIM_Prescaler = 72 - 1; 
    tim.TIM_ClockDivision = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &tim);
    TIM_Cmd(TIM4, ENABLE);
}

void Init_Peripherals(void) {
    RCC_Configuration();
    GPIO_Configuration();
    USART_Init_Config();
    ADC_TIM_Config();
    Servo_Init_Config();
    SysTick_Config(SystemCoreClock / 1000);
}


/* --- Actuator Functions --- */
void Servo_Write(uint8_t angle) {
    uint16_t pulse = 500 + (angle * 2000 / 180);
    if (pulse < 500) pulse = 500;
    if (pulse > 2500) pulse = 2500;
    TIM_SetCompare4(TIM3, pulse);
}

void Fan_Control(int on_off) {
    if (on_off) GPIO_SetBits(FAN_PORT, FAN_PIN);   // Fan ON
    else        GPIO_ResetBits(FAN_PORT, FAN_PIN); // Fan OFF
}

/* --- Sensor Functions --- */
float Light_ReadLux(void) {
    uint32_t sum = 0;
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
    for (int i = 0; i < 8; i++) {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        sum += ADC_GetConversionValue(ADC1);
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    }
    return ((sum / 8.0f) / 4095.0f) * 1500.0f;
}

// DHT22 logic
#define DHT_OUT() { GPIOB->CRL = (GPIOB->CRL & 0xFFFFFFF0) | 0x3; }
#define DHT_IN()  { GPIOB->CRL = (GPIOB->CRL & 0xFFFFFFF0) | 0x4; }
#define DHT_GET() ((GPIOB->IDR & GPIO_Pin_0) != 0)
#define MAX_T     100000

int DHT22_Read(float *temp, float *hum) {
    uint8_t data[5] = {0};
    uint32_t cnt, width;
    __disable_irq();
    DHT_OUT(); GPIOB->BRR = GPIO_Pin_0; delay_us(2000);
    GPIOB->BSRR = GPIO_Pin_0; delay_us(30); DHT_IN();
    cnt = 0; while (DHT_GET()) if (++cnt > MAX_T) { __enable_irq(); return -1; }
    cnt = 0; while (!DHT_GET()) if (++cnt > MAX_T) { __enable_irq(); return -2; }
    cnt = 0; while (DHT_GET()) if (++cnt > MAX_T) { __enable_irq(); return -3; }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 8; j++) {
            cnt = 0; while (!DHT_GET()) if (++cnt > MAX_T) { __enable_irq(); return -4; }
            width = 0;
            while (DHT_GET()) { if (++width > MAX_T) { __enable_irq(); return -5; } }
            if (width > 125) data[i] |= (1 << (7 - j));
        }
    }
    __enable_irq();
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) return -6;
    *hum = ((data[0] << 8) | data[1]) / 10.0f;
    uint16_t raw_t = (data[2] << 8) | data[3];
    *temp = (raw_t & 0x8000) ? -(raw_t & 0x7FFF) / 10.0f : raw_t / 10.0f;
    return 0;
}

// HX-2000U logic (Dummy implementation)
int CO2_Read(uint16_t *ppm) {
    // Communication logic omitted for brevity as we use fixed value
    return 0; 
}

float Calc_HumidityRatio(float temp, float hum) {
    float Es = 6.112f * expf((17.67f * temp) / (temp + 243.5f));
    float E  = (hum / 100.0f) * Es;
    return 0.622f * (E / (1013.25f - E));
}

// Occupancy Model
int classify_human(float *data) {
    if (data[2] > 850) { // Light
        if (data[2] <= 279.041672) return data[2] <= 190.75 ? 0 : 0;
        else return data[0] <= 21.722501 ? 0 : 0;
    } else {
        if (data[3] <= 486.975006) return 1;
        else return data[1] <= 26.893333 ? 1 : 1;
    }
}

/* --- Main --- */
int main(void) {
    Init_Peripherals(); // 초기화 (PA4는 Output Push-Pull로 설정됨)

    printf("\r\n=== STM32 System Started ===\r\n");
    delay_ms(2000);

    float temp, hum, lux, hr;
    uint16_t co2 = 1000; // CO2 값 고정
    float model_data[4];
    int isHuman;

    while (1) {
        // 1. 센서 값 읽기
        lux = Light_ReadLux();
        DHT22_Read(&temp, &hum);
        hr = Calc_HumidityRatio(temp, hum);

        // 2. 모델 데이터 준비
        model_data[0] = temp;
        model_data[1] = hum;
        model_data[2] = lux;
        model_data[3] = (float)co2;

        // 3. 사람 인식 판별 (1: 있음, 0: 없음)
        isHuman = classify_human(model_data);

        // ====================================================
        // [핵심] PA4 핀 제어 로직 (Fan)
        // isHuman이 1이면 PA4 = 1 (High), 0이면 PA4 = 0 (Low)
        // ====================================================
        if (isHuman == 1) {
            GPIO_SetBits(GPIOA, GPIO_Pin_4);   // PA4 -> 1 (ON)
            Servo_Write(90);                   // (옵션) 문 열기
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_4); // PA4 -> 0 (OFF)
            Servo_Write(0);                    // (옵션) 문 닫기
        }

        // 4. 결과 출력
        printf("Temp:%.1f, Hum:%.1f, Lux:%.0f, CO2:%u, Human:%d, Fan(PA4):%d\r\n", 
               temp, hum, lux, co2, isHuman, isHuman);

        delay_ms(2000);
    }
}
