/*
    This is main.c file for STM32 microcontroller to read sensors:
    - DHT22 (Temperature and Humidity) : Done
    - LDR (Light Intensity) : Done
    - HX-2000U (CO2 Concentration) : TODO
*/

#include "stm32f10x.h"
#include "stdio.h"
#include "math.h"

/* --- Pin Definitions --- */
#define DHT22_PORT      GPIOB
#define DHT22_PIN       GPIO_Pin_0
#define LIGHT_PORT      GPIOC
#define LIGHT_PIN       GPIO_Pin_0

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

/* --- Initialization Functions --- */
void RCC_Configuration(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO  |
                           RCC_APB2Periph_ADC1  | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM4, ENABLE);
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

    // USART3 (CO2 Sensor: PB10 TX, PB11 RX)
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

    // USART3 (9600) [cite: 238]
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
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // TIM4
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
    SysTick_Config(SystemCoreClock / 1000);
}

/* --- Retarget Printf --- */
int fputc(int ch, FILE *f) {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}

/* --- Sensor Functions --- */

// LDR Sensor
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

// DHT22 Sensor
#define DHT_OUT() { GPIOB->CRL = (GPIOB->CRL & 0xFFFFFFF0) | 0x3; }
#define DHT_IN()  { GPIOB->CRL = (GPIOB->CRL & 0xFFFFFFF0) | 0x4; }
#define DHT_GET() ((GPIOB->IDR & GPIO_Pin_0) != 0)
#define MAX_T     100000

int DHT22_Read(float *temp, float *hum) {
    uint8_t data[5] = {0};
    uint32_t cnt, width;

    __disable_irq();
    DHT_OUT();
    GPIOB->BRR = GPIO_Pin_0; delay_us(2000);
    GPIOB->BSRR = GPIO_Pin_0; delay_us(30);
    DHT_IN();

    cnt = 0; while (DHT_GET())  if (++cnt > MAX_T) { __enable_irq(); return -1; }
    cnt = 0; while (!DHT_GET()) if (++cnt > MAX_T) { __enable_irq(); return -2; }
    cnt = 0; while (DHT_GET())  if (++cnt > MAX_T) { __enable_irq(); return -3; }

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 8; j++) {
            cnt = 0; while (!DHT_GET()) if (++cnt > MAX_T) { __enable_irq(); return -4; }
            width = 0;
            while (DHT_GET()) {
                if (++width > MAX_T) { __enable_irq(); return -5; }
            }
            if (width > 125) data[i] |= (1 << (7 - j));
        }
    }
    __enable_irq();

    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) return -6;

    uint16_t raw_h = (data[0] << 8) | data[1];
    uint16_t raw_t = (data[2] << 8) | data[3];

    *hum = raw_h / 10.0f;
    *temp = (raw_t & 0x8000) ? -(raw_t & 0x7FFF) / 10.0f : raw_t / 10.0f;
    return 0;
}

// HX-2000U CO2 Sensor (Protocol based on Spec Sheet)
int CO2_Read(uint16_t *ppm) {
    uint8_t req[7] = {0x42, 0x4D, 0xE3, 0x00, 0x00, 0x01, 0x72}; // [cite: 243]
    uint8_t buf[12] = {0};
    uint32_t timeout;

    // 1. Flush Buffer
    while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET) USART_ReceiveData(USART3);

    // 2. Send Request
    for (int i = 0; i < 7; i++) {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
        USART_SendData(USART3, req[i]);
    }
    
    delay_ms(10); 

    // 3. Receive Response (12 Bytes)
    for (int i = 0; i < 12; i++) {
        timeout = 500000;
        while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET) {
            if (--timeout == 0) return -1;
        }
        buf[i] = (uint8_t)USART_ReceiveData(USART3);
    }

    // 4. Validate Header
    if (buf[0] != 0x42 || buf[1] != 0x4D) return -2;

    // 5. Validate Checksum [cite: 251]
    uint16_t sum = 0;
    for (int i = 0; i < 10; i++) sum += buf[i];
    
    if ((uint8_t)(sum / 256) != buf[10] || (uint8_t)(sum % 256) != buf[11]) {
        printf("[Err] ChkSum: Calc(%02X%02X) != Recv(%02X%02X)\r\n", 
               (uint8_t)(sum/256), (uint8_t)(sum%256), buf[10], buf[11]);
        return -3;
    }

    // 6. Calculate PPM (Index 4 & 5) [cite: 255]
    *ppm = ((uint16_t)buf[4] << 8) | buf[5];
    return 0;
}

float Calc_HumidityRatio(float temp, float hum) {
    float Es = 6.112f * expf((17.67f * temp) / (temp + 243.5f));
    float E  = (hum / 100.0f) * Es;
    return 0.622f * (E / (1013.25f - E));
}

/* --- Main --- */
int main(void) {
    Init_Peripherals();

    printf("\r\n=== STM32 HX-2000U System Ready ===\r\n");
    delay_ms(2000);

    float temp, hum, lux, hr;
    uint16_t co2;
    int co2_stat;

    while (1) {
        lux = Light_ReadLux();
        DHT22_Read(&temp, &hum);
        co2_stat = CO2_Read(&co2);
        hr = Calc_HumidityRatio(temp, hum);

        if (co2_stat == 0)
            printf("%.1f, %.1f, %.0f, %u, %.5f\r\n", temp, hum, lux, co2, hr);
        else
            printf("%.1f, %.1f, %.0f, [Err:%d], %.5f\r\n", temp, hum, lux, co2_stat, hr);

        delay_ms(2000);
    }
}