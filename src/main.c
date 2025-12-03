#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "lcd.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h"

/*===========================
1. 핀 배치 (이 코드 기준)

[DHT22 (AM2302)]
    센서 1(VDD) → 3.3V
    센서 2(DATA) → PA1
    센서 3 -> Not connected 연결 X
    센서 4(GND) → GND
    센서 1–2 사이에 4.7kΩ 풀업

[조도센서(LDR + 저항)]
    3.3V ─ LDR ─●─ 20kΩ ─ GND
    ● 노드 → PC0 (ADC1_IN10)

[CO₂ 센서 HX-2000U]
    1(VCC) → 5V
    2(GND) → GND
    3(Tx) → (레벨다운 후) PA3 (USART2_RX)
    4(Rx) ← (레벨업 or 그대로) PA2 (USART2_TX)
    5(RST) → 5V에 10kΩ 풀업
    6 -> not connected
    7 -> not connected

[PC 터미널(UART1)]
    PA9 → USB-UART RX
    PA10 → USB-UART TX

[에어컨 풍향 서보]
    PB0 → TIM3_CH3 (PWM)

[창문 서보]
    PB1 → TIM3_CH4 (PWM)

[상태등 LED 2개]
    PD2 → 에어컨 가동 상태등 (밝게, 일반 저항)
    PD3 → 수면 상태등       (어둡게, 큰 저항)

[팬 모터 릴레이]
    PC8 → 팬 제어 (HIGH = ON)
 *===========================*/

#define DHT22_PORT     GPIOA
#define DHT22_PIN      GPIO_Pin_1      // PA1

#define LIGHT_PORT     GPIOC
#define LIGHT_PIN      GPIO_Pin_0      // PC0 (ADC1_IN10)

 /*===========================
  * 시스템 모드 정의
  *===========================*/
#define MODE_POWERSAVE   0  // 절전모드
#define MODE_VENT        1  // 환기모드
#define MODE_DEFAULT     2  // 디폴트(사람 있을 때 기본)
#define MODE_SLEEP       3  // 수면모드
#define MODE_COOL        4  // 냉방모드

  /*===========================
   * 서보/LED/팬 논리 모드 정의
   *===========================*/
   // 에어컨 풍향 서보 (AC)
#define AC_SERVO_MODE_UNKNOWN   0
#define AC_SERVO_MODE_UP        1   // 바람 위쪽
#define AC_SERVO_MODE_DOWN      2   // 바람 아래쪽

// 창문 서보
#define WINDOW_MODE_UNKNOWN     0
#define WINDOW_MODE_CLOSE       1   // 창문 닫힘
#define WINDOW_MODE_OPEN        2   // 창문 열림

// LED 상태등
//  - LED1 (PD2) : 에어컨 가동 여부 표시
//  - LED2 (PD3) : 수면 모드 표시 (저항으로 어둡게)
#define LED_MODE_OFF            0
#define LED_MODE_COOL_ON        1   // 냉방 모드 상태등
#define LED_MODE_SLEEP_ON       2   // 수면 모드 상태등

// 팬 모드
#define FAN_MODE_OFF           0
#define FAN_MODE_VENT          1  // 환기용
#define FAN_MODE_COOL          2  // 냉방용

/*===========================
 * 전역 변수
 *===========================*/
volatile uint32_t g_ms_tick = 0;

// 센서 값 (의철 파트)
float    g_temperature = 0.0f;
float    g_humidity = 0.0f;
uint16_t g_lightRaw = 0;     // 조도 Raw (0~4095)
uint16_t g_co2ppm = 0;

// 시스템/AI 상태 (유성 + 컨트롤 파트)
volatile int g_SystemMode = MODE_DEFAULT;  // 현재 시스템 모드
volatile int g_IsHuman = 0;             // 0: 없음, 1: 있음

int SUBO_MOTOR_MODE = AC_SERVO_MODE_UNKNOWN; // 에어컨 풍향 서보 모드
int WINDOW_MOTOR_MODE = WINDOW_MODE_UNKNOWN;   // 창문 서보 모드
int LED_MODE_STATE = LED_MODE_OFF;          // LED 상태 모드
int FAN_MOTOR_MODE = FAN_MODE_OFF;          // 팬 모드

int RESIDE_MODE = 0;                     // 사람 유무 모드(기존 템플릿 호환용)

/*===========================
 * 프로토타입
 *===========================*/
void SysTick_Handler(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

void RCC_Configuration(void);
void GPIO_Configuration(void);
void ADC_Configuration(void);
void USART1_Configuration(void);
void USART2_Configuration(void);
void TIM4_Configuration(void);
void TIM3_Configuration(void);

uint16_t Light_ReadRaw(void);
float    Light_ReadLux(void);

int  DHT22_Read(float* temp, float* hum);
int  CO2_Read(uint16_t* ppm);

float Calculate_HumidityRatio(float temp, float hum);

/* 컨트롤부 함수들 */
int   controlSuboMotor(void);
int   controlLED(void);
int   controlFanMotor(void);
int   model(float co2, float humididy, float temperature, float illuminance);
int   isHumanIn(void);

/* LCD UI */
void LCD_InitUI(void);
void LCD_UpdateUI(void);
static const char* GetModeString(int mode);

/* 센서 값 래퍼 (컨트롤/LCD에서 사용) */
float getTemperature(void);
float getHumidity(void);
float getIlluminance(void);
float getCO2(void);

/* 하드웨어 적용 함수 (서보/LED/팬) */
static void ApplyAcServoMode(int mode);
static void ApplyWindowServoMode(int mode);
static void ApplyLedMode(int mode);
static void ApplyFanMotorMode(int mode);

/* printf -> USART1 */
int fputc(int ch, FILE* f) {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}

/*===========================
 * SysTick & Delay
 *===========================*/
void SysTick_Handler(void) {
    g_ms_tick++;
}

void delay_ms(uint32_t ms) {
    uint32_t start = g_ms_tick;
    while ((g_ms_tick - start) < ms);
}

/* TIM4 1MHz 기준 us 딜레이 */
void delay_us(uint32_t us) {
    TIM_SetCounter(TIM4, 0);
    while (TIM_GetCounter(TIM4) < us);
}

/*===========================
 * 초기화 루틴
 *===========================*/
void RCC_Configuration(void) {
    /* GPIOA, GPIOB, GPIOC, GPIOD, ADC1, AFIO, USART1, TIM3 클럭 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC |
        RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_AFIO |
        RCC_APB2Periph_ADC1 |
        RCC_APB2Periph_USART1, ENABLE);

    /* USART2, TIM4, TIM3 (TIM3는 APB1) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 |
        RCC_APB1Periph_TIM4 |
        RCC_APB1Periph_TIM3, ENABLE);

    /* ADC 클럭: PCLK2 / 6 = 12MHz */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
}

void GPIO_Configuration(void) {
    GPIO_InitTypeDef gpio;

    /* USART1: PA9(TX) / PA10(RX) */
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* USART2: PA2(TX) / PA3(RX) */
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* 조도센서: PC0 (ADC 입력) */
    gpio.GPIO_Pin = LIGHT_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(LIGHT_PORT, &gpio);

    /* DHT22: PA1 – 시작은 출력으로 설정 (읽기 함수에서 in/out 전환) */
    gpio.GPIO_Pin = DHT22_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(DHT22_PORT, &gpio);

    /* 에어컨 풍향 서보: PB0 (TIM3_CH3, AF_PP) */
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    /* 창문 서보: PB1 (TIM3_CH4, AF_PP) */
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    /* 상태등 LED: PD2, PD3 (일반 출력) */
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &gpio);

    /* 팬 모터 릴레이: PC8 (일반 출력) */
    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio);

    /* 초기 상태: LED, 팬 OFF */
    GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
}

void ADC_Configuration(void) {
    ADC_InitTypeDef adc;

    ADC_DeInit(ADC1);

    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    ADC_Cmd(ADC1, ENABLE);

    /* Calibration */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

void USART1_Configuration(void) {
    USART_InitTypeDef us;

    us.USART_BaudRate = 115200;
    us.USART_WordLength = USART_WordLength_8b;
    us.USART_StopBits = USART_StopBits_1;
    us.USART_Parity = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &us);
    USART_Cmd(USART1, ENABLE);
}

void USART2_Configuration(void) {
    USART_InitTypeDef us;

    us.USART_BaudRate = 9600;     // HX-2000U 기본 설정
    us.USART_WordLength = USART_WordLength_8b;
    us.USART_StopBits = USART_StopBits_1;
    us.USART_Parity = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &us);
    USART_Cmd(USART2, ENABLE);
}

void TIM4_Configuration(void) {
    TIM_TimeBaseInitTypeDef tim;

    tim.TIM_Period = 0xFFFF;
    tim.TIM_Prescaler = 72 - 1;   // 72MHz / 72 = 1MHz (1us)
    tim.TIM_ClockDivision = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &tim);
    TIM_Cmd(TIM4, ENABLE);
}

/* 서보 PWM용 TIM3: 1MHz 기반, 주기 20ms(50Hz) */
void TIM3_Configuration(void) {
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    // 1MHz, 주기 20ms → Period = 20000-1
    tim.TIM_Period = 20000 - 1;
    tim.TIM_Prescaler = 72 - 1;     // 72MHz / 72 = 1MHz
    tim.TIM_ClockDivision = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tim);

    // 공통 PWM 설정
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = 1500; // 중간 위치

    // CH3 (PB0, 에어컨 서보)
    TIM_OC3Init(TIM3, &oc);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // CH4 (PB1, 창문 서보)
    TIM_OC4Init(TIM3, &oc);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

/*===========================
 * 조도 센서 (ADC)
 *===========================*/
uint16_t Light_ReadRaw(void) {
    /* PC0 → ADC1_IN10 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    uint16_t val = ADC_GetConversionValue(ADC1);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    return val;
}

/* Occupancy light 범위(0~1500 lux 근처)에 맞게 단순 스케일링 */
float Light_ReadLux(void) {
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += Light_ReadRaw();
    }
    float raw = sum / 8.0f;
    return (raw / 4095.0f) * 1500.0f;
}

/*===========================
 * DHT22(AM2302) 읽기
 *===========================*/
static void DHT22_SetOutput(void) {
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = DHT22_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(DHT22_PORT, &gpio);
}

static void DHT22_SetInput(void) {
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = DHT22_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // 외부 풀업 사용
    GPIO_Init(DHT22_PORT, &gpio);
}

/* AM2302/DHT22 – ONE-WIRE 프로토콜 구현 */
int DHT22_Read(float* temp, float* hum) {
    uint8_t data[5] = { 0 };
    uint32_t timeout;

    /* 1. Start signal: host pulls low >= 800us (여기선 1ms) */
    DHT22_SetOutput();
    GPIO_ResetBits(DHT22_PORT, DHT22_PIN);
    delay_ms(1);
    GPIO_SetBits(DHT22_PORT, DHT22_PIN);
    delay_us(30);

    /* 2. 센서 응답 대기: 80us low + 80us high */
    DHT22_SetInput();

    timeout = 10000;
    while (GPIO_ReadInputDataBit(DHT22_PORT, DHT22_PIN)) {
        if (--timeout == 0) return -1; // 응답 없음
        delay_us(1);
    }

    timeout = 10000;
    while (!GPIO_ReadInputDataBit(DHT22_PORT, DHT22_PIN)) {
        if (--timeout == 0) return -2; // low가 너무 길다
        delay_us(1);
    }

    timeout = 10000;
    while (GPIO_ReadInputDataBit(DHT22_PORT, DHT22_PIN)) {
        if (--timeout == 0) return -3; // high가 너무 길다
        delay_us(1);
    }

    /* 3. 40비트 데이터 수신 */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 8; j++) {
            /* 각 비트의 시작: 50us low */
            timeout = 10000;
            while (!GPIO_ReadInputDataBit(DHT22_PORT, DHT22_PIN)) {
                if (--timeout == 0) return -4;
                delay_us(1);
            }

            /* high가 얼마나 오래 유지되는지 측정 */
            uint32_t high_cnt = 0;
            while (GPIO_ReadInputDataBit(DHT22_PORT, DHT22_PIN)) {
                high_cnt++;
                delay_us(1);
                if (high_cnt > 100) break;
            }

            /* 26~28us → '0', 70us → '1' 정도.
               여기서는 40us를 기준으로 0/1 구분 */
            if (high_cnt > 40) {
                data[i] |= (1 << (7 - j));  // bit '1'
            }
        }
    }

    /* 4. 체크섬 확인 */
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4]) {
        return -5;
    }

    /* 5. 온습도 변환 (값은 ×10 형태로 전송됨) */
    uint16_t raw_h = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_t = ((uint16_t)data[2] << 8) | data[3];

    *hum = raw_h / 10.0f;

    if (raw_t & 0x8000) {
        raw_t &= 0x7FFF;
        *temp = -(raw_t / 10.0f);
    }
    else {
        *temp = raw_t / 10.0f;
    }

    return 0;
}

/*===========================
 * CO₂ 센서 HX-2000U 읽기 (UART 12바이트 프레임)
 *===========================*/

 /* Host request frame: 42 4D E3 00 00 01 72 */
static void CO2_SendRequest(void) {
    uint8_t req[7] = { 0x42, 0x4D, 0xE3, 0x00, 0x00, 0x01, 0x72 };
    for (int i = 0; i < 7; i++) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, req[i]);
    }
}

/* 12바이트 응답을 읽고 CO2 ppm 계산 */
int CO2_Read(uint16_t* ppm) {
    uint8_t buf[12];
    uint32_t timeout;

    CO2_SendRequest();

    /* 센서 샘플링 주기 1Hz라, 응답 대기 시간 여유 있게 500ms 정도 둠 */
    for (int i = 0; i < 12; i++) {
        timeout = 500000; // 500ms @1us step
        while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET) {
            if (--timeout == 0) return -1;
            delay_us(1);
        }
        buf[i] = (uint8_t)USART_ReceiveData(USART2);
    }

    /* 헤더 검사 */
    if (buf[0] != 0x42 || buf[1] != 0x4D || buf[2] != 0x00 || buf[3] != 0x08) {
        return -2;
    }

    /* 체크섬 계산 */
    uint16_t sum = 0;
    for (int i = 0; i < 10; i++) { // 0~9 바이트 합
        sum += buf[i];
    }
    uint8_t ch_high = sum / 256;
    uint8_t ch_low = sum % 256;

    if (ch_high != buf[10] || ch_low != buf[11]) {
        return -3;
    }

    uint16_t co2 = ((uint16_t)buf[4] << 8) | buf[5];
    *ppm = co2;

    return 0;
}

/*===========================
 * 습도비 계산 (optional)
 *===========================*/
float Calculate_HumidityRatio(float temp, float hum) {
    /* Magnus formula 기반 포화수증기압 (hPa) 근사 */
    float Es = 6.112f * expf((17.67f * temp) / (temp + 243.5f));
    float E = (hum / 100.0f) * Es;
    float P_atm = 1013.25f;     // hPa
    float w = 0.622f * (E / (P_atm - E));
    return w;   // kg/kg
}

/*===========================
 * 센서 래퍼 (컨트롤/LCD에서 사용)
 *===========================*/
float getTemperature(void) { return g_temperature; }
float getHumidity(void) { return g_humidity; }

/* 조도는 lux 단위로 반환 (Light_ReadLux 사용) */
float getIlluminance(void) {
    return Light_ReadLux();
}

/* CO2는 ppm */
float getCO2(void) {
    return (float)g_co2ppm;
}

/*===========================
 * 컨트롤부 – 서보/LED/팬 로직
 *===========================*/
int controlSuboMotor(void) {
    int ac_mode = AC_SERVO_MODE_UNKNOWN;
    int window_mode = WINDOW_MODE_UNKNOWN;

    switch (g_SystemMode) {
    case MODE_POWERSAVE:
    case MODE_DEFAULT:
    case MODE_SLEEP:
        // 절전 / 디폴트 / 수면 :
        //   - 창문 닫기
        //   - 풍향 위로
        ac_mode = AC_SERVO_MODE_UP;
        window_mode = WINDOW_MODE_CLOSE;
        break;

    case MODE_VENT:
        // 환기 :
        //   - 창문 개방 유지
        //   - 풍향은 위로
        ac_mode = AC_SERVO_MODE_UP;
        window_mode = WINDOW_MODE_OPEN;
        break;

    case MODE_COOL:
        // 냉방 :
        //   - 창문은 닫고 에어컨으로만 냉방
        //   - 사람이 있으면 바람 아래, 없으면 위로
        window_mode = WINDOW_MODE_CLOSE;
        if (g_IsHuman) {
            ac_mode = AC_SERVO_MODE_DOWN;
        }
        else {
            ac_mode = AC_SERVO_MODE_UP;
        }
        break;

    default:
        ac_mode = AC_SERVO_MODE_UNKNOWN;
        window_mode = WINDOW_MODE_UNKNOWN;
        break;
    }

    SUBO_MOTOR_MODE = ac_mode;
    WINDOW_MOTOR_MODE = window_mode;

    ApplyAcServoMode(ac_mode);
    ApplyWindowServoMode(window_mode);

    return SUBO_MOTOR_MODE;
}

int controlLED(void) {
    int mode = LED_MODE_OFF;

    switch (g_SystemMode) {
    case MODE_COOL:
        // 냉방 모드 : 에어컨 상태등 ON (PD2)
        mode = LED_MODE_COOL_ON;
        break;

    case MODE_SLEEP:
        // 수면 모드 : 수면 상태등 ON (PD3, 저항으로 어둡게)
        mode = LED_MODE_SLEEP_ON;
        break;

    case MODE_POWERSAVE:
    case MODE_VENT:
    case MODE_DEFAULT:
    default:
        mode = LED_MODE_OFF;
        break;
    }

    LED_MODE_STATE = mode;
    ApplyLedMode(mode);

    return LED_MODE_STATE;
}

int controlFanMotor(void) {
    int mode = FAN_MODE_OFF;

    switch (g_SystemMode) {
    case MODE_POWERSAVE:
    case MODE_DEFAULT:
    case MODE_SLEEP:
        mode = FAN_MODE_OFF;
        break;

    case MODE_VENT:
        mode = FAN_MODE_VENT;
        break;

    case MODE_COOL:
        mode = FAN_MODE_COOL;
        break;

    default:
        mode = FAN_MODE_OFF;
        break;
    }

    FAN_MOTOR_MODE = mode;
    ApplyFanMotorMode(mode);

    return FAN_MOTOR_MODE;
}

/*===========================
 * AI / 사람 유무 판단 (유성 파트)
 *===========================*/
int model(float co2, float humididy, float temperature, float illuminance) {
    int isHuman = 0;
    // TODO: 유성 파트에서 AI 모델 구현
    (void)co2;
    (void)humididy;
    (void)temperature;
    (void)illuminance;
    return isHuman;
}

int isHumanIn(void) {
    float co2 = getCO2();
    float humidity = getHumidity();
    float temperature = getTemperature();
    float illuminance = getIlluminance();

    int isHuman = model(co2, humidity, temperature, illuminance);

    g_IsHuman = isHuman;
    RESIDE_MODE = isHuman;

    return RESIDE_MODE;
}

/*===========================
 * LCD UI
 *===========================*/
#define LCD_TEAM_NAME_X   10
#define LCD_TEAM_NAME_Y   10

#define LCD_MODE_LABEL_X  10
#define LCD_MODE_LABEL_Y  40
#define LCD_MODE_VAL_X    70
#define LCD_MODE_VAL_Y    40

#define LCD_CO2_LABEL_X   10
#define LCD_CO2_LABEL_Y   60
#define LCD_CO2_VAL_X     70
#define LCD_CO2_VAL_Y     60

#define LCD_TEMP_LABEL_X  10
#define LCD_TEMP_LABEL_Y  80
#define LCD_TEMP_VAL_X    70
#define LCD_TEMP_VAL_Y    80

#define LCD_HUM_LABEL_X   10
#define LCD_HUM_LABEL_Y   100
#define LCD_HUM_VAL_X     70
#define LCD_HUM_VAL_Y     100

#define LCD_ILL_LABEL_X   10
#define LCD_ILL_LABEL_Y   120
#define LCD_ILL_VAL_X     70
#define LCD_ILL_VAL_Y     120

#define LCD_HUMAN_LABEL_X 10
#define LCD_HUMAN_LABEL_Y 140
#define LCD_HUMAN_VAL_X   70
#define LCD_HUMAN_VAL_Y   140

void LCD_InitUI(void)
{
    LCD_Init();
    LCD_Clear(WHITE);

    LCD_ShowString(LCD_TEAM_NAME_X, LCD_TEAM_NAME_Y, "WEN_03", BLUE, WHITE);

    LCD_ShowString(LCD_MODE_LABEL_X, LCD_MODE_LABEL_Y, "MODE:", BLACK, WHITE);
    LCD_ShowString(LCD_CO2_LABEL_X, LCD_CO2_LABEL_Y, "CO2:", BLACK, WHITE);
    LCD_ShowString(LCD_TEMP_LABEL_X, LCD_TEMP_LABEL_Y, "TEMP:", BLACK, WHITE);
    LCD_ShowString(LCD_HUM_LABEL_X, LCD_HUM_LABEL_Y, "HUMI:", BLACK, WHITE);
    LCD_ShowString(LCD_ILL_LABEL_X, LCD_ILL_LABEL_Y, "LUX:", BLACK, WHITE);
    LCD_ShowString(LCD_HUMAN_LABEL_X, LCD_HUMAN_LABEL_Y, "HUMAN:", BLACK, WHITE);
}

static const char* GetModeString(int mode)
{
    switch (mode) {
    case MODE_POWERSAVE: return "PWR ";
    case MODE_VENT:      return "VENT";
    case MODE_DEFAULT:   return "DFLT";
    case MODE_SLEEP:     return "SLEE";
    case MODE_COOL:      return "COOL";
    default:             return "UNKN";
    }
}

void LCD_UpdateUI(void)
{
    float co2_f = getCO2();
    float temp_f = getTemperature();
    float hum_f = getHumidity();
    float illum_f = getIlluminance();

    unsigned int co2 = (unsigned int)(co2_f + 0.5f);
    unsigned int temp = (unsigned int)(temp_f + 0.5f);
    unsigned int hum = (unsigned int)(hum_f + 0.5f);
    unsigned int illum = (unsigned int)(illum_f + 0.5f);

    LCD_ShowString(LCD_MODE_VAL_X, LCD_MODE_VAL_Y,
        (char*)GetModeString(g_SystemMode),
        BLUE, WHITE);

    LCD_ShowNum(LCD_CO2_VAL_X, LCD_CO2_VAL_Y, co2, 4, BLACK, WHITE);
    LCD_ShowNum(LCD_TEMP_VAL_X, LCD_TEMP_VAL_Y, temp, 2, BLACK, WHITE);
    LCD_ShowNum(LCD_HUM_VAL_X, LCD_HUM_VAL_Y, hum, 2, BLACK, WHITE);
    LCD_ShowNum(LCD_ILL_VAL_X, LCD_ILL_VAL_Y, illum, 4, BLACK, WHITE);

    if (g_IsHuman) {
        LCD_ShowString(LCD_HUMAN_VAL_X, LCD_HUMAN_VAL_Y, "YES ", RED, WHITE);
    }
    else {
        LCD_ShowString(LCD_HUMAN_VAL_X, LCD_HUMAN_VAL_Y, "NO  ", RED, WHITE);
    }
}

/*===========================
 * 실제 하드웨어 제어 (서보/LED/팬)
 *===========================*/

 // 에어컨 풍향 서보 : TIM3_CH3, PB0
#define AC_SERVO_PULSE_UP      1200  // us, 바람 위쪽
#define AC_SERVO_PULSE_DOWN    1800  // us, 바람 아래쪽

static void ApplyAcServoMode(int mode)
{
    uint16_t pulse = AC_SERVO_PULSE_UP;

    switch (mode) {
    case AC_SERVO_MODE_UP:
        pulse = AC_SERVO_PULSE_UP;
        break;
    case AC_SERVO_MODE_DOWN:
        pulse = AC_SERVO_PULSE_DOWN;
        break;
    case AC_SERVO_MODE_UNKNOWN:
    default:
        pulse = AC_SERVO_PULSE_UP;
        break;
    }

    TIM_SetCompare3(TIM3, pulse);
}

// 창문 서보 : TIM3_CH4, PB1
#define WINDOW_SERVO_PULSE_CLOSE   1000  // us, 창문 닫힘
#define WINDOW_SERVO_PULSE_OPEN    2000  // us, 창문 열림

static void ApplyWindowServoMode(int mode)
{
    uint16_t pulse = WINDOW_SERVO_PULSE_CLOSE;

    switch (mode) {
    case WINDOW_MODE_CLOSE:
        pulse = WINDOW_SERVO_PULSE_CLOSE;
        break;
    case WINDOW_MODE_OPEN:
        pulse = WINDOW_SERVO_PULSE_OPEN;
        break;
    case WINDOW_MODE_UNKNOWN:
    default:
        pulse = WINDOW_SERVO_PULSE_CLOSE;
        break;
    }

    TIM_SetCompare4(TIM3, pulse);
}

// LED : GPIOD Pin 2,3
static void ApplyLedMode(int mode)
{
    switch (mode) {
    case LED_MODE_COOL_ON:
        // 냉방 : 에어컨 상태등 ON, 수면등 OFF
        GPIO_SetBits(GPIOD, GPIO_Pin_2);
        GPIO_ResetBits(GPIOD, GPIO_Pin_3);
        break;

    case LED_MODE_SLEEP_ON:
        // 수면 : 수면 상태등 ON, 에어컨등 OFF
        GPIO_ResetBits(GPIOD, GPIO_Pin_2);
        GPIO_SetBits(GPIOD, GPIO_Pin_3);
        break;

    case LED_MODE_OFF:
    default:
        GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
        break;
    }
}

// 팬모터 : GPIOC Pin 8
static void ApplyFanMotorMode(int mode)
{
    switch (mode) {
    case FAN_MODE_OFF:
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        break;

    case FAN_MODE_VENT:
    case FAN_MODE_COOL:
        GPIO_SetBits(GPIOC, GPIO_Pin_8);
        break;

    default:
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        break;
    }
}

/*===========================
 * main
 *===========================*/
int main(void) {
    SystemInit();
    RCC_Configuration();
    GPIO_Configuration();
    ADC_Configuration();
    USART1_Configuration();
    USART2_Configuration();
    TIM4_Configuration();
    TIM3_Configuration();
    SysTick_Config(SystemCoreClock / 1000);  // 1ms tick

    LCD_InitUI();

    printf("\r\n=== Smart Cooling System Integrated ===\r\n");
    printf("Temp(C),Humidity(%%),Light(lux),CO2(ppm),HumidityRatio(kg/kg)\r\n");

    /* DHT22 최소 샘플링 주기 2초이므로, 첫 측정 전 약간 대기 */
    delay_ms(2000);

    while (1) {
        float temp = 0.0f, hum = 0.0f;
        float lux = 0.0f;
        uint16_t co2 = 0;
        float hr = 0.0f;

        /* 1) 조도 (lux) */
        lux = Light_ReadLux();
        // 필요하면 RAW도 저장해서 쓸 수 있음
        g_lightRaw = (uint16_t)((lux / 1500.0f) * 4095.0f);

        /* 2) 온습도 */
        if (DHT22_Read(&temp, &hum) == 0) {
            g_temperature = temp;
            g_humidity = hum;
        }
        else {
            printf("DHT22 read error\r\n");
        }

        /* 3) CO2 */
        if (CO2_Read(&co2) == 0) {
            g_co2ppm = co2;
        }
        else {
            printf("CO2 read error\r\n");
        }

        /* 4) 습도비 계산 (optional) */
        hr = Calculate_HumidityRatio(g_temperature, g_humidity);

        /* 5) 사람 유무 판단 (AI 모델은 유성이가 model()에 구현) */
        isHumanIn();

        /* 6) 모드 상태머신 (TODO: 유성/의철 파트에서 조건에 맞게 g_SystemMode 갱신)
         *
         *   예시:
         *   - 사람이 없고, 센서 조건 만족하면 MODE_POWERSAVE
         *   - 사람이 있고 CO2 높으면 MODE_VENT
         *   - 사람이 있고 온도 높으면 MODE_COOL
         *   - 사용자 블루투스 입력 들어오면 MODE_SLEEP
         *
         *   여기서는 일단 기본값인 MODE_DEFAULT 유지
         */

         /* 7) 컨트롤부: 서보/LED/팬 제어 */
        controlSuboMotor();
        controlLED();
        controlFanMotor();

        /* 8) LCD 갱신 */
        LCD_UpdateUI();

        /* 9) PC 터미널로 CSV 출력 (디버그/모델 학습용) */
        printf("%.2f,%.2f,%.1f,%u,%.6f\r\n",
            g_temperature, g_humidity, lux, g_co2ppm, hr);

        /* AM2302 최소 샘플링 주기 2s 준수 */
        delay_ms(2000);
    }
}
