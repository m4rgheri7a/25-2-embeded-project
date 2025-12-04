#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"

// ---------------------------------------------------------------------------
// 시스템 모드 정의
// ---------------------------------------------------------------------------
#define MODE_POWERSAVE   0  // 0번 모드 : 절전모드
#define MODE_VENT        1  // 1번 모드 : 환기모드 (Ventilation)
#define MODE_DEFAULT     2  // 2번 모드 : 디폴트(사람 있을 때 기본)
#define MODE_SLEEP       3  // 3번 모드 : 수면모드
#define MODE_COOL        4  // 4번 모드 : 냉방모드

// ---------------------------------------------------------------------------
// 서보모터 논리 모드 정의
//  - 에어컨 풍향 서보 : SUBO_MOTOR_MODE 값으로 표현
//  - 창문 서보        : WINDOW_MOTOR_MODE 값으로 표현
// ---------------------------------------------------------------------------

// 에어컨 풍향 서보 모드 (controlSuboMotor()의 리턴값 / SUBO_MOTOR_MODE 사용)
#define AC_SERVO_MODE_UNKNOWN   0
#define AC_SERVO_MODE_UP        1   // 바람 위쪽
#define AC_SERVO_MODE_DOWN      2   // 바람 아래쪽

// 창문 서보 모드 (내부적으로만 사용, WINDOW_MOTOR_MODE에 저장)
#define WINDOW_MODE_UNKNOWN     0
#define WINDOW_MODE_CLOSE       1   // 창문 닫힘
#define WINDOW_MODE_OPEN        2   // 창문 열림

// LED 모드 정의 (상태등 두 개)
//  - LED1 (PD2) : 에어컨 가동 여부 표시 (냉방 모드)
//  - LED2 (PD3) : 수면 모드 표시 (수면 전용, 저항으로 밝기 낮춤)
#define LED_MODE_OFF            0
#define LED_MODE_COOL_ON        1   // 냉방 모드 : LED1 ON
#define LED_MODE_SLEEP_ON       2   // 수면 모드 : LED2 ON

// 팬모터 모드 정의
#define FAN_MODE_OFF           0
#define FAN_MODE_VENT          1  // 환기용
#define FAN_MODE_COOL          2  // 냉방용

// ----------------------------------------------------------------------------
// 전역 상태
// ----------------------------------------------------------------------------

// 현재 시스템 모드 (상태머신 파트에서 변경해 줄 값)
volatile int g_SystemMode = MODE_DEFAULT;

// AI/센서 파트에서 업데이트해 줄 사람 유무 (0: 없음, 1: 있음)
volatile int g_IsHuman = 0;

// 컨트롤부가 계산한 최종 모드 값 (리턴값 그대로 사용)
// SUBO_MOTOR_MODE   : 에어컨 풍향 서보 모드
// WINDOW_MOTOR_MODE : 창문 서보 모드
int SUBO_MOTOR_MODE = AC_SERVO_MODE_UNKNOWN;
int WINDOW_MOTOR_MODE = WINDOW_MODE_UNKNOWN;
int LED_MODE = LED_MODE_OFF;
int FAN_MOTOR_MODE = FAN_MODE_OFF;

// AI 쪽에서 쓰는 사람 유무 모드
int RESIDE_MODE = 0;

// ----------------------------------------------------------------------------
// LCD 관련 위치 매크로 & 함수 프로토타입
// ----------------------------------------------------------------------------

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

void LCD_InitUI(void);
void LCD_UpdateUI(void);
static const char* GetModeString(int mode);

// ----------------------------------------------------------------------------
// 하드웨어 제어용 내부 함수 프로토타입
// ----------------------------------------------------------------------------

// 에어컨 풍향 서보
static void ApplyAcServoMode(int mode);
// 창문 서보
static void ApplyWindowServoMode(int mode);
// LED, 팬
static void ApplyLedMode(int mode);
static void ApplyFanMotorMode(int mode);

// ----------------------------------------------------------------------------
// 센서 값 읽기 – 의철 파트 구현 영역 (지금은 더미)
// ---------------------------------------------------------------------------- 

float getCO2() {
    float co2 = 0.0f;
    // TODO: 의철 파트에서 실제 센서값 읽어오기 (ppm)
    return co2;
}

float getHumidity() {
    float humididy = 0.0f;
    // TODO: 의철 파트에서 실제 센서값 읽어오기 (%)
    return humididy;
}

float getTemperature() {
    float temperature = 0.0f;
    // TODO: 의철 파트에서 실제 센서값 읽어오기 (섭씨)
    return temperature;
}

float getIlluminance() {
    float illuminance = 0.0f;
    // TODO: 의철 파트에서 실제 센서값 읽어오기 (ADC 값 등)
    return illuminance;
}

// ----------------------------------------------------------------------------
// 컨트롤부 – 서보모터 / LED / 팬모터 제어 로직
// ---------------------------------------------------------------------------- 

int controlSuboMotor() {
    // 이 함수는 "서보모터 전체 제어"를 담당한다.
    //  - SUBO_MOTOR_MODE   : 에어컨 풍향 서보 모드 (리턴값)
    //  - WINDOW_MOTOR_MODE : 창문 서보 모드 (전역)

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

    // 실제 하드웨어에 반영
    ApplyAcServoMode(ac_mode);
    ApplyWindowServoMode(window_mode);

    return SUBO_MOTOR_MODE;
}

int controlLED() {
    int mode = LED_MODE_OFF;

    switch (g_SystemMode) {
    case MODE_COOL:
        // 냉방 모드 : 에어컨 가동 상태등 ON (LED1)
        mode = LED_MODE_COOL_ON;
        break;

    case MODE_SLEEP:
        // 수면 모드 : 수면 상태등 ON (LED2, 물리적으로 저항 더 커서 어둡게 설계)
        mode = LED_MODE_SLEEP_ON;
        break;

    case MODE_POWERSAVE:
    case MODE_VENT:
    case MODE_DEFAULT:
    default:
        // 절전 / 환기 / 디폴트 / 나머지 : 상태등 OFF
        mode = LED_MODE_OFF;
        break;
    }

    LED_MODE = mode;
    ApplyLedMode(mode);

    return LED_MODE;
}

int controlFanMotor() {
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

// ----------------------------------------------------------------------------
// 유성 – AI / 사람 유무 판단 (템플릿 유지, 안에만 더미)
// ---------------------------------------------------------------------------- 

int model(float co2, float humididy, float temperature, float illuminance) {
    int isHuman = 0;
    // TODO: 유성 파트에서 AI 모델 결과를 여기서 계산
    (void)co2;
    (void)humididy;
    (void)temperature;
    (void)illuminance;
    return isHuman;
}
// ---------------------------------------------------------------------------- // 
// 유성
// ["Temperature", "Humidity", "Light", "CO2"]
int model(float co2, float humididy, float temperature, float illuminance){
    float data[4];

    // 순서대로 매핑
    data[0] = temperature;
    data[1] = humididy;
    data[2] = illuminance;
    data[3] = co2;

    // 결정트리 분류 호출
    int isHuman = classify_human(data);

    return isHuman;
}

// ["Temperature", "Humidity", "Light", "CO2"]
int classify_human(float *data) {
    if (data[2] <= 365.125000) {
        if (data[2] <= 279.041672) {
            if (data[2] <= 190.750000) {
                return 0;
            } else {
                return 0;
            }
        } else {
            if (data[0] <= 21.722501) {
                return 0;
            } else {
                return 0;
            }
        }
    } else {
        if (data[3] <= 486.975006) {
            return 1;
        } else {
            if (data[1] <= 26.893333) {
                return 1;
            } else {
                return 1;
            }
        }
    }
}

int isHumanIn(){
    // 모델 분류결과
    // 0 : 사람없음
    // 1 : 사람있음
    float co2 = getCO2();
    float humididy = getHumidity();
    float temperature = getTemperature();
    float illuminance = getIlluminance();
    int isHumanIn = model(co2, humididy, temperature, illuminance);

    return isHumanIn;
}

// ----------------------------------------------------------------------------
// LCD UI 구현부 – 현재 모드 / 센서 값 / 사람 유무를 화면에 출력
// ---------------------------------------------------------------------------- 

void LCD_InitUI(void) // 조도 값으로 LCD 배경 및 색 조절
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

static const char* GetModeString(int mode)
{
    switch (mode) { // struct data{  }
    case MODE_POWERSAVE: return "PWR "; //
    case MODE_VENT:      return "VENT"; // 
    case MODE_DEFAULT:   return "DFLT"; // 
    case MODE_SLEEP:     return "SLEE"; // 
    case MODE_COOL:      return "COOL"; // 23도 -> 목표 20도
    default:             return "UNKN";
    }
}

// ----------------------------------------------------------------------------
// 실제 하드웨어 제어 함수 – 핀/Timer 결선에 맞춰 이 부분만 수정하면 됨
// ----------------------------------------------------------------------------

// 에어컨 풍향 서보 : TIM3 CH3, PB0 에 연결되어 있다고 가정
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

    // TIM3 PWM 이 1MHz(1us 단위)로 설정되어 있다는 전제
    TIM_SetCompare3(TIM3, pulse);
}

// 창문 서보 : TIM3 CH4, PB1 에 연결되어 있다고 가정
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
//  - PD2 : 에어컨 상태등 (밝게, 일반 저항)
//  - PD3 : 수면 상태등   (어둡게, 더 큰 저항 사용)
static void ApplyLedMode(int mode)
{
    switch (mode) {
    case LED_MODE_COOL_ON:
        // 냉방 : 에어컨 상태등 ON, 수면등 OFF
        GPIO_SetBits(GPIOD, GPIO_Pin_2);
        GPIO_ResetBits(GPIOD, GPIO_Pin_3);
        break;

    case LED_MODE_SLEEP_ON:
        // 수면 : 수면 상태등 ON(저항으로 밝기 낮춘 LED), 에어컨등 OFF
        GPIO_ResetBits(GPIOD, GPIO_Pin_2);
        GPIO_SetBits(GPIOD, GPIO_Pin_3);
        break;

    case LED_MODE_OFF:
    default:
        // 둘 다 OFF
        GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
        break;
    }
}

// 팬모터 : GPIOC Pin 8 (릴레이/트랜지스터 입력)
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

// ----------------------------------------------------------------------------
// main() – 상태머신/주기 제어는 나중에 팀이 같이 채우는 부분
// ----------------------------------------------------------------------------

int main() {
    while (1)
    {
        // 여기서:
        //  - 센서 읽고
        //  - isHumanIn();
        //  - g_SystemMode 상태머신으로 결정
        //  - controlSuboMotor();
        //  - controlLED();
        //  - controlFanMotor();
        //  - LCD_UpdateUI();
        // 를 주기적으로 호출하면 된다.
    }

    return 0;
}
