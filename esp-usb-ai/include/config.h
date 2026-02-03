#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ========== USB 핀 (Memory Stick 용) ==========
#define USB_DM          19
#define USB_DP          20

// ========== ADC 센서 ==========
#define LEVEL_SENSOR_PIN    2       // Level Sensor ADC
#define LEVEL_SENSOR_CHAN   ADC_CHANNEL_1  // GPIO2 = ADC1_CH1

// ========== USB VBUS 모니터링 ==========
#define USB_VBUS_ADC_PIN    1       // GPIO1 = ADC1_CH0
#define USB_VBUS_ADC_CHAN   ADC_CHANNEL_0

// ========== LED 제어 ==========
#define LED_RED         (gpio_num_t)4
#define LED_YELLOW      (gpio_num_t)5
#define LED_GREEN       (gpio_num_t)6
#define LED_BLUE        (gpio_num_t)7

// ========== I2C (RTC, 기타 센서) ==========
#define I2C_SDA         8
#define I2C_SCL         9
#define I2C_PORT        I2C_NUM_0
#define I2C_FREQ_HZ     100000

// ========== MODBUS RTU (UART1) ==========
#define MODBUS_UART_NUM UART_NUM_1
#define MODBUS_TXD      17
#define MODBUS_RXD      18
#define MODBUS_RW       16      // RS485 Direction Control
#define MODBUS_BAUDRATE 9600

// ========== VBUS 파워 제어 (STMPS2151STR) ==========
#define VBUS_EN         (gpio_num_t)10      // VBUS Enable (Active High)
#define USB_OVER_FLG    (gpio_num_t)21      // Over Current Flag (Active High)

// ========== 신호 및 알림 ==========
#define SIGNAL_LED      (gpio_num_t)47
#define BUZZER          (gpio_num_t)38

// ========== UART (Debug 용) ==========
#define UART0_TXD       43
#define UART0_RXD       44
#define DEBUG_UART_BAUD 115200

// ========== USB File System ==========
#define USB_MOUNT_POINT "/usb"
#define CSV_FILE_PATH   "/usb/data.csv"

// ========== 타이밍 설정 ==========
#define USB_CHECK_INTERVAL      500     // USB 체크 간격 (ms)
#define DATA_SAVE_INTERVAL      10000   // 데이터 저장 간격 (10초)
#define SENSOR_READ_INTERVAL    1000    // 센서 읽기 간격 (ms)

// ========== ADC 설정 ==========
#define ADC_RESOLUTION          12      // 12-bit ADC
#define ADC_SAMPLES             10      // 평균값 샘플 수
#define ADC_ATTEN               ADC_ATTEN_DB_12  // Full-scale 3.3V (was ADC_ATTEN_DB_11)

// ========== VBUS 분압 계산 ==========
// VBUS(5V) -> 47K + 24.3K 분압 -> ADC
// Vout = 5V * 24.3K / (47K + 24.3K) ≈ 1.48V (3.3V 기준)
#define VBUS_VOLTAGE_DIVIDER    (47000.0 + 24300.0) / 24300.0
#define ADC_REFERENCE_VOLTAGE   3.3
#define VBUS_THRESHOLD          4.0     // 4.0V 이상 USB 연결로 인정
#define VBUS_MAX_VOLTAGE        5.5     // 최대 예상 전압

// ========== RTC 설정 ==========
#define RTC_I2C_ADDR    0x51    // PCF8563 I2C Address

// ========== Task 설정 ==========
#define USB_TASK_PRIORITY       5
#define SENSOR_TASK_PRIORITY    4
#define SAVE_TASK_PRIORITY      3
#define STACK_SIZE              8192  // 스택 크기 증가 (4096 → 8192)

#endif // CONFIG_H