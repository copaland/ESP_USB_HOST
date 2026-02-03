#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "config.h"
#include <stdio.h>
#include <time.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <driver/i2c.h>

static const char *SENSOR_TAG = "Sensor_Manager";

// PCF8563 RTC 레지스터 정의
#define PCF8563_REG_CONTROL1    0x00
#define PCF8563_REG_CONTROL2    0x01
#define PCF8563_REG_SECONDS     0x02
#define PCF8563_REG_MINUTES     0x03
#define PCF8563_REG_HOURS       0x04
#define PCF8563_REG_DAYS        0x05
#define PCF8563_REG_WEEKDAYS    0x06
#define PCF8563_REG_MONTHS      0x07
#define PCF8563_REG_YEARS       0x08

class SensorManager {
private:
    adc_oneshot_unit_handle_t adc_handle = nullptr;  // USBManager에서 공유받을 핸들
    float last_level = 0;
    
    // BCD to Binary 변환
    uint8_t bcd2bin(uint8_t val) {
        return (val >> 4) * 10 + (val & 0x0f);
    }

    // Binary to BCD 변환
    uint8_t bin2bcd(uint8_t val) {
        return ((val / 10) << 4) + (val % 10);
    }

public:
    struct SensorData {
        char date[16];          // "YYYY-MM-DD"
        char time[16];          // "HH:MM:SS"
        char level;             // 레벨 (H/M/L/A)
        float vbus_voltage;     // VBUS 전압
        char status;            // 상태 (O/X)
    };

    SensorManager() {}

    // ADC 핸들을 외부에서 설정 (USBManager로부터)
    void setADCHandle(adc_oneshot_unit_handle_t handle) {
        adc_handle = handle;
    }

    bool init(adc_oneshot_unit_handle_t shared_adc_handle = nullptr) {
        // GPIO LED 초기화
        gpio_config_t gpio_cfg = {};
        gpio_cfg.pin_bit_mask = (1ULL << LED_RED) | (1ULL << LED_YELLOW) | 
                                (1ULL << LED_GREEN) | (1ULL << LED_BLUE);
        gpio_cfg.mode = GPIO_MODE_OUTPUT;
        gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&gpio_cfg);

        // 모든 LED 초기화
        gpio_set_level(LED_RED, 0);
        gpio_set_level(LED_YELLOW, 0);
        gpio_set_level(LED_GREEN, 0);
        gpio_set_level(LED_BLUE, 0);

        // ADC 초기화 (USBManager에서 공유받음)
        if (shared_adc_handle != nullptr) {
            adc_handle = shared_adc_handle;
            ESP_LOGI(SENSOR_TAG, "Using shared ADC handle from USBManager");
        } else {
            ESP_LOGE(SENSOR_TAG, "No ADC handle provided!");
            return false;
        }

        // 레벨 센서 채널 설정
        adc_oneshot_chan_cfg_t config = {
            .atten = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LEVEL_SENSOR_CHAN, &config));

        // I2C 초기화 (RTC)
        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_SDA;
        conf.scl_io_num = I2C_SCL;
        conf.master.clk_speed = I2C_FREQ_HZ;
        conf.clk_flags = 0;
        
        ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
        ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

        // RTC 초기화
        initRTC();

        ESP_LOGI(SENSOR_TAG, "Sensor Manager initialized");
        return true;
    }

    // RTC 초기화 (필요시 시간 설정)
    void initRTC() {
        // RTC 레지스터 읽기 테스트
        uint8_t reg_addr = PCF8563_REG_SECONDS;
        uint8_t seconds = 0;
        
        // 레지스터 주소 쓰기 후 읽기
        esp_err_t ret = i2c_master_write_read_device(I2C_PORT, RTC_I2C_ADDR,
                                                      &reg_addr, 1, &seconds, 1,
                                                      pdMS_TO_TICKS(100));
        
        if (ret != ESP_OK) {
            ESP_LOGE(SENSOR_TAG, "RTC communication failed: %s", esp_err_to_name(ret));
            ESP_LOGI(SENSOR_TAG, "Check I2C connections and RTC address (0x%02X)", RTC_I2C_ADDR);
            return;
        }
        
        // VL 비트 확인 (bit 7, Power loss indicator)
        if (seconds & 0x80) {
            ESP_LOGW(SENSOR_TAG, "RTC power lost, setting to 2026-02-03 00:00:00");
            // 현재 날짜로 설정: 2026-02-03 00:00:00
            setRTCTime(2026, 2, 3, 0, 0, 0);
        } else {
            // RTC 시간 확인 및 출력
            int year, month, day, hour, minute, second;
            uint8_t time_regs[7];
            esp_err_t ret = i2c_master_write_read_device(I2C_PORT, RTC_I2C_ADDR,
                                                          &reg_addr, 1, time_regs, 7,
                                                          pdMS_TO_TICKS(100));
            if (ret == ESP_OK) {
                year = 2000 + bcd2bin(time_regs[6]);
                month = bcd2bin(time_regs[5] & 0x1f);
                day = bcd2bin(time_regs[3] & 0x3f);
                hour = bcd2bin(time_regs[2] & 0x3f);
                minute = bcd2bin(time_regs[1] & 0x7f);
                second = bcd2bin(time_regs[0] & 0x7f);
                ESP_LOGI(SENSOR_TAG, "RTC initialized - Current time: %04d-%02d-%02d %02d:%02d:%02d",
                         year, month, day, hour, minute, second);
            }
        }
    }

    // RTC에 시간 설정
    bool setRTCTime(int year, int month, int day, 
                    int hour, int minute, int second) {
        uint8_t buffer[8];
        
        buffer[0] = PCF8563_REG_SECONDS;        // 레지스터 주소
        buffer[1] = bin2bcd(second) & 0x7f;     // VL 비트 제거
        buffer[2] = bin2bcd(minute);
        buffer[3] = bin2bcd(hour);
        buffer[4] = bin2bcd(day);
        buffer[5] = 0;                           // weekday (자동 계산됨)
        buffer[6] = bin2bcd(month);
        buffer[7] = bin2bcd(year % 100);

        esp_err_t ret = i2c_master_write_to_device(I2C_PORT, RTC_I2C_ADDR, 
                                                     buffer, 8, pdMS_TO_TICKS(100));
        
        if (ret == ESP_OK) {
            ESP_LOGI(SENSOR_TAG, "RTC time set: %04d-%02d-%02d %02d:%02d:%02d",
                     year, month, day, hour, minute, second);
        } else {
            ESP_LOGE(SENSOR_TAG, "Failed to set RTC time: %s", esp_err_to_name(ret));
        }
        
        return ret == ESP_OK;
    }

    // RTC에서 현재 시간 읽기
    void getRTCTime(int& year, int& month, int& day,
                    int& hour, int& minute, int& second) {
        uint8_t reg_addr = PCF8563_REG_SECONDS;
        uint8_t buffer[7] = {0};
        
        // 레지스터 주소 쓰기 후 7바이트 읽기
        esp_err_t ret = i2c_master_write_read_device(I2C_PORT, RTC_I2C_ADDR,
                                                      &reg_addr, 1, buffer, 7,
                                                      pdMS_TO_TICKS(100));
        
        if (ret != ESP_OK) {
            ESP_LOGW(SENSOR_TAG, "RTC read failed: %s, using default time", esp_err_to_name(ret));
            // RTC 읽기 실패 시 기본 시간 사용
            year = 1970;
            month = 1;
            day = 1;
            hour = 0;
            minute = 0;
            second = 0;
            return;
        }

        second = bcd2bin(buffer[0] & 0x7f);
        minute = bcd2bin(buffer[1] & 0x7f);
        hour = bcd2bin(buffer[2] & 0x3f);
        day = bcd2bin(buffer[3] & 0x3f);
        // buffer[4]는 weekday (건너뛁)
        month = bcd2bin(buffer[5] & 0x1f);
        year = 2000 + bcd2bin(buffer[6]);
        
        // 유효성 검사 (BCD 값이 이상하면 기본값 사용)
        if (year < 2000 || year > 2100 || month < 1 || month > 12 || 
            day < 1 || day > 31 || hour > 23 || minute > 59 || second > 59) {
            ESP_LOGW(SENSOR_TAG, "Invalid RTC data: %04d-%02d-%02d %02d:%02d:%02d",
                     year, month, day, hour, minute, second);
            year = 1970;
            month = 1;
            day = 1;
            hour = 0;
            minute = 0;
            second = 0;
        }
    }

    // 레벨 센서 읽기
    float readLevelSensor() {
        int adc_raw = 0;
        adc_oneshot_read(adc_handle, LEVEL_SENSOR_CHAN, &adc_raw);
        
        float level = (adc_raw / 4095.0) * 100.0;
        if (level < 0) level = 0;
        if (level > 100) level = 100;
        
        last_level = level;
        return level;
    }

    // 날짜와 시간 문자열 생성 (분리)
    void getDateTimeStrings(char* date_buffer, size_t date_size,
                           char* time_buffer, size_t time_size) {
        int year, month, day, hour, minute, second;
        getRTCTime(year, month, day, hour, minute, second);
        
        snprintf(date_buffer, date_size, "%04d-%02d-%02d",
                year, month, day);
        snprintf(time_buffer, time_size, "%02d:%02d:%02d",
                hour, minute, second);
    }

    // 센서 데이터 수집
    void collectData(SensorData& data, float vbus_voltage, bool usb_connected) {
        getDateTimeStrings(data.date, sizeof(data.date),
                          data.time, sizeof(data.time));
        float level_percent = readLevelSensor();
        
        // 레벨을 H/M/L/A로 변환 (현재는 모두 H로 고정)
        // TODO: 실제 센서 값에 따라 H/M/L/A 결정
        data.level = 'H';  // High
        
        data.vbus_voltage = vbus_voltage;
        data.status = usb_connected ? 'O' : 'X';
    }

    // 센서 데이터를 CSV 형식으로 변환
    void formatAsCSV(const SensorData& data, char* csv_line, size_t buffer_size) {
        // Date와 Time을 따옴표로 감싸서 Excel의 자동 형식 변환 방지
        snprintf(csv_line, buffer_size, "\"%s\",\"%s\",%.2f,%c,%c\n",
                data.date, data.time, data.vbus_voltage, data.level, data.status);
    }

    // LED 상태 업데이트 (레벨 표시)
    void updateStatusLED(float level) {
        gpio_set_level(LED_RED, level < 25 ? 1 : 0);
        gpio_set_level(LED_YELLOW, (level >= 25 && level < 50) ? 1 : 0);
        gpio_set_level(LED_GREEN, (level >= 50 && level < 75) ? 1 : 0);
        gpio_set_level(LED_BLUE, level >= 75 ? 1 : 0);
    }

    void cleanup() {
        if (adc_handle != nullptr) {
            adc_oneshot_del_unit(adc_handle);
        }
        i2c_driver_delete(I2C_PORT);
    }
};

#endif // SENSOR_MANAGER_H