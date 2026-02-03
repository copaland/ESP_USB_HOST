#ifndef USB_MANAGER_H
#define USB_MANAGER_H

#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>

static const char *USB_TAG = "USB_Manager";

class USBManager {
private:
    bool usb_connected = false;
    bool fs_mounted = false;
    uint32_t last_check_time = 0;
    float last_vbus_voltage = 0;
    
    static adc_oneshot_unit_handle_t adc_handle;
    
public:
    USBManager() {}
    
    // ADC 핸들 getter (다른 매니저가 사용할 수 있도록)
    static adc_oneshot_unit_handle_t getADCHandle() { return adc_handle; }

    bool init() {
        // GPIO 초기화
        gpio_config_t gpio_cfg = {};
        gpio_cfg.pin_bit_mask = (1ULL << VBUS_EN) | (1ULL << USB_OVER_FLG) | 
                                (1ULL << SIGNAL_LED) | (1ULL << BUZZER);
        gpio_cfg.mode = GPIO_MODE_OUTPUT;
        gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&gpio_cfg);

        // VBUS_EN 초기화 (Active High)
        gpio_set_level(VBUS_EN, 1);
        
        // USB_OVER_FLG 입력 설정
        gpio_set_direction(USB_OVER_FLG, GPIO_MODE_INPUT);
        gpio_set_pull_mode(USB_OVER_FLG, GPIO_FLOATING);

        // ADC 초기화
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
            .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

        adc_oneshot_chan_cfg_t config = {
            .atten = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, USB_VBUS_ADC_CHAN, &config));

        ESP_LOGI(USB_TAG, "USB Manager initialized (ADC shared)");
        return true;
    }

    // VBUS 전압 측정 (분압 회로 고려)
    float readVBUSVoltage() {
        int adc_raw = 0;
        adc_oneshot_read(adc_handle, USB_VBUS_ADC_CHAN, &adc_raw);
        
        // ADC 값을 실제 전압으로 변환
        // Vout = (ADC / 4095) * 3.3V
        // VBUS = Vout * (47K + 24.3K) / 24.3K
        float vout = (adc_raw / 4095.0) * ADC_REFERENCE_VOLTAGE;
        float vbus = vout * VBUS_VOLTAGE_DIVIDER;
        
        // 안정성을 위해 범위 제한
        if (vbus < 0) vbus = 0;
        if (vbus > VBUS_MAX_VOLTAGE) vbus = VBUS_MAX_VOLTAGE;
        
        return vbus;
    }

    // USB 연결 상태 확인 (VBUS + Over-current 체크)
    bool checkUSBConnection() {
        uint32_t current_time = esp_log_timestamp();
        
        if (current_time - last_check_time < USB_CHECK_INTERVAL) {
            return usb_connected;
        }
        
        last_check_time = current_time;
        last_vbus_voltage = readVBUSVoltage();
        
        // Over-current 플래그 확인 (Active High)
        bool over_current = gpio_get_level(USB_OVER_FLG) != 0;
        
        // VBUS 전압이 임계값 이상이고 over-current 아닐 때
        bool vbus_detected = last_vbus_voltage > VBUS_THRESHOLD;
        bool prev_connected = usb_connected;
        usb_connected = vbus_detected && !over_current;
        
        if (usb_connected != prev_connected) {
            if (usb_connected) {
                ESP_LOGI(USB_TAG, "USB Connected (VBUS: %.2fV)", last_vbus_voltage);
                gpio_set_level(SIGNAL_LED, 1);
            } else {
                ESP_LOGI(USB_TAG, "USB Disconnected (VBUS: %.2fV)", last_vbus_voltage);
                gpio_set_level(SIGNAL_LED, 0);
                fs_mounted = false;
            }
        }
        
        return usb_connected;
    }

    bool isConnected() {
        return usb_connected;
    }

    float getVBUSVoltage() {
        return last_vbus_voltage;
    }

    void setFSMounted(bool mounted) {
        fs_mounted = mounted;
    }

    bool isFSMounted() {
        return fs_mounted;
    }

    // 신호 LED 제어 (마운트 상태 표시)
    void setSignalLED(bool on) {
        gpio_set_level(SIGNAL_LED, on ? 1 : 0);
    }

    // 파일 읽기
    bool readFile(const char* path, char* buffer, size_t buffer_size) {
        if (!fs_mounted) {
            ESP_LOGE(USB_TAG, "File system not mounted");
            return false;
        }

        FILE *f = fopen(path, "r");
        if (f == NULL) {
            ESP_LOGE(USB_TAG, "Failed to open file: %s", path);
            return false;
        }

        size_t read_size = fread(buffer, 1, buffer_size - 1, f);
        buffer[read_size] = '\0';
        fclose(f);

        return true;
    }

    // 파일 쓰기
    bool writeFile(const char* path, const char* content, bool append = false) {
        if (!fs_mounted) {
            ESP_LOGE(USB_TAG, "File system not mounted");
            return false;
        }

        FILE *f = fopen(path, append ? "a" : "w");
        if (f == NULL) {
            ESP_LOGE(USB_TAG, "Failed to open file: %s", path);
            return false;
        }

        size_t written = fwrite(content, 1, strlen(content), f);
        fclose(f);

        return written > 0;
    }

    // CSV 헤더 초기화
    bool initCSV(const char* header) {
        if (!fs_mounted) {
            return false;
        }
        return writeFile(CSV_FILE_PATH, header, false);
    }

    // CSV에 데이터 추가
    bool appendCSV(const char* csv_line) {
        if (!fs_mounted) {
            return false;
        }
        return writeFile(CSV_FILE_PATH, csv_line, true);
    }

    void buzzNotify(int duration_ms, int count) {
        for (int i = 0; i < count; i++) {
            gpio_set_level(BUZZER, 1);
            vTaskDelay(pdMS_TO_TICKS(duration_ms));
            gpio_set_level(BUZZER, 0);
            vTaskDelay(pdMS_TO_TICKS(duration_ms));
        }
    }

    void cleanup() {
        if (adc_handle != nullptr) {
            adc_oneshot_del_unit(adc_handle);
        }
    }
};

// Static 멤버 변수 정의
adc_oneshot_unit_handle_t USBManager::adc_handle = nullptr;

#endif // USB_MANAGER_H