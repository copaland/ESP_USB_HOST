/*
USB 메모리 데이터 로거 (ESP32-S3, IDF 프레임워크)
# 1. 빌드
platformio run -e esp32-s3

# platformio run -e esp32-s3 --target clean;

# 2. 업로드
platformio run -e esp32-s3 -t upload

# 3. 모니터 (115200 baud)
platformio device monitor -e esp32-s3 -b 115200
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_err.h>
#include <usb/usb_host.h>
#include <usb/msc_host.h>
#include <usb/msc_host_vfs.h>
#include <esp_vfs_fat.h>

#include "config.h"
#include "USBManager.h"
#include "SensorManager.h"

static const char *APP_TAG = "USB_Data_Logger";

// 전역 객체
USBManager usb_manager;
SensorManager sensor_manager;

// Task 핸들
TaskHandle_t usb_monitor_task_handle = nullptr;
TaskHandle_t sensor_task_handle = nullptr;
TaskHandle_t data_save_task_handle = nullptr;

// 동기화
SemaphoreHandle_t usb_mount_semaphore = nullptr;

// USB 호스트 객체
usb_host_client_handle_t usb_host_client = nullptr;
msc_host_device_handle_t msc_device = nullptr;
msc_host_vfs_handle_t vfs_handle = nullptr;

// 전역 상태
volatile bool usb_mounted = false;
volatile uint32_t data_count = 0;
volatile uint8_t msc_device_address = 0;  // MSC 디바이스 주소
volatile bool msc_device_ready = false;   // MSC 디바이스 준비 완료
volatile bool msc_device_disconnected = false;  // MSC 디바이스 연결 해제

// ============= USB 호스트 콜백 =============
static void usb_lib_task(void *arg) {
    while (true) {  // 무한 루프 - 재연결 감지를 위해 계속 실행
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        // 로그 제거 - 조용히 실행
    }
    vTaskDelete(nullptr);
}

static void usb_event_cb(const usb_host_client_event_msg_t *event_msg, void *user_ctx) {
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        // USB Host가 새 디바이스를 감지하면 MSC 주소 저장
        msc_device_address = event_msg->new_dev.address;
        msc_device_ready = true;
    } else if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        // USB Host가 디바이스 제거를 감지하면 언마운트 트리거
        if (usb_mounted) {
            msc_device_disconnected = true;
        }
    }
}

// ============= USB MSC 콜백 =============
// MSC 이벤트 타입 정의 (msc_host.h의 익명 enum 복사)
enum msc_event_type {
    MSC_DEVICE_CONNECTED = 0,
    MSC_DEVICE_DISCONNECTED = 1
};

static void msc_event_cb(const msc_host_event_t *event, void *arg) {
    if (event->event == MSC_DEVICE_CONNECTED) {
        // MSC 디바이스 감지 시 항상 플래그 설정 (재연결 포함)
        msc_device_address = event->device.address;
        msc_device_ready = true;
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        if (usb_mounted && !msc_device_disconnected) {
            msc_device_disconnected = true;
        }
    }
}

// ============= USB 마운트 (Mass Storage) =============
static esp_err_t mount_usb_filesystem(uint8_t device_address) {
    // 기존 핸들이 있으면 안됨 - unmount에서 NULL로 정리되어야 함
    if (msc_device != nullptr) {
        msc_device = nullptr;
    }
    
    // MSC 디바이스 초기화
    esp_err_t err = msc_host_install_device(device_address, &msc_device);
    if (err != ESP_OK) {
        return err;
    }
    
    // VFS 마운트
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };
    
    err = msc_host_vfs_register(msc_device, USB_MOUNT_POINT, &mount_config, &vfs_handle);
    if (err == ESP_OK) {
        xSemaphoreGive(usb_mount_semaphore);
        return ESP_OK;
    } else {
        msc_host_uninstall_device(msc_device);
        msc_device = nullptr;
        return err;
    }
}

static esp_err_t unmount_usb_filesystem(bool force_cleanup = false) {
    // VFS 해제
    if (vfs_handle != nullptr) {
        msc_host_vfs_unregister(vfs_handle);
        vfs_handle = nullptr;
    }
    
    // 디바이스 핸들 정리
    if (msc_device != nullptr) {
        msc_host_uninstall_device(msc_device);
        msc_device = nullptr;
    }
    
    xSemaphoreTake(usb_mount_semaphore, 0);
    
    // USB Host 라이브러리가 디바이스 제거를 완전히 처리할 시간 확보
    vTaskDelay(pdMS_TO_TICKS(500));  // 500ms 대기
    
    return ESP_OK;
}

// ============= USB 모니터 Task =============
static void usb_monitor_task_func(void *pvParameters) {
    static bool prev_usb_status = false;
    static bool prev_fs_mounted = false;
    
    ESP_LOGI(APP_TAG, "USB Monitor Task started");
    
    while (1) {
        // USB 연결 상태 확인
        usb_manager.checkUSBConnection();
        
        bool current_usb_status = usb_manager.isConnected();
        
        // USB Host 이벤트 기반 마운트 시도
        if (msc_device_ready && !usb_mounted) {
            // USB MSC 마운트
            esp_err_t mount_result = mount_usb_filesystem(msc_device_address);
            if (mount_result == ESP_OK) {
                usb_manager.setFSMounted(true);
                usb_mounted = true;
                usb_manager.setSignalLED(true);  // 마운트 시 LED ON
                msc_device_ready = false;  // 마운트 성공 시에만 플래그 리셋
                
                // 날짜/시간과 함께 마운트 메시지 출력
                char date_buf[16], time_buf[16];
                sensor_manager.getDateTimeStrings(date_buf, sizeof(date_buf), time_buf, sizeof(time_buf));
                ESP_LOGI(APP_TAG, "%s %s - USB Mounted\n", date_buf, time_buf);
                
                // CSV 파일 존재 여부 확인
                FILE* test_file = fopen(CSV_FILE_PATH, "r");
                bool file_exists = (test_file != nullptr);
                if (file_exists) {
                    fclose(test_file);
                } else {
                    // CSV 헤더 초기화 (파일이 없을 때만)
                    const char* csv_header = "Date,Time,VBUS(V),Level,Status\n";
                    if (!usb_manager.initCSV(csv_header)) {
                        ESP_LOGE(APP_TAG, "Failed to initialize CSV");
                    }
                }
                
                usb_manager.buzzNotify(200, 2);  // 2회 신호음
            } else {
                // 마운트 실패 시 플래그는 유지하여 재시도 가능하도록 함 (단, 3초 후 재시도)
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
        }
        
        // MSC 디바이스 연결 해제 감지 (USB Host 모드)
        if (msc_device_disconnected && usb_mounted) {
            // 날짜/시간과 함께 언마운트 메시지 출력
            char date_buf[16], time_buf[16];
            sensor_manager.getDateTimeStrings(date_buf, sizeof(date_buf), time_buf, sizeof(time_buf));
            ESP_LOGI(APP_TAG, "%s %s - USB Unmounted\n", date_buf, time_buf);
            
            unmount_usb_filesystem(true);  // force_cleanup=true (디바이스 이미 제거됨)
            usb_manager.setFSMounted(false);
            usb_mounted = false;
            usb_manager.setSignalLED(false);  // 언마운트 시 LED OFF
            data_count = 0;  // 데이터 카운트 리셋
            
            // 재연결 감지를 위한 플래그 리셋
            msc_device_disconnected = false;
            msc_device_ready = false;
            msc_device_address = 0;
            
            usb_manager.buzzNotify(500, 1);  // 1회 긴 신호음
            
            // USB Host가 새 디바이스를 감지할 시간 확보 (추가 대기)
            vTaskDelay(pdMS_TO_TICKS(1000));  // 1초 대기
        }
        
        prev_usb_status = current_usb_status;
        
        vTaskDelay(pdMS_TO_TICKS(USB_CHECK_INTERVAL));
    }
}

// ============= 센서 Task =============
static void sensor_task_func(void *pvParameters) {
    ESP_LOGI(APP_TAG, "Sensor Task started");
    
    while (1) {
        // USB 호스트 모드에서는 항상 센서 읽기
        float level = sensor_manager.readLevelSensor();
        sensor_manager.updateStatusLED(level);
        
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
    }
}

// ============= 데이터 저장 Task =============
static void data_save_task_func(void *pvParameters) {
    ESP_LOGI(APP_TAG, "Data Save Task started");
    
    uint32_t last_save_time = xTaskGetTickCount();
    
    while (1) {
        uint32_t current_time = xTaskGetTickCount();
        
        // USB 호스트 모드에서는 isFSMounted()만 체크
        if (usb_manager.isFSMounted()) {
            if ((current_time - last_save_time) >= pdMS_TO_TICKS(DATA_SAVE_INTERVAL)) {
                last_save_time = current_time;
                
                // 센서 데이터 수집
                SensorManager::SensorData data;
                sensor_manager.collectData(data, usb_manager.getVBUSVoltage(), 
                                          usb_manager.isFSMounted());
                
                // CSV 형식으로 변환
                char csv_line[256];
                sensor_manager.formatAsCSV(data, csv_line, sizeof(csv_line));
                
                // csv_line에서 끝의 개행문자 제거 (로그 출력용)
                size_t len = strlen(csv_line);
                if (len > 0 && csv_line[len-1] == '\n') {
                    csv_line[len-1] = '\0';
                }
                
                // 저장할 데이터 미리 로그 출력
                ESP_LOGI(APP_TAG, "Saving to USB: %s", csv_line);
                
                // 개행문자 복원 (파일 저장용)
                csv_line[len-1] = '\n';
                
                // 파일에 저장
                if (usb_manager.appendCSV(csv_line)) {
                    data_count++;  // volatile 변수 증가
                    ESP_LOGI(APP_TAG, "✓ Data #%lu saved successfully to %s\n", (unsigned long)data_count, "data.csv");
                } else {
                    ESP_LOGE(APP_TAG, "✗ Failed to save data to %s\n", "data.csv");
                    usb_manager.buzzNotify(100, 3);  // 3회 단신호음
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============= USB 호스트 초기화 =============
static esp_err_t init_usb_host() {
    // USB 호스트 라이브러리 초기화
    const usb_host_config_t usb_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };
    
    ESP_ERROR_CHECK(usb_host_install(&usb_config));
    
    // USB 호스트 클라이언트 생성
    const usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usb_event_cb,
            .callback_arg = nullptr,
        },
    };
    
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &usb_host_client));
    
    // MSC Host 초기화
    const msc_host_driver_config_t msc_config = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = tskNO_AFFINITY,
        .callback = msc_event_cb,
        .callback_arg = nullptr,
    };
    
    ESP_ERROR_CHECK(msc_host_install(&msc_config));
    
    // USB 호스트 라이브러리 작업 Task 생성
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib_task", 4096, 
                            nullptr, 5, nullptr, 0);
    
    return ESP_OK;
}

// ============= Main =============
static void app_main_impl(void) {
    ESP_LOGI(APP_TAG, "\n========================================");
    ESP_LOGI(APP_TAG, "  ESP32-S3 USB Memory Data Logger");
    ESP_LOGI(APP_TAG, "  (IDF Framework)");
    ESP_LOGI(APP_TAG, "========================================\n");

    // Semaphore 생성
    usb_mount_semaphore = xSemaphoreCreateBinary();

    // USB Manager 초기화
    if (!usb_manager.init()) {
        ESP_LOGE(APP_TAG, "USB Manager initialization failed");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // Sensor Manager 초기화 (USBManager의 ADC 핸들 공유)
    if (!sensor_manager.init(USBManager::getADCHandle())) {
        ESP_LOGE(APP_TAG, "Sensor Manager initialization failed");
        return;
    }
    
    // RTC 시간을 현재 날짜로 업데이트 (필요시 주석 해제)
    // sensor_manager.setRTCTime(2026, 2, 3, 14, 30, 0);  // 2026-02-03 14:30:00
    
    vTaskDelay(pdMS_TO_TICKS(500));

    // USB 호스트 초기화
    if (init_usb_host() != ESP_OK) {
        ESP_LOGE(APP_TAG, "USB Host initialization failed");
        return;
    }

    // Task 생성
    xTaskCreatePinnedToCore(usb_monitor_task_func, "usb_monitor", STACK_SIZE, 
                           nullptr, USB_TASK_PRIORITY, &usb_monitor_task_handle, 1);
    
    xTaskCreatePinnedToCore(sensor_task_func, "sensor", STACK_SIZE, 
                           nullptr, SENSOR_TASK_PRIORITY, &sensor_task_handle, 1);
    
    xTaskCreatePinnedToCore(data_save_task_func, "data_save", STACK_SIZE, 
                           nullptr, SAVE_TASK_PRIORITY, &data_save_task_handle, 1);

    ESP_LOGI(APP_TAG, "All tasks created");
    ESP_LOGI(APP_TAG, "Waiting for USB memory connection...\n");

    // 메인 Task는 유휴 상태 유지
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// app_main을 C 링키지로 export
extern "C" void app_main(void) {
    app_main_impl();
}