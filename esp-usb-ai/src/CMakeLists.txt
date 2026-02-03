# ESP-IDF component CMakeLists.txt for main component

# 소스 파일 수집
file(GLOB_RECURSE COMPONENT_SRCS "main.cpp" "*.c" "*.cpp")

# 컴포넌트 등록
idf_component_register(SRCS ${COMPONENT_SRCS}
                       INCLUDE_DIRS "." "../include"
                       REQUIRES usb usb_host_msc driver esp_adc nvs_flash vfs
                       PRIV_REQUIRES fatfs esp_driver_gpio esp_timer)