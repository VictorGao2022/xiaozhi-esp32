// 休眠状态下跌落？？

#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "backlight.h"
#include "emote_display.h"

#include <wifi_station.h>
#include <esp_log.h>

#include <driver/i2c_master.h>
#include <driver/i2c.h>
#include "i2c_device.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_st77916.h>
#include "esp_lcd_touch_cst816s.h"
#include "touch.h"

#include "driver/temperature_sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// BMI270相关头文件和定义
#include "bmi270.h"
#include "bmi2_defs.h"

// 宏定义与常量
#define TAG "EchoEar"
#define BMI270_TAG "BMI270"
#define USE_LVGL_DEFAULT    0

// BMI270配置
#define BMI270_I2C_ADDR 0x68  // SA0接地
#define BMI270_SDA_GPIO 1     // ICM_SDA复用GPIO1
#define BMI270_SCL_GPIO 2     // ICM_SCL复用GPIO2

// 运动检测参数（使用constexpr提高编译期计算效率）
constexpr float SHAKE_THRESHOLD = 15.0f;       // 摇晃加速度阈值(g)
constexpr float FALL_THRESHOLD = 0.3f;         // 跌落检测阈值(g)
constexpr uint32_t FALL_DURATION = 500;        // 跌落持续时间(ms)
constexpr uint32_t SHAKE_DURATION = 1000;      // 摇晃检测时间窗口(ms)
constexpr uint8_t SHAKE_COUNT_THRESHOLD = 3;   // 检测窗口内摇晃次数阈值

// 字体声明
LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

// 全局设备句柄与变量
static temperature_sensor_handle_t temp_sensor = NULL;
static float tsens_value;

// 硬件引脚配置（使用constexpr确保不可修改）
constexpr gpio_num_t AUDIO_I2S_GPIO_DIN_DEFAULT = AUDIO_I2S_GPIO_DIN_1;
constexpr gpio_num_t AUDIO_CODEC_PA_PIN_DEFAULT = AUDIO_CODEC_PA_PIN_1;
constexpr gpio_num_t QSPI_PIN_NUM_LCD_RST_DEFAULT = QSPI_PIN_NUM_LCD_RST_1;
constexpr gpio_num_t TOUCH_PAD2_DEFAULT = TOUCH_PAD2_1;
constexpr gpio_num_t UART1_TX_DEFAULT = UART1_TX_1;
constexpr gpio_num_t UART1_RX_DEFAULT = UART1_RX_1;

// 动态引脚变量
static gpio_num_t AUDIO_I2S_GPIO_DIN = AUDIO_I2S_GPIO_DIN_DEFAULT;
static gpio_num_t AUDIO_CODEC_PA_PIN = AUDIO_CODEC_PA_PIN_DEFAULT;
static gpio_num_t QSPI_PIN_NUM_LCD_RST = QSPI_PIN_NUM_LCD_RST_DEFAULT;
static gpio_num_t TOUCH_PAD2 = TOUCH_PAD2_DEFAULT;
static gpio_num_t UART1_TX = UART1_TX_DEFAULT;
static gpio_num_t UART1_RX = UART1_RX_DEFAULT;

// LCD初始化命令（使用constexpr数组）
constexpr st77916_lcd_init_cmd_t vendor_specific_init_yysj[] = {
    {0xF0, (uint8_t []){0x28}, 1, 0},
    {0xF2, (uint8_t []){0x28}, 1, 0},
    {0x73, (uint8_t []){0xF0}, 1, 0},
    {0x7C, (uint8_t []){0xD1}, 1, 0},
    {0x83, (uint8_t []){0xE0}, 1, 0},
    {0x84, (uint8_t []){0x61}, 1, 0},
    {0xF2, (uint8_t []){0x82}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x01}, 1, 0},
    {0xF1, (uint8_t []){0x01}, 1, 0},
    {0xB0, (uint8_t []){0x56}, 1, 0},
    {0xB1, (uint8_t []){0x4D}, 1, 0},
    {0xB2, (uint8_t []){0x24}, 1, 0},
    {0xB4, (uint8_t []){0x87}, 1, 0},
    {0xB5, (uint8_t []){0x44}, 1, 0},
    {0xB6, (uint8_t []){0x8B}, 1, 0},
    {0xB7, (uint8_t []){0x40}, 1, 0},
    {0xB8, (uint8_t []){0x86}, 1, 0},
    {0xBA, (uint8_t []){0x00}, 1, 0},
    {0xBB, (uint8_t []){0x08}, 1, 0},
    {0xBC, (uint8_t []){0x08}, 1, 0},
    {0xBD, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x80}, 1, 0},
    {0xC1, (uint8_t []){0x10}, 1, 0},
    {0xC2, (uint8_t []){0x37}, 1, 0},
    {0xC3, (uint8_t []){0x80}, 1, 0},
    {0xC4, (uint8_t []){0x10}, 1, 0},
    {0xC5, (uint8_t []){0x37}, 1, 0},
    {0xC6, (uint8_t []){0xA9}, 1, 0},
    {0xC7, (uint8_t []){0x41}, 1, 0},
    {0xC8, (uint8_t []){0x01}, 1, 0},
    {0xC9, (uint8_t []){0xA9}, 1, 0},
    {0xCA, (uint8_t []){0x41}, 1, 0},
    {0xCB, (uint8_t []){0x01}, 1, 0},
    {0xD0, (uint8_t []){0x91}, 1, 0},
    {0xD1, (uint8_t []){0x68}, 1, 0},
    {0xD2, (uint8_t []){0x68}, 1, 0},
    {0xF5, (uint8_t []){0x00, 0xA5}, 2, 0},
    {0xDD, (uint8_t []){0x4F}, 1, 0},
    {0xDE, (uint8_t []){0x4F}, 1, 0},
    {0xF1, (uint8_t []){0x10}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x02}, 1, 0},
    {0xE0, (uint8_t []){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
    {0xE1, (uint8_t []){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
    {0xF0, (uint8_t []){0x10}, 1, 0},
    {0xF3, (uint8_t []){0x10}, 1, 0},
    {0xE0, (uint8_t []){0x07}, 1, 0},
    {0xE1, (uint8_t []){0x00}, 1, 0},
    {0xE2, (uint8_t []){0x00}, 1, 0},
    {0xE3, (uint8_t []){0x00}, 1, 0},
    {0xE4, (uint8_t []){0xE0}, 1, 0},
    {0xE5, (uint8_t []){0x06}, 1, 0},
    {0xE6, (uint8_t []){0x21}, 1, 0},
    {0xE7, (uint8_t []){0x01}, 1, 0},
    {0xE8, (uint8_t []){0x05}, 1, 0},
    {0xE9, (uint8_t []){0x02}, 1, 0},
    {0xEA, (uint8_t []){0xDA}, 1, 0},
    {0xEB, (uint8_t []){0x00}, 1, 0},
    {0xEC, (uint8_t []){0x00}, 1, 0},
    {0xED, (uint8_t []){0x0F}, 1, 0},
    {0xEE, (uint8_t []){0x00}, 1, 0},
    {0xEF, (uint8_t []){0x00}, 1, 0},
    {0xF8, (uint8_t []){0x00}, 1, 0},
    {0xF9, (uint8_t []){0x00}, 1, 0},
    {0xFA, (uint8_t []){0x00}, 1, 0},
    {0xFB, (uint8_t []){0x00}, 1, 0},
    {0xFC, (uint8_t []){0x00}, 1, 0},
    {0xFD, (uint8_t []){0x00}, 1, 0},
    {0xFE, (uint8_t []){0x00}, 1, 0},
    {0xFF, (uint8_t []){0x00}, 1, 0},
    {0x60, (uint8_t []){0x40}, 1, 0},
    {0x61, (uint8_t []){0x04}, 1, 0},
    {0x62, (uint8_t []){0x00}, 1, 0},
    {0x63, (uint8_t []){0x42}, 1, 0},
    {0x64, (uint8_t []){0xD9}, 1, 0},
    {0x65, (uint8_t []){0x00}, 1, 0},
    {0x66, (uint8_t []){0x00}, 1, 0},
    {0x67, (uint8_t []){0x00}, 1, 0},
    {0x68, (uint8_t []){0x00}, 1, 0},
    {0x69, (uint8_t []){0x00}, 1, 0},
    {0x6A, (uint8_t []){0x00}, 1, 0},
    {0x6B, (uint8_t []){0x00}, 1, 0},
    {0x70, (uint8_t []){0x40}, 1, 0},
    {0x71, (uint8_t []){0x03}, 1, 0},
    {0x72, (uint8_t []){0x00}, 1, 0},
    {0x73, (uint8_t []){0x42}, 1, 0},
    {0x74, (uint8_t []){0xD8}, 1, 0},
    {0x75, (uint8_t []){0x00}, 1, 0},
    {0x76, (uint8_t []){0x00}, 1, 0},
    {0x77, (uint8_t []){0x00}, 1, 0},
    {0x78, (uint8_t []){0x00}, 1, 0},
    {0x79, (uint8_t []){0x00}, 1, 0},
    {0x7A, (uint8_t []){0x00}, 1, 0},
    {0x7B, (uint8_t []){0x00}, 1, 0},
    {0x80, (uint8_t []){0x48}, 1, 0},
    {0x81, (uint8_t []){0x00}, 1, 0},
    {0x82, (uint8_t []){0x06}, 1, 0},
    {0x83, (uint8_t []){0x02}, 1, 0},
    {0x84, (uint8_t []){0xD6}, 1, 0},
    {0x85, (uint8_t []){0x04}, 1, 0},
    {0x86, (uint8_t []){0x00}, 1, 0},
    {0x87, (uint8_t []){0x00}, 1, 0},
    {0x88, (uint8_t []){0x48}, 1, 0},
    {0x89, (uint8_t []){0x00}, 1, 0},
    {0x8A, (uint8_t []){0x08}, 1, 0},
    {0x8B, (uint8_t []){0x02}, 1, 0},
    {0x8C, (uint8_t []){0xD8}, 1, 0},
    {0x8D, (uint8_t []){0x04}, 1, 0},
    {0x8E, (uint8_t []){0x00}, 1, 0},
    {0x8F, (uint8_t []){0x00}, 1, 0},
    {0x90, (uint8_t []){0x48}, 1, 0},
    {0x91, (uint8_t []){0x00}, 1, 0},
    {0x92, (uint8_t []){0x0A}, 1, 0},
    {0x93, (uint8_t []){0x02}, 1, 0},
    {0x94, (uint8_t []){0xDA}, 1, 0},
    {0x95, (uint8_t []){0x04}, 1, 0},
    {0x96, (uint8_t []){0x00}, 1, 0},
    {0x97, (uint8_t []){0x00}, 1, 0},
    {0x98, (uint8_t []){0x48}, 1, 0},
    {0x99, (uint8_t []){0x00}, 1, 0},
    {0x9A, (uint8_t []){0x0C}, 1, 0},
    {0x9B, (uint8_t []){0x02}, 1, 0},
    {0x9C, (uint8_t []){0xDC}, 1, 0},
    {0x9D, (uint8_t []){0x04}, 1, 0},
    {0x9E, (uint8_t []){0x00}, 1, 0},
    {0x9F, (uint8_t []){0x00}, 1, 0},
    {0xA0, (uint8_t []){0x48}, 1, 0},
    {0xA1, (uint8_t []){0x00}, 1, 0},
    {0xA2, (uint8_t []){0x05}, 1, 0},
    {0xA3, (uint8_t []){0x02}, 1, 0},
    {0xA4, (uint8_t []){0xD5}, 1, 0},
    {0xA5, (uint8_t []){0x04}, 1, 0},
    {0xA6, (uint8_t []){0x00}, 1, 0},
    {0xA7, (uint8_t []){0x00}, 1, 0},
    {0xA8, (uint8_t []){0x48}, 1, 0},
    {0xA9, (uint8_t []){0x00}, 1, 0},
    {0xAA, (uint8_t []){0x07}, 1, 0},
    {0xAB, (uint8_t []){0x02}, 1, 0},
    {0xAC, (uint8_t []){0xD7}, 1, 0},
    {0xAD, (uint8_t []){0x04}, 1, 0},
    {0xAE, (uint8_t []){0x00}, 1, 0},
    {0xAF, (uint8_t []){0x00}, 1, 0},
    {0xB0, (uint8_t []){0x48}, 1, 0},
    {0xB1, (uint8_t []){0x00}, 1, 0},
    {0xB2, (uint8_t []){0x09}, 1, 0},
    {0xB3, (uint8_t []){0x02}, 1, 0},
    {0xB4, (uint8_t []){0xD9}, 1, 0},
    {0xB5, (uint8_t []){0x04}, 1, 0},
    {0xB6, (uint8_t []){0x00}, 1, 0},
    {0xB7, (uint8_t []){0x00}, 1, 0},
    {0xB8, (uint8_t []){0x48}, 1, 0},
    {0xB9, (uint8_t []){0x00}, 1, 0},
    {0xBA, (uint8_t []){0x0B}, 1, 0},
    {0xBB, (uint8_t []){0x02}, 1, 0},
    {0xBC, (uint8_t []){0xDB}, 1, 0},
    {0xBD, (uint8_t []){0x04}, 1, 0},
    {0xBE, (uint8_t []){0x00}, 1, 0},
    {0xBF, (uint8_t []){0x00}, 1, 0},
    {0xC0, (uint8_t []){0x10}, 1, 0},
    {0xC1, (uint8_t []){0x47}, 1, 0},
    {0xC2, (uint8_t []){0x56}, 1, 0},
    {0xC3, (uint8_t []){0x65}, 1, 0},
    {0xC4, (uint8_t []){0x74}, 1, 0},
    {0xC5, (uint8_t []){0x88}, 1, 0},
    {0xC6, (uint8_t []){0x99}, 1, 0},
    {0xC7, (uint8_t []){0x01}, 1, 0},
    {0xC8, (uint8_t []){0xBB}, 1, 0},
    {0xC9, (uint8_t []){0xAA}, 1, 0},
    {0xD0, (uint8_t []){0x10}, 1, 0},
    {0xD1, (uint8_t []){0x47}, 1, 0},
    {0xD2, (uint8_t []){0x56}, 1, 0},
    {0xD3, (uint8_t []){0x65}, 1, 0},
    {0xD4, (uint8_t []){0x74}, 1, 0},
    {0xD5, (uint8_t []){0x88}, 1, 0},
    {0xD6, (uint8_t []){0x99}, 1, 0},
    {0xD7, (uint8_t []){0x01}, 1, 0},
    {0xD8, (uint8_t []){0xBB}, 1, 0},
    {0xD9, (uint8_t []){0xAA}, 1, 0},
    {0xF3, (uint8_t []){0x01}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0x21, (uint8_t []){}, 0, 0},
    {0x11, (uint8_t []){}, 0, 0},
    {0x00, (uint8_t []){}, 0, 120},
};

// BMI270设备和数据结构（使用std::unique_ptr管理）
static struct bmi2_dev bmi270_dev;
static struct bmi2_sens_data bmi2_sensor_data;
static SemaphoreHandle_t bmi270_mutex = nullptr;
static bool is_falling = false;
static int shake_count = 0;
static TickType_t last_shake_time = 0;

// BMI270 I2C读写函数（优化设备添加/移除逻辑）
static int8_t bmi270_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    if (!intf_ptr || !data) return BMI2_E_NULL_PTR;
    
    i2c_master_bus_handle_t i2c_bus = static_cast<i2c_master_bus_handle_t>(intf_ptr);
    i2c_master_dev_handle_t dev_handle = nullptr;
    
    i2c_master_dev_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI270_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(BMI270_TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return BMI2_E_COMM_FAIL;
    }
    
    ret = i2c_master_transmit(dev_handle, &reg_addr, 1, data, len, pdMS_TO_TICKS(100));
    i2c_master_bus_remove_device(dev_handle);
    
    return (ret == ESP_OK) ? BMI2_OK : BMI2_E_COMM_FAIL;
}

static int8_t bmi270_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    if (!intf_ptr || !data) return BMI2_E_NULL_PTR;
    
    i2c_master_bus_handle_t i2c_bus = static_cast<i2c_master_bus_handle_t>(intf_ptr);
    i2c_master_dev_handle_t dev_handle = nullptr;
    
    i2c_master_dev_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI270_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(BMI270_TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return BMI2_E_COMM_FAIL;
    }
    
    ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, pdMS_TO_TICKS(100));
    i2c_master_bus_remove_device(dev_handle);
    
    return (ret == ESP_OK) ? BMI2_OK : BMI2_E_COMM_FAIL;
}

// BMI270初始化函数（增加错误处理和配置检查）
static int8_t bmi270_init(i2c_master_bus_handle_t i2c_bus) {
    if (!i2c_bus) {
        ESP_LOGE(BMI270_TAG, "Invalid I2C bus handle");
        return BMI2_E_NULL_PTR;
    }

    int8_t rslt;
    
    // 初始化BMI270设备结构体
    bmi270_dev.intf = BMI2_I2C_INTF;
    bmi270_dev.read = bmi270_i2c_read;
    bmi270_dev.write = bmi270_i2c_write;
    bmi270_dev.intf_ptr = i2c_bus;
    bmi270_dev.delay_us = [](uint32_t period, void *intf_ptr) {
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    };
    bmi270_dev.config_file_ptr = nullptr;
    
    // 初始化传感器
    rslt = bmi270_init(&bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(BMI270_TAG, "Failed to initialize BMI270: %d", rslt);
        return rslt;
    }
    
    // 配置加速度计
    struct bmi2_sens_config config;
    config.type = BMI2_ACCEL;
    config.acc.odr = BMI2_ACC_ODR_100HZ;
    config.acc.range = BMI2_ACC_RANGE_2G;
    config.acc.bw = BMI2_ACC_BW_NORMAL;
    
    rslt = bmi270_set_sensor_config(&config, 1, &bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(BMI270_TAG, "Failed to configure accelerometer: %d", rslt);
        return rslt;
    }
    
    // 启用加速度计
    rslt = bmi270_sensor_enable(BMI2_ACCEL, &bmi270_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(BMI270_TAG, "Failed to enable accelerometer: %d", rslt);
        return rslt;
    }
    
    ESP_LOGI(BMI270_TAG, "BMI270 initialized successfully");
    return BMI2_OK;
}

// 运动检测任务（优化逻辑和资源锁定）
static void motion_detection_task(void *arg) {
    auto& app = Application::GetInstance();
    auto& board = static_cast<EspS3Cat&>(Board::GetInstance());
    TickType_t last_fall_time = 0;
    
    if (!bmi270_mutex) {
        ESP_LOGE(BMI270_TAG, "BMI270 mutex not initialized");
        vTaskDelete(nullptr);
        return;
    }
    
    while (true) {
        if (xSemaphoreTake(bmi270_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // 读取加速度数据
            int8_t rslt = bmi270_get_sensor_data(&bmi2_sensor_data, &bmi270_dev);
            xSemaphoreGive(bmi270_mutex);
            
            if (rslt == BMI2_OK) {
                // 计算加速度大小 (转换为g)
                float accel_x = bmi2_sensor_data.acc.x / 1024.0f;
                float accel_y = bmi2_sensor_data.acc.y / 1024.0f;
                float accel_z = bmi2_sensor_data.acc.z / 1024.0f;
                float accel_mag = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
                
                // 跌落检测
                if (accel_mag < FALL_THRESHOLD) {
                    if (!is_falling) {
                        last_fall_time = xTaskGetTickCount();
                        is_falling = true;
                    } else if (xTaskGetTickCount() - last_fall_time > pdMS_TO_TICKS(FALL_DURATION)) {
                        ESP_LOGI(BMI270_TAG, "Fall detected! Acceleration: %.2fg", accel_mag);
                        
                        // 发送消息给大模型
                        app.SendUserMessage("你被摔倒在地");
                        // 显示相应表情
                        if (board.GetDisplay()) {
                            board.GetDisplay()->SetEmotion("shocked");
                        }
                        
                        is_falling = false;
                        vTaskDelay(pdMS_TO_TICKS(2000)); // 防止重复检测
                    }
                } else {
                    is_falling = false;
                }
                
                // 摇晃检测
                if (accel_mag > SHAKE_THRESHOLD) {
                    TickType_t current_time = xTaskGetTickCount();
                    
                    // 检查是否在时间窗口内
                    if (current_time - last_shake_time < pdMS_TO_TICKS(SHAKE_DURATION)) {
                        shake_count++;
                        
                        // 达到摇晃次数阈值
                        if (shake_count >= SHAKE_COUNT_THRESHOLD) {
                            ESP_LOGI(BMI270_TAG, "Shake detected! Acceleration: %.2fg", accel_mag);
                            
                            // 发送消息给大模型
                            app.SendUserMessage("你被狠狠地摇晃了");
                            // 显示相应表情
                            if (board.GetDisplay()) {
                                board.GetDisplay()->SetEmotion("confused");
                            }
                            
                            shake_count = 0;
                            vTaskDelay(pdMS_TO_TICKS(2000)); // 防止重复检测
                        }
                    } else {
                        // 重置时间窗口和计数
                        last_shake_time = current_time;
                        shake_count = 1;
                    }
                }
            } else {
                ESP_LOGE(BMI270_TAG, "Failed to read sensor data: %d", rslt);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz采样率
    }
}

// 充电管理类（使用智能指针和RAII）
class Charge : public I2cDevice {
public:
    Charge(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        read_buffer_ = new uint8_t[8];
    }
    
    ~Charge() override {
        delete[] read_buffer_;
    }
    
    void Printcharge() {
        ReadRegs(0x08, read_buffer_, 2);
        ReadRegs(0x0c, read_buffer_ + 2, 2);
        
        if (temp_sensor) {
            ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));
        }

        int16_t voltage = static_cast<uint16_t>(read_buffer_[1] << 8 | read_buffer_[0]);
        int16_t current = static_cast<int16_t>(read_buffer_[3] << 8 | read_buffer_[2]);
        
        // 可在此处添加电压电流处理逻辑
        (void)voltage;
        (void)current;
    }
    
    static void TaskFunction(void *pvParameters) {
        if (!pvParameters) {
            ESP_LOGE(TAG, "Invalid parameter for Charge task");
            vTaskDelete(nullptr);
            return;
        }
        
        Charge* charge = static_cast<Charge*>(pvParameters);
        while (true) {
            charge->Printcharge();
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

private:
    uint8_t* read_buffer_ = nullptr;
};

// 触摸管理类（优化事件处理和资源管理）
class Cst816s : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };

    enum TouchEvent {
        TOUCH_NONE,
        TOUCH_PRESS,
        TOUCH_RELEASE,
        TOUCH_HOLD
    };

    Cst816s(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        read_buffer_ = new uint8_t[6];
        was_touched_ = false;
        press_count_ = 0;

        // 创建触摸中断信号量
        touch_isr_mux_ = xSemaphoreCreateBinary();
        if (!touch_isr_mux_) {
            ESP_LOGE("EchoEar", "Failed to create touch semaphore");
        }
    }

    ~Cst816s() override {
        delete[] read_buffer_;
        if (touch_isr_mux_) {
            vSemaphoreDelete(touch_isr_mux_);
        }
    }

    void NotifyTouchEvent() {
        if (touch_isr_mux_) {
            xSemaphoreGiveFromISR(touch_isr_mux_, NULL);
        }
    }

    bool WaitForTouchEvent(TickType_t timeout = portMAX_DELAY) {
        return touch_isr_mux_ && xSemaphoreTake(touch_isr_mux_, timeout) == pdTRUE;
    }

    void UpdateTouchPoint() {
        ReadRegs(0x02, read_buffer_, 6);
        
        point_.num = read_buffer_[0] & 0x0F;
        if (point_.num == 0) {
            point_.x = -1;
            point_.y = -1;
            return;
        }

        point_.x = ((read_buffer_[2] & 0x0F) << 8) | read_buffer_[3];
        point_.y = ((read_buffer_[4] & 0x0F) << 8) | read_buffer_[5];
    }

    TouchEvent CheckTouchEvent() {
        bool is_touched = (point_.num > 0);
        TouchEvent event = TOUCH_NONE;

        if (is_touched) {
            if (!was_touched_) {
                event = TOUCH_PRESS;
                press_count_ = 0;
            } else {
                press_count_++;
                event = (press_count_ > 20) ? TOUCH_HOLD : TOUCH_NONE;
            }
        } else if (was_touched_) {
            event = TOUCH_RELEASE;
        }

        was_touched_ = is_touched;
        return event;
    }

    TouchPoint_t GetTouchPoint() const {
        return point_;
    }

private:
    uint8_t* read_buffer_;
    TouchPoint_t point_;
    SemaphoreHandle_t touch_isr_mux_;
    bool was_touched_;
    int press_count_;
};

// 主板类（优化初始化流程和资源管理）
class EspS3Cat : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    i2c_master_bus_handle_t bmi270_i2c_bus_ = nullptr;
    std::unique_ptr<Cst816s> cst816s_;
    std::unique_ptr<Charge> charge_;
    Button boot_button_;
#if USE_LVGL_DEFAULT
    std::unique_ptr<LcdDisplay> display_;
#else
    std::unique_ptr<anim::EmoteDisplay> display_;
#endif
    std::unique_ptr<PwmBacklight> backlight_;
    esp_lcd_touch_handle_t tp = nullptr;   // LCD触摸句柄

    // 初始化I2C总线
    void InitializeI2c() {
        // 初始化主I2C总线
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // 初始化BMI270专用I2C总线 (GPIO1和GPIO2)
        i2c_bus_cfg.i2c_port = I2C_NUM_1;  // 使用不同的I2C端口
        i2c_bus_cfg.sda_io_num = BMI270_SDA_GPIO;
        i2c_bus_cfg.scl_io_num = BMI270_SCL_GPIO;
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &bmi270_i2c_bus_));

        // 初始化温度传感器
        temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

        // 初始化BMI270传感器和互斥锁
        bmi270_mutex = xSemaphoreCreateMutex();
        if (bmi270_mutex && bmi270_init(bmi270_i2c_bus_) == BMI2_OK) {
            // 创建运动检测任务
            xTaskCreatePinnedToCore(motion_detection_task, "motion_detection", 4096, 
                                   nullptr, 5, nullptr, 1);
        } else {
            ESP_LOGE(BMI270_TAG, "Failed to initialize BMI270, motion detection disabled");
            if (bmi270_mutex) {
                vSemaphoreDelete(bmi270_mutex);
                bmi270_mutex = nullptr;
            }
        }
    }

    // 检测PCB版本
    uint8_t DetectPcbVersion() {
        esp_err_t ret = i2c_master_probe(i2c_bus_, 0x18, 100);
        uint8_t pcb_version = 0;
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "PCB version V1.0");
            pcb_version = 0;
        } else {
            // 配置GPIO48为输出
            gpio_config_t gpio_conf = {
                .pin_bit_mask = (1ULL << GPIO_NUM_48),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            ESP_ERROR_CHECK(gpio_config(&gpio_conf));
            ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_48, 1));
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // 再次探测
            ret = i2c_master_probe(i2c_bus_, 0x18, 100);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "PCB version V1.2");
                pcb_version = 1;
                // 更新引脚配置
                AUDIO_I2S_GPIO_DIN = AUDIO_I2S_GPIO_DIN_2;
                AUDIO_CODEC_PA_PIN = AUDIO_CODEC_PA_PIN_2;
                QSPI_PIN_NUM_LCD_RST = QSPI_PIN_NUM_LCD_RST_2;
                TOUCH_PAD2 = TOUCH_PAD2_2;
                UART1_TX = UART1_TX_2;
                UART1_RX = UART1_RX_2;
            } else {
                ESP_LOGE(TAG, "PCB version detection error");
            }
        }
        return pcb_version;
    }

    // 触摸中断回调
    static void touch_isr_callback(void* arg) {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (touchpad) {
            touchpad->NotifyTouchEvent();
        }
    }

    // 触摸事件处理任务
    static void touch_event_task(void* arg) {
        Cst816s* touchpad = static_cast<Cst816s*>(arg);
        if (!touchpad) {
            ESP_LOGE(TAG, "Invalid touchpad pointer in touch_event_task");
            vTaskDelete(nullptr);
            return;
        }

        while (true) {
            if (touchpad->WaitForTouchEvent()) {
                auto &app = Application::GetInstance();
                auto &board = static_cast<EspS3Cat&>(Board::GetInstance());

                ESP_LOGI(TAG, "Touch event, TP_PIN_NUM_INT: %d", gpio_get_level(TP_PIN_NUM_INT));
                touchpad->UpdateTouchPoint();
                auto touch_event = touchpad->CheckTouchEvent();

                if (touch_event == Cst816s::TOUCH_RELEASE) {
                    if (app.GetDeviceState() == kDeviceStateStarting &&
                        !WifiStation::GetInstance().IsConnected()) {
                        board.ResetWifiConfiguration();
                    } else {
                        app.ToggleChatState();
                    }
                }
            }
        }
    }

    // 初始化充电管理
    void InitializeCharge() {
        charge_ = std::make_unique<Charge>(i2c_bus_, 0x55);
        xTaskCreatePinnedToCore(Charge::TaskFunction, "batterydecTask", 3 * 1024, 
                               charge_.get(), 6, nullptr, 0);
    }

    // 初始化触摸板
    void InitializeCst816sTouchPad() {
        cst816s_ = std::make_unique<Cst816s>(i2c_bus_, 0x15);

        // 创建触摸任务
        xTaskCreatePinnedToCore(touch_event_task, "touch_task", 4 * 1024, 
                               cst816s_.get(), 5, nullptr, 1);

        // 配置触摸中断引脚
        const gpio_config_t int_gpio_config = {
            .pin_bit_mask = (1ULL << TP_PIN_NUM_INT),
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_ANYEDGE
        };
        ESP_ERROR_CHECK(gpio_config(&int_gpio_config));
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        ESP_ERROR_CHECK(gpio_intr_enable(TP_PIN_NUM_INT));
        ESP_ERROR_CHECK(gpio_isr_handler_add(TP_PIN_NUM_INT, 
                                           EspS3Cat::touch_isr_callback, 
                                           cst816s_.get()));
    }

    // 初始化SPI总线
    void InitializeSpi() {
        const spi_bus_config_t bus_config = TAIJIPI_ST77916_PANEL_BUS_QSPI_CONFIG(
            QSPI_PIN_NUM_LCD_PCLK,
            QSPI_PIN_NUM_LCD_DATA0,
            QSPI_PIN_NUM_LCD_DATA1,
            QSPI_PIN_NUM_LCD_DATA2,
            QSPI_PIN_NUM_LCD_DATA3,
            QSPI_LCD_H_RES * 80 * sizeof(uint16_t)
        );
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    // 初始化显示屏
    void Initializest77916Display(uint8_t pcb_version) {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        const esp_lcd_panel_io_spi_config_t io_config = ST77916_PANEL_IO_QSPI_CONFIG(
            QSPI_PIN_NUM_LCD_CS, nullptr, nullptr
        );
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
            (esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, 
            &io_config, 
            &panel_io
        ));

        st77916_vendor_config_t vendor_config = {
            .init_cmds = vendor_specific_init_yysj,
            .init_cmds_size = sizeof(vendor_specific_init_yysj) / sizeof(st77916_lcd_init_cmd_t),
        };

        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = QSPI_PIN_NUM_LCD_RST,
            .rgb_endian = LCD_RGB_ENDIAN_BGR,
            .bits_per_pixel = 16,
            .vendor_config = &vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(panel_io, &panel_config, &panel));

        // 初始化显示面板
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
        ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, 0, 40));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, true, false));

        // 初始化触摸
        const esp_lcd_touch_config_t tp_cfg = {
            .x_max = QSPI_LCD_H_RES,
            .y_max = QSPI_LCD_V_RES,
            .rst_gpio_num = -1,
            .int_gpio_num = TP_PIN_NUM_INT,
            .flags = {
                .swap_xy = true,
                .mirror_x = true,
                .mirror_y = false,
            },
        };
        ESP_ERROR_CHECK(esp_lcd_touch_new_cst816s(i2c_bus_, 0x15, &tp_cfg, &tp));

#if USE_LVGL_DEFAULT
        display_ = std::make_unique<LcdDisplay>(panel_io, panel, tp,
                                               QSPI_LCD_H_RES, QSPI_LCD_V_RES,
                                               DisplayFonts{
                                                   .text_font = &font_puhui_20_4,
                                                   .icon_font = &font_awesome_20_4,
                                                   .emoji_font = font_emoji_32_init(),
                                               });
#else
        display_ = std::make_unique<anim::EmoteDisplay>(panel_io, panel, tp,
                                                      QSPI_LCD_H_RES, QSPI_LCD_V_RES);
#endif

        backlight_ = std::make_unique<PwmBacklight>(LCD_BACKLIGHT_GPIO);
        backlight_->RestoreBrightness();
    }

public:
    EspS3Cat() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        uint8_t pcb_version = DetectPcbVersion();
        InitializeCharge();
        InitializeSpi();
        Initializest77916Display(pcb_version);
        InitializeCst816sTouchPad();
    }

    ~EspS3Cat() override {
        // 移除中断处理
        if (TP_PIN_NUM_INT != GPIO_NUM_NC) {
            gpio_isr_handler_remove(TP_PIN_NUM_INT);
        }
        
        // 释放I2C总线
        if (i2c_bus_) {
            i2c_del_master_bus(i2c_bus_);
        }
        if (bmi270_i2c_bus_) {
            i2c_del_master_bus(bmi270_i2c_bus_);
        }
        
        // 释放温度传感器
        if (temp_sensor) {
            temperature_sensor_disable(temp_sensor);
            temperature_sensor_uninstall(temp_sensor);
        }
        
        // 释放互斥锁
        if (bmi270_mutex) {
            vSemaphoreDelete(bmi270_mutex);
            bmi270_mutex = nullptr;
        }
    }

    // 实现基类接口
    Led* GetLed() override {
        static RgbLed led(RGB_LED_R_PIN, RGB_LED_G_PIN, RGB_LED_B_PIN);
        return &led;
    }

    Display* GetDisplay() override {
        return display_.get();
    }

    Backlight* GetBacklight() override {
        return backlight_.get();
    }

    AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_,
            I2C_NUM_0,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_GPIO_PA
        );
        return &audio_codec;
    }

    Button* GetButton() override {
        return &boot_button_;
    }
};

DECLARE_BOARD(EspS3Cat);
