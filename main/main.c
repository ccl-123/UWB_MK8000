/**
 * @file app_main.c
 * @brief UWB模块驱动应用层示例。
 * @details 演示如何初始化UWB驱动，配置模块为“主机”模式，
 * 并通过回调函数接收和打印测距数据。
 * @date 2025-05-28
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "uwb_mk8000.h" 

static const char* TAG = "UWB_APP";

/**
 * @brief 测距数据回调处理函数。
 * @details 当UWB驱动接收并解析到一帧测距数据时，此函数会被调用。
 * @param[in] data 指向测距数据的指针。
 */
void ranging_data_handler(const uwb_ranging_data_t* data)
{
    if (data) {
        // 打印接收到的数据
        ESP_LOGI(TAG, "Ranging Data: Addr=0x%04X, Dist=%u cm, RSSI=%d dBm",
                 data->sender_address,
                 data->distance_cm,
                 data->rssi_dbm);
    }
}

/**
 * @brief app_main
 */
void app_main(void)
{
    // 1. 初始化NVS闪存 
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "UWB Application Started.");

    // 2. 定义UART配置 
    uwb_uart_config_t uart_config = {
        .uart_num = UART_NUM_2,         // 使用UART2 
        .tx_pin = 17,                   // UWB_RX 连接到 ESP32_TX (GPIO17)
        .rx_pin = 16,                   // UWB_TX 连接到 ESP32_RX (GPIO16)
        .baud_rate = UWB_DEFAULT_BAUD_RATE, // 使用模块默认波特率 
        .rx_buffer_size = 1024 * 2,
        .tx_buffer_size = 512
    };

    // 3. 初始化UWB驱动
    ret = uwb_driver_init(&uart_config, ranging_data_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UWB driver!");
        return; // 初始化失败，无法继续
    }
    ESP_LOGI(TAG, "UWB driver initialized.");

    // 给一点时间稳定
    vTaskDelay(pdMS_TO_TICKS(500));

    // 4. 定义模块配置 (配置为“主机”，与从机0x0001通信，周期200ms)
    uwb_settings_t my_settings = {
        .role = UWB_ROLE_MASTER,        // 设置为 主机 
        .self_address = 0x0000,         // 主机地址
        .master_address = 0x0000,       
        .slave_addr_0 = 0x0001,         // 设置从机0地址 
        .slave_addr_1 = 0x0002,         // 设置从机1地址  
        .slave_addr_2 = 0x0003,         // 设置从机2地址 
        .network_id = UWB_DEFAULT_NETWORK_ID, // 使用默认网络ID 
        .ranging_period = 20,           // 20 * 10ms = 200ms 周期 
        .low_power_mode = UWB_LPWR_OFF  // 关闭低功耗 
    };

    // 5. 配置UWB模块
    ESP_LOGI(TAG, "Configuring UWB module...");
    ret = uwb_configure_module(&my_settings);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UWB module! Error: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "UWB module configured. Waiting for ranging data...");
    }

    // 6. 主循环 - 保持运行，测距数据将在回调函数中处理
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 每10秒打印一条心跳信息
        ESP_LOGI(TAG, "Application running...");
    }
}