/**
 * @file app_main.c
 * @brief UWB模块驱动应用层示例。
 * @details 演示如何初始化UWB驱动，配置模块为"主机"模式，
 * 并通过回调函数接收和打印测距数据。
 * @date 2025-05-28
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "uwb_mk8000.h" 

#ifdef UWB_DUAL_UART_SIM_MODE

#include "uwb_module_simulator.h" 
#endif

static const char* TAG = "UWB_APP";

/* 此函数在uwb_mk8000_at.c */
extern esp_err_t uwb_at_send_cmd_sync(const char* cmd, char* response_buf, size_t buf_len, uint32_t timeout_ms);

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
 * @brief 测试UWB查询命令功能
 * @details 测试多种AT查询命令，验证模拟器响应是否符合数据手册规范
 */
void test_uwb_query_commands(void)
{
    esp_err_t ret;
    char buffer[512] = {0};
    
    ESP_LOGI(TAG, "===== 开始UWB查询命令测试 =====");
    
    // 1. 切换到AT命令模式
    ESP_LOGI(TAG, "切换到AT指令模式...");
    ret = uwb_set_work_mode(UWB_MODE_AT_COMMAND);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置AT命令模式失败: %s", esp_err_to_name(ret));
        return;
    }
    
    // 等待模式切换完成
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 2. 测试AT+VER查询
    ESP_LOGI(TAG, "【测试1】查询固件版本 AT+VER");
    memset(buffer, 0, sizeof(buffer));
    ret = uwb_query_version(buffer, sizeof(buffer));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "版本查询成功：\n%s", buffer);
    } else {
        ESP_LOGE(TAG, "版本查询失败: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 3. 测试AT+ALL查询
    ESP_LOGI(TAG, "【测试2】查询所有参数 AT+ALL");
    memset(buffer, 0, sizeof(buffer));
    ret = uwb_query_all_params(buffer, sizeof(buffer));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "全部参数查询成功：\n%s", buffer);
    } else {
        ESP_LOGE(TAG, "全部参数查询失败: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 4. 测试AT+ROLE?查询
    ESP_LOGI(TAG, "【测试3】查询角色参数范围 AT+ROLE?");
    memset(buffer, 0, sizeof(buffer));
    ret = uwb_at_send_cmd_sync("AT+ROLE?\r\n", buffer, sizeof(buffer), AT_CMD_TIMEOUT_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "角色参数范围查询成功：\n%s", buffer);
    } else {
        ESP_LOGE(TAG, "角色参数范围查询失败: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 5. 测试AT+ROLE=?查询
    ESP_LOGI(TAG, "【测试4】查询当前角色值 AT+ROLE=?");
    memset(buffer, 0, sizeof(buffer));
    ret = uwb_at_send_cmd_sync("AT+ROLE=?\r\n", buffer, sizeof(buffer), AT_CMD_TIMEOUT_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "当前角色值查询成功：\n%s", buffer);
    } else {
        ESP_LOGE(TAG, "当前角色值查询失败: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 6. 测试AT+PERIOD?查询
    ESP_LOGI(TAG, "【测试5】查询测距周期参数范围 AT+PERIOD?");
    memset(buffer, 0, sizeof(buffer));
    ret = uwb_at_send_cmd_sync("AT+PERIOD?\r\n", buffer, sizeof(buffer), AT_CMD_TIMEOUT_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "测距周期参数范围查询成功：\n%s", buffer);
    } else {
        ESP_LOGE(TAG, "测距周期参数范围查询失败: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 7. 测试AT+PERIOD=?查询
    ESP_LOGI(TAG, "【测试6】查询当前测距周期值 AT+PERIOD=?");
    memset(buffer, 0, sizeof(buffer));
    ret = uwb_at_send_cmd_sync("AT+PERIOD=?\r\n", buffer, sizeof(buffer), AT_CMD_TIMEOUT_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "当前测距周期值查询成功：\n%s", buffer);
    } else {
        ESP_LOGE(TAG, "当前测距周期值查询失败: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 8. 恢复测距模式
    ESP_LOGI(TAG, "恢复测距模式...");
    ret = uwb_set_work_mode(UWB_MODE_RANGING);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "恢复测距模式失败: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "===== UWB查询命令测试完成 =====");
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

#ifdef UWB_DUAL_UART_SIM_MODE
    ESP_LOGI(TAG, "Dual UART Simulation Mode is ENABLED.");
    // 配置和初始化 UWB 模块模拟器 (UART_B)
    uwb_simulator_uart_config_t sim_uart_config = {
        .uart_num = UART_NUM_1,       // 使用 UART1 作为模拟器端口
        .tx_pin = UWB_SIM_UART_TX_PIN, // 模拟器 UART_B TX (连接到主应用 UART_A RX)
        .rx_pin = UWB_SIM_UART_RX_PIN, // 模拟器 UART_B RX (连接到主应用 UART_A TX)
        .baud_rate = UWB_DEFAULT_BAUD_RATE, // 与主应用UART波特率一致
        .rx_buffer_size = UWB_UART_RX_BUFFER_SIZE,
        .tx_buffer_size = UWB_UART_TX_BUFFER_SIZE
    };
    ret = uwb_simulator_init(&sim_uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UWB module simulator!");
        // 决定是否在此处中止，或允许应用继续（可能没有UWB功能）
    } else {
        ESP_LOGI(TAG, "UWB module simulator initialized on UART%d.", sim_uart_config.uart_num);
    }
    vTaskDelay(pdMS_TO_TICKS(100)); 
#endif

    // 2. 定义主应用 UART 配置 (UART_A)
    uwb_uart_config_t app_uart_config = {
        .uart_num = UART_NUM_2,         // 使用UART2 
        .tx_pin = UWB_UART_TX_PIN,      // UWB_RX 连接到 ESP32_TX
        .rx_pin = UWB_UART_RX_PIN,      // UWB_TX 连接到 ESP32_RX
        .baud_rate = UWB_DEFAULT_BAUD_RATE, // 使用模块默认波特率 
        .rx_buffer_size = UWB_UART_RX_BUFFER_SIZE,
        .tx_buffer_size = UWB_UART_TX_BUFFER_SIZE
    };

    // 3. 初始化UWB驱动
    ret = uwb_driver_init(&app_uart_config, ranging_data_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UWB driver!");
        return;
    }
    ESP_LOGI(TAG, "UWB driver initialized.");

    // 给一点时间稳定
    ESP_LOGI(TAG, "Waiting for system to stabilize before UWB configuration...");
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    // 4. 定义模块配置 (配置为"主机"，与从机通信，周期200ms)
    uwb_settings_t my_settings = {
        .role = UWB_ROLE_MASTER,        // 设置为 主机 
        .self_address = UWB_MASTER_SELF_ADDR, // 主机地址
        .master_address = UWB_MASTER_SELF_ADDR, //从机模式调用
        .slave_addr_0 = UWB_SLAVE_ADDR_0,    // 设置从机0地址 
        .slave_addr_1 = UWB_SLAVE_ADDR_1,    // 设置从机1地址  
        .slave_addr_2 = UWB_SLAVE_ADDR_2,    // 设置从机2地址 
        .network_id = UWB_DEFAULT_NETWORK_ID, // 使用默认网络ID 
        .ranging_period = UWB_RANGING_PERIOD, // 测距周期 
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

    // 6. 运行UWB查询命令测试
    ESP_LOGI(TAG, "Running UWB query command tests after 3 seconds...");
    vTaskDelay(pdMS_TO_TICKS(3000)); // 等待3秒再开始测试
    test_uwb_query_commands();

    // 7. 主循环 - 保持运行，测距数据将在回调函数中处理
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 每10秒打印一条心跳信息
        ESP_LOGI(TAG, "Application running...");
    }
}