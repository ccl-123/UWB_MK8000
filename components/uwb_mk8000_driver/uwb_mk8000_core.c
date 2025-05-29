/**
 * @file uwb_mk8000_core.c
 * @brief UWB模块驱动核心逻辑实现。
 * @details 实现了公共API函数，管理模块状态，并协调UART和AT层。
 * 还包含接收数据的处理和分发逻辑。
 * @date 2025-05-28
 */

#include "include/uwb_mk8000.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

/* ------------------------- 内部定义 ------------------------- */
#define CORE_TAG "UWB_CORE"
#define RX_LINE_BUF_SIZE 256 ///< 行缓冲区大小
#define RANGING_FRAME_SIZE 8 ///< 测距数据帧大小

/* ------------------------- 内部变量 ------------------------- */
static uwb_ranging_callback_t g_ranging_cb = NULL; ///< 测距回调函数
static bool g_driver_initialized = false;          ///< 驱动是否初始化标志
static uwb_mode_t g_current_mode = UWB_MODE_RANGING; ///< 假设模块当前模式
static uint8_t g_rx_line_buffer[RX_LINE_BUF_SIZE]; ///< 接收行缓冲区
static uint16_t g_rx_line_pos = 0;                 ///< 行缓冲区当前位置

// --- 引用外部函数 ---
extern esp_err_t uwb_uart_init_internal(const uwb_uart_config_t* config);
extern esp_err_t uwb_uart_deinit_internal(void);
extern esp_err_t uwb_at_init_internal(void);
extern void uwb_at_deinit_internal(void);
extern esp_err_t uwb_at_send_cmd_sync(const char* cmd, char* response_buf, size_t buf_len, uint32_t timeout_ms);
extern void uwb_at_handle_response_line(const char* line);

extern void uwb_process_received_data(const uint8_t* data, uint16_t len); // 确保这个函数是可访问的


/* ------------------------- 仿真函数 ------------------------- */
#ifdef UWB_SIMULATION_MODE
/**
 * @brief 模拟UWB模块发送测距数据的任务。
 * @details 只在 UWB_SIMULATION_MODE 宏定义时编译和运行。
 * 它会周期性地生成假的测距数据帧，并调用
 * uwb_process_received_data 来模拟接收过程。
 * @param[in] arg 未使用。
 */
static void uwb_ranging_simulator_task(void *arg) {
    uint16_t dist = 50;
    uint16_t slave_addr = 0x0001;
    ESP_LOGW(CORE_TAG, "SIM_MODE: Ranging simulator task started.");
    vTaskDelay(pdMS_TO_TICKS(1000));//等待AT command 完成

    while(1) {
        // 模拟测距周期 ( 200ms)
        vTaskDelay(pdMS_TO_TICKS(200));

        uint8_t frame[RANGING_FRAME_SIZE];
        frame[0] = 0xF0; // 帧头
        frame[1] = 0x05; // 长度
        frame[2] = slave_addr & 0xFF; // 从机地址 L
        frame[3] = (slave_addr >> 8) & 0xFF; // 从机地址 H
        frame[4] = dist & 0xFF; // 距离 L
        frame[5] = (dist >> 8) & 0xFF; // 距离 H
        frame[6] = 180; // 模拟 RSSI (-76dBm)
        frame[7] = 0xAA; // 帧尾

        ESP_LOGD(CORE_TAG, "SIM_MODE: Injecting ranging frame, Dist=%d", dist);
        // 直接调用数据处理函数，模拟接收
        uwb_process_received_data(frame, RANGING_FRAME_SIZE);

        // 改变距离和地址，模拟多个从机
        dist = 50 + (rand() % 50); // 50 到 99 之间变化
        slave_addr++;
        if (slave_addr > 0x0003) {
            slave_addr = 0x0001;
        }
    }
}
#endif // UWB_SIMULATION_MODE



/* ------------------------- 内部函数 ------------------------- */

/**
 * @brief 解析并处理测距数据帧。
 * @param[in] frame 数据帧指针。
 * @param[in] len   数据帧长度。
 */
static void handle_ranging_frame(const uint8_t* frame, uint16_t len)
{
    // 根据数据手册[cite: 30], 帧格式为 F0 05 AddrL AddrH DistL DistH Rssi AA
    if (len == RANGING_FRAME_SIZE && frame[0] == 0xF0 && frame[1] == 0x05 && frame[7] == 0xAA) {
        uwb_ranging_data_t data;
        data.sender_address = frame[2] | (frame[3] << 8);
        data.distance_cm = frame[4] | (frame[5] << 8);
        data.rssi_dbm = (int8_t)(frame[6] - 256); //RSSI = 值 - 256

        ESP_LOGD(CORE_TAG, "Parsed Ranging: Addr=0x%04X, Dist=%u, RSSI=%d",data.sender_address, data.distance_cm, data.rssi_dbm);

        if (g_ranging_cb) {
            g_ranging_cb(&data);// <--- 通过函数指针调用 main.c 中的 ranging_data_handler
        }
    } else {
        ESP_LOGW(CORE_TAG, "Invalid ranging frame detected.");
    }
}

/**
 * @brief 将地址转换为4位十六进制字符串。
 * @param[in] addr 地址。
 * @param[out] hex_str 输出的字符串缓冲区 (至少5字节)。
 */
static void addr_to_hex_str(uint16_t addr, char* hex_str)
{
    sprintf(hex_str, "%04X", addr);
}

/* ------------------------- 数据处理入口 ------------------------- */

/**
 * @brief 处理从UART接收到的原始数据。
 * @details 此函数由 uwb_mk8000_uart.c 中的任务调用。
 * 它负责缓冲数据，检测完整的AT响应行或测距数据帧，
 * 并将其分发给相应的处理函数。
 * @param[in] data 接收到的数据指针。
 * @param[in] len  接收到的数据长度。
 */
void uwb_process_received_data(const uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) 
    {
        uint8_t byte = data[i];

        // 尝试检测测距帧头 F0
        if (byte == 0xF0 && g_rx_line_pos == 0) { // 只有在缓冲区开始时才认F0
            g_rx_line_buffer[0] = 0xF0;
            g_rx_line_pos = 1;
            continue;
        }

        // 如果正在接收测距帧
        if (g_rx_line_pos > 0 && g_rx_line_buffer[0] == 0xF0) {
            g_rx_line_buffer[g_rx_line_pos++] = byte;
            // 检查是否达到完整帧长度
            if (g_rx_line_pos == RANGING_FRAME_SIZE) 
            {
                handle_ranging_frame(g_rx_line_buffer, RANGING_FRAME_SIZE);
                g_rx_line_pos = 0; // 重置缓冲区
            } 
            else if (g_rx_line_pos >= RX_LINE_BUF_SIZE) {
                ESP_LOGW(CORE_TAG, "Ranging frame buffer overflow, resetting.");
                g_rx_line_pos = 0; // 缓冲区溢出，重置
            }
            continue;
        }

        // 否则，假设是AT响应，按行处理
        if (byte == '\n') {
            if (g_rx_line_pos > 0 && g_rx_line_buffer[g_rx_line_pos - 1] == '\r') {
                g_rx_line_buffer[g_rx_line_pos - 1] = '\0'; // 去掉 \r
                uwb_at_handle_response_line((const char*)g_rx_line_buffer);
            } else {
                 g_rx_line_buffer[g_rx_line_pos] = '\0';
                 uwb_at_handle_response_line((const char*)g_rx_line_buffer);
            }
            g_rx_line_pos = 0; // 新行，重置
        } else if (g_rx_line_pos < RX_LINE_BUF_SIZE - 1) {
            g_rx_line_buffer[g_rx_line_pos++] = byte;
        } else {
             ESP_LOGW(CORE_TAG, "AT response line buffer overflow, resetting.");
             g_rx_line_pos = 0; // 缓冲区溢出，重置
        }
    }
}

/* ------------------------- 公共API实现 ------------------------- */

esp_err_t uwb_driver_init(const uwb_uart_config_t* uart_config, uwb_ranging_callback_t ranging_cb)
{
    if (g_driver_initialized) {
        ESP_LOGW(CORE_TAG, "Driver already initialized.");
        return ESP_OK;
    }
    if (uart_config == NULL || ranging_cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    g_ranging_cb = ranging_cb;

    esp_err_t ret = uwb_at_init_internal();
    if (ret != ESP_OK) return ret;

#ifdef UWB_SIMULATION_MODE
    ESP_LOGW(CORE_TAG, "SIM_MODE: Skipping UART initialization.");
    ret = ESP_OK; // 模拟模式下不初始化UART
#else
    ret = uwb_uart_init_internal(uart_config);
#endif

    if (ret != ESP_OK) {
        uwb_at_deinit_internal();
        return ret;
    }

#ifdef UWB_SIMULATION_MODE
    // 创建模拟测距数据任务
    BaseType_t task_ret = xTaskCreate(uwb_ranging_simulator_task,
                                      "uwb_sim_task",
                                      2048,
                                      NULL,
                                      5, 
                                      NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(CORE_TAG, "Failed to create UWB simulator task");
        uwb_driver_deinit(); // 清理
        return ESP_FAIL;
    }
#endif // UWB_SIMULATION_MODE

    g_driver_initialized = true;
    ESP_LOGI(CORE_TAG, "UWB driver initialized %s.",
             #ifdef UWB_SIMULATION_MODE
             "in SIMULATION MODE"
             #else
             "in REAL HARDWARE MODE"
             #endif
             );
    return ESP_OK;
}


esp_err_t uwb_driver_deinit(void)
{
    if (!g_driver_initialized) return ESP_OK;

    uwb_uart_deinit_internal();
    uwb_at_deinit_internal();
    g_ranging_cb = NULL;
    g_driver_initialized = false;
    ESP_LOGI(CORE_TAG, "UWB driver deinitialized.");
    return ESP_OK;
}

esp_err_t uwb_set_work_mode(uwb_mode_t mode)
{
    if (!g_driver_initialized) return ESP_ERR_INVALID_STATE;
    char cmd[20];
    sprintf(cmd, "AT+MODE=%d\r\n", mode); // 根据数据手册[cite: 44]
    esp_err_t ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
    if (ret == ESP_OK) {
        g_current_mode = mode;
    }
    return ret;
}

esp_err_t uwb_configure_module(const uwb_settings_t* settings)
{
    if (!g_driver_initialized || settings == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t ret;
    char cmd[32];
    char addr_str[5];

    ESP_LOGI(CORE_TAG, "Starting UWB module configuration...");

    // 1. 进入AT模式
    ret = uwb_set_work_mode(UWB_MODE_AT_COMMAND);
    if (ret != ESP_OK) {
        ESP_LOGE(CORE_TAG, "Failed to enter AT mode.");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // 给模块一点时间

    // 2. 配置角色
    sprintf(cmd, "AT+ROLE=%d\r\n", settings->role); // 根据数据手册[cite: 46]
    ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
    if (ret != ESP_OK) goto config_error;

    // 3. 配置网络ID
    sprintf(cmd, "AT+PID=%d\r\n", settings->network_id);
    ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
    if (ret != ESP_OK) goto config_error;

    // 4. 配置周期
    sprintf(cmd, "AT+PERIOD=%d\r\n", settings->ranging_period);
    ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
    if (ret != ESP_OK) goto config_error;

    // 5. 配置地址 (根据角色)
    if (settings->role == UWB_ROLE_SLAVE) {
        addr_to_hex_str(settings->self_address, addr_str);
        sprintf(cmd, "AT+MADDR=%s\r\n", addr_str); //  从机模式下, MADDR是本机地址
        ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
        if (ret != ESP_OK) goto config_error;

        addr_to_hex_str(settings->master_address, addr_str);
        sprintf(cmd, "AT+SADDR0=%s\r\n", addr_str); //  从机模式下, SADDR0是主机地址
        ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
        if (ret != ESP_OK) goto config_error;
    } else { // UWB_ROLE_MASTER
        addr_to_hex_str(settings->self_address, addr_str);
        sprintf(cmd, "AT+MADDR=%s\r\n", addr_str); //主机模式下, MADDR是本机地址
        ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
        if (ret != ESP_OK) goto config_error;

        addr_to_hex_str(settings->slave_addr_0, addr_str);
        sprintf(cmd, "AT+SADDR0=%s\r\n", addr_str); 
        ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
        if (ret != ESP_OK) goto config_error;

        addr_to_hex_str(settings->slave_addr_1, addr_str);
        sprintf(cmd, "AT+SADDR1=%s\r\n", addr_str);
        ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
        if (ret != ESP_OK) goto config_error;

        addr_to_hex_str(settings->slave_addr_2, addr_str);
        sprintf(cmd, "AT+SADDR2=%s\r\n", addr_str); 
        ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
        if (ret != ESP_OK) goto config_error;
    }

    // 6. 配置低功耗模式
    sprintf(cmd, "AT+LPWR=%d\r\n", settings->low_power_mode); 
    ret = uwb_at_send_cmd_sync(cmd, NULL, 0, AT_CMD_TIMEOUT_MS);
    if (ret != ESP_OK) goto config_error;

    // 7. 复位模块使配置生效
    ESP_LOGI(CORE_TAG, "Configuration sent, resetting module...");
    ret = uwb_software_reset();
    if (ret != ESP_OK) {
        ESP_LOGE(CORE_TAG, "Failed to reset module.");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(1500)); // 等待模块复位完成

    // 8. 确保进入测距模式 (复位后应默认进入)
    g_current_mode = UWB_MODE_RANGING;
    ESP_LOGI(CORE_TAG, "UWB module configuration completed.");
    return ESP_OK;

config_error:
    ESP_LOGE(CORE_TAG, "Configuration failed! Attempting to exit AT mode.");
    uwb_set_work_mode(UWB_MODE_RANGING); // 尝试恢复
    return ret;
}


esp_err_t uwb_software_reset(void)
{
    if (!g_driver_initialized) return ESP_ERR_INVALID_STATE;
    return uwb_at_send_cmd_sync("AT+RST\r\n", NULL, 0, 1500);
}

esp_err_t uwb_factory_reset(void)
{
    if (!g_driver_initialized) return ESP_ERR_INVALID_STATE;
    return uwb_at_send_cmd_sync("AT+DEFT\r\n", NULL, 0, 1500); 
}

esp_err_t uwb_query_version(char* buffer, size_t buffer_len)
{
    if (!g_driver_initialized) return ESP_ERR_INVALID_STATE;
    return uwb_at_send_cmd_sync("AT+VER\r\n", buffer, buffer_len, AT_CMD_TIMEOUT_MS); 
}

esp_err_t uwb_query_all_params(char* buffer, size_t buffer_len)
{
    if (!g_driver_initialized) return ESP_ERR_INVALID_STATE;
    return uwb_at_send_cmd_sync("AT+ALL\r\n", buffer, buffer_len, AT_CMD_TIMEOUT_MS); 
}