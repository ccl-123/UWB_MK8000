/**
 * @file uwb_mk8000_at.c
 * @brief UWB模块AT指令的构建、发送和响应处理实现。
 * @details 提供发送AT指令并同步等待响应的机制。
 * @date 2025-05-28
 */

#include "include/uwb_mk8000.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ------------------------- 内部定义 ------------------------- */
#define AT_TAG "UWB_AT"
#define AT_RESPONSE_BUF_SIZE (512) ///< AT响应缓冲区大小

/* ------------------------- 内部变量 ------------------------- */
static SemaphoreHandle_t    g_at_response_sem = NULL; ///< AT响应同步信号量
static char                 g_at_response_buffer[AT_RESPONSE_BUF_SIZE]; ///< AT响应缓冲区
static bool                 g_at_response_ok = false; ///< AT响应是否为OK

// --- 引用外部函数 ---
extern int uwb_uart_send_data(const char* data, size_t len);

/* ------------------------- 内部API ------------------------- */

/**
 * @brief 初始化AT指令处理模块。
 * @note 由 uwb_driver_init 调用。
 * @return esp_err_t
 */
esp_err_t uwb_at_init_internal(void)
{
    if (g_at_response_sem == NULL) {
        g_at_response_sem = xSemaphoreCreateBinary();
        if (g_at_response_sem == NULL) {
            ESP_LOGE(AT_TAG, "Failed to create AT response semaphore");
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

/**
 * @brief 反初始化AT指令处理模块。
 * @note 由 uwb_driver_deinit 调用。
 */
void uwb_at_deinit_internal(void)
{
    if (g_at_response_sem != NULL) {
        vSemaphoreDelete(g_at_response_sem);
        g_at_response_sem = NULL;
    }
}

/**
 * @brief 发送AT指令并同步等待响应。(支持模拟模式)
 * @param[in] cmd 要发送的AT指令字符串 (必须以 \r\n 结尾)。
 * @param[out] response_buf 存储响应的缓冲区 (可选, 可为NULL)。
 * @param[in] buf_len 缓冲区大小。
 * @param[in] timeout_ms 等待响应的超时时间。
 * @return esp_err_t
 * - ESP_OK: 收到 "OK" 响应
 * - ESP_FAIL: 收到 "ERROR" 响应
 * - ESP_ERR_TIMEOUT: 等待超时
 * - ESP_ERR_INVALID_STATE: 信号量未创建
 * - ESP_ERR_NO_MEM: 发送失败
 */
esp_err_t uwb_at_send_cmd_sync(const char* cmd, char* response_buf, size_t buf_len, uint32_t timeout_ms)
{
    if (g_at_response_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // 清空上次响应并获取信号量 (确保它是空的)
    memset(g_at_response_buffer, 0, AT_RESPONSE_BUF_SIZE);
    xSemaphoreTake(g_at_response_sem, 0);

    ESP_LOGW(AT_TAG, ">>> SENDING AT CMD: [%s] (len: %d)", cmd, strlen(cmd));
    
    if (uwb_uart_send_data(cmd, strlen(cmd)) <= 0) {//发送非NULL命令uwb_uart_send_data
        ESP_LOGE(AT_TAG, "Failed to send AT command: %s", cmd);
        return ESP_ERR_NO_MEM;
    }

    if (xSemaphoreTake(g_at_response_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        // 收到了响应 (或模拟响应)
        if (response_buf != NULL && buf_len > 0) {
            strncpy(response_buf, g_at_response_buffer, buf_len - 1);
            response_buf[buf_len - 1] = '\0';
        }
        return g_at_response_ok ? ESP_OK : ESP_FAIL;
    } else {
        // 等待超时
        ESP_LOGE(AT_TAG, "Timeout waiting for response for: %s", cmd);
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief 处理接收到的AT响应行。
 * @note 此函数由 uwb_process_received_data 调用。
 * @param[in] line 接收到的AT响应行 (不含 \r\n)。
 */
void uwb_at_handle_response_line(const char* line)
{
    if (line == NULL) return;

    ESP_LOGD(AT_TAG, "Handling AT line: [%s]", line);

    // 检查是否是 OK 或 ERROR
    bool is_final_response = false;
    if (strcmp(line, "OK") == 0) {
        g_at_response_ok = true;
        is_final_response = true;
    } else if (strcmp(line, "ERROR") == 0) {
        g_at_response_ok = false;
        is_final_response = true;
    }

    // 累加响应到缓冲区 (简单实现)
    // 实际应处理多行响应，如 AT+ALL
    strncat(g_at_response_buffer, line, AT_RESPONSE_BUF_SIZE - strlen(g_at_response_buffer) - 1);
    strncat(g_at_response_buffer, "\n", AT_RESPONSE_BUF_SIZE - strlen(g_at_response_buffer) - 1);


    // 如果是最终响应 (OK/ERROR)，则释放信号量
    if (is_final_response && g_at_response_sem != NULL) {
        xSemaphoreGive(g_at_response_sem);
    }
}