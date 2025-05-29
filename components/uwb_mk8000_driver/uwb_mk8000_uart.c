/**
 * @file uwb_mk8000_uart.c
 * @brief UWB模块UART底层通信实现。
 * @details 负责UART端口的初始化、数据发送、以及创建FreeRTOS任务
 * 来异步接收和初步处理UART数据。
 * @date 2025-05-28
 */

#include "include/uwb_mk8000.h"
#include "esp_log.h"
#include <string.h>

/* ------------------------- 内部定义 ------------------------- */
#define UART_TAG "UWB_UART"
#define UART_RX_BUF_SIZE (1024) ///< UART接收缓冲区大小
#define UART_TIMEOUT_MS (100)   ///< UART读取超时时间

/* ------------------------- 内部变量 ------------------------- */
static uart_port_t      g_uart_port;           ///< 使用的UART端口号
static QueueHandle_t    g_uart_queue = NULL;     ///< UART事件队列
static TaskHandle_t     g_uart_rx_task_handle = NULL; ///< UART接收任务句柄

// --- 函数声明 ---
void uwb_process_received_data(const uint8_t* data, uint16_t len);

/* ------------------------- FreeRTOS任务 ------------------------- */

#ifndef UWB_SIMULATION_MODE // 只有在非模拟模式下才编译和运行真实UART任务
/**
 * @brief UART接收和事件处理任务。
 * @details 持续监听UART事件队列，当有数据到达时读取数据，
 * 并调用处理函数进行解析。
 * @param[in] arg 未使用。
 */
static void uwb_uart_event_task(void *arg)
{
    uart_event_t event;
    uint8_t *rx_buffer = (uint8_t *) malloc(UART_RX_BUF_SIZE);
    if (rx_buffer == NULL) {
        ESP_LOGE(UART_TAG, "Failed to allocate RX buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(UART_TAG, "UART RX task started.");

    while (1) {
        // 等待UART事件
        if (xQueueReceive(g_uart_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    {
                        // 读取UART接收到的数据
                        int len = uart_read_bytes(g_uart_port, rx_buffer, event.size, pdMS_TO_TICKS(UART_TIMEOUT_MS));
                        if (len > 0) 
                        {
                            ESP_LOGD(UART_TAG, "UART received %d bytes.", len);
                            ESP_LOG_BUFFER_HEXDUMP(UART_TAG, rx_buffer, len, ESP_LOG_DEBUG);
                            // 调用数据处理函数
                            uwb_process_received_data(rx_buffer, len);
                        } else if (len < 0) {
                            ESP_LOGE(UART_TAG, "UART read error");
                        }
                    }
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGW(UART_TAG, "UART RX FIFO overflow");
                    uart_flush_input(g_uart_port);
                    xQueueReset(g_uart_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(UART_TAG, "UART RX buffer full");
                    uart_flush_input(g_uart_port);
                    xQueueReset(g_uart_queue);
                    break;

                case UART_BREAK:
                    ESP_LOGW(UART_TAG, "UART RX break detected");
                    break;

                case UART_PARITY_ERR:
                    ESP_LOGE(UART_TAG, "UART parity error");
                    break;

                case UART_FRAME_ERR:
                    ESP_LOGE(UART_TAG, "UART frame error");
                    break;

                default:
                    ESP_LOGI(UART_TAG, "UART event type: %d", event.type);
                    break;
            }
        }
    }

    free(rx_buffer);
    rx_buffer = NULL;
    vTaskDelete(NULL);
}
#endif // !UWB_SIMULATION_MODE

/* ------------------------- 内部API实现 ------------------------- */

/**
 * @brief 初始化UART驱动和接收任务。
 * @note 此函数由 uwb_driver_init 调用。
 * @param[in] config UART配置参数。
 * @return esp_err_t
 */
esp_err_t uwb_uart_init_internal(const uwb_uart_config_t* config)
{
#ifdef UWB_SIMULATION_MODE
    ESP_LOGW(UART_TAG, "SIM_MODE: Skipping UART driver install and task creation.");
    g_uart_port = config->uart_num; // 仍然保存端口号, 尽管不用
    return ESP_OK;
#else
    
    g_uart_port = config->uart_num;

    uart_config_t uart_config_idf = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装UART驱动，并配置事件队列
    esp_err_t ret = uart_driver_install(g_uart_port,
                                        config->rx_buffer_size,
                                        config->tx_buffer_size,
                                        20, // UART事件队列大小
                                        &g_uart_queue,
                                        0); // 中断分配标志
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(g_uart_port, &uart_config_idf);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "Failed to set UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(g_uart_port, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // 创建UART接收任务
    BaseType_t task_ret = xTaskCreate(uwb_uart_event_task,
                                      "uwb_uart_event_task",
                                      3072, 
                                      NULL,
                                      10,   
                                      &g_uart_rx_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(UART_TAG, "Failed to create UART RX task");
        uart_driver_delete(g_uart_port);
        return ESP_FAIL;
    }

    return (task_ret == pdPASS) ? ESP_OK : ESP_FAIL;
#endif // UWB_SIMULATION_MODE
}

/**
 * @brief 卸载UART驱动并删除任务。
 * @note 此函数由 uwb_driver_deinit 调用。
 * @return esp_err_t
 */
esp_err_t uwb_uart_deinit_internal(void)
{
#ifdef UWB_SIMULATION_MODE
    ESP_LOGW(UART_TAG, "SIM_MODE: Skipping UART driver deinit.");
    return ESP_OK;
#else
    if (g_uart_rx_task_handle != NULL) {
        vTaskDelete(g_uart_rx_task_handle);
        g_uart_rx_task_handle = NULL;
    }

    if (uart_is_driver_installed(g_uart_port)) {
        uart_driver_delete(g_uart_port);
    }
    g_uart_queue = NULL;
    return ESP_OK;
#endif // UWB_SIMULATION_MODE
}

/**
 * @brief 通过UART发送数据。
 * @param[in] data 要发送的数据指针。
 * @param[in] len  要发送的数据长度。
 * @return int 实际发送的字节数，或-1表示失败。
 */
int uwb_uart_send_data(const char* data, size_t len)
{
#ifdef UWB_SIMULATION_MODE
    ESP_LOGW(UART_TAG, "SIM_MODE: UART send ignored.");
    return len; // 假装发送成功
#else
    if (!uart_is_driver_installed(g_uart_port)) {
        ESP_LOGE(UART_TAG, "UART driver not installed.");
        return -1;
    }
    ESP_LOGD(UART_TAG, "UART sending %d bytes: %.*s", len, len, data);
    return uart_write_bytes(g_uart_port, data, len);
#endif // UWB_SIMULATION_MODE
}