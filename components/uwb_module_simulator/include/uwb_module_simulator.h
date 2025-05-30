#ifndef UWB_MODULE_SIMULATOR_H
#define UWB_MODULE_SIMULATOR_H

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>


#include "../../uwb_mk8000_driver/include/uwb_mk8000.h" 

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UWB模块模拟器UART配置结构体
 */
typedef struct {
    uart_port_t uart_num;     ///< 模拟器使用的UART端口号 (e.g., UART_NUM_1)
    int tx_pin;               ///< 模拟器UART TX引脚号
    int rx_pin;               ///< 模拟器UART RX引脚号
    int baud_rate;            ///< 模拟器UART 通信波特率
    int rx_buffer_size;       ///< 模拟器UART RX 缓冲区大小
    int tx_buffer_size;       ///< 模拟器UART TX 缓冲区大小
} uwb_simulator_uart_config_t;

/**
 * @brief 初始化UWB模块模拟器
 * @details 初始化模拟器使用的UART端口，创建后台任务处理AT指令并发送模拟测距数据。
 * @param[in] sim_uart_config 模拟器UART配置参数。
 * @return esp_err_t
 * - ESP_OK: 成功
 * - ESP_FAIL: 失败
 */
esp_err_t uwb_simulator_init(const uwb_simulator_uart_config_t* sim_uart_config);

/**
 * @brief 卸载UWB模块模拟器
 * @details 停止后台任务，释放资源，卸载UART驱动。
 * @return esp_err_t
 * - ESP_OK: 成功
 */
esp_err_t uwb_simulator_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // UWB_MODULE_SIMULATOR_H 