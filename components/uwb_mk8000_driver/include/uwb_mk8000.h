/**
 * @file uwb_mk8000.h
 * @brief MK8000PATR7.9-GC UWB模块驱动程序公共API接口定义。
 * @details 定义了与UWB模块交互所需的数据结构、枚举、回调函数以及
 * 上层应用可以调用的主要API函数。
 * @date 2025-05-28
 */

#ifndef UWB_MK8000_H
#define UWB_MK8000_H

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* +++++++++++++++++++++++++ 仿真模拟模式开关 +++++++++++++++++++++++++ */
// 定义这个宏来开启模拟模式。在实际部署时需要移除，
#define UWB_SIMULATION_MODE 1
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */



/* ------------------------- 宏定义 ------------------------- */
#define UWB_DEFAULT_BAUD_RATE 115200 ///< 模块出厂默认波特率
#define UWB_DEFAULT_NETWORK_ID 255   ///< 模块出厂默认网络ID
#define UWB_DEFAULT_PERIOD 100       ///< 模块出厂默认测距周期 (100 * 10ms = 1s)
#define AT_CMD_TIMEOUT_MS (2000)     ///< AT指令响应默认超时时间 (新增)

/* ------------------------- 枚举定义 ------------------------- */

/**
 * @brief UWB模块角色定义
 */
typedef enum {
    UWB_ROLE_SLAVE = 0, ///< 从机模式 (标签)
    UWB_ROLE_MASTER = 1 ///< 主机模式 (基站)
} uwb_role_t;

/**
 * @brief UWB模块工作模式定义
 */
typedef enum {
    UWB_MODE_AT_COMMAND = 0, ///< AT指令模式
    UWB_MODE_RANGING = 1     ///< 测距模式 (默认)
} uwb_mode_t;

/**
 * @brief UWB模块低功耗模式定义
 */
typedef enum {
    UWB_LPWR_OFF = 0, ///< 关闭低功耗模式 (默认)
    UWB_LPWR_ON = 1   ///< 开启低功耗模式
} uwb_lpwr_t;

/* ------------------------- 结构体定义 ------------------------- */

/**
 * @brief UWB模块UART配置结构体
 */
typedef struct {
    uart_port_t uart_num;     ///< 使用的UART端口号 (e.g., UART_NUM_1)
    int tx_pin;               ///< UART TX引脚号
    int rx_pin;               ///< UART RX引脚号
    int baud_rate;            ///< UART 通信波特率
    int rx_buffer_size;       ///< UART RX 缓冲区大小
    int tx_buffer_size;       ///< UART TX 缓冲区大小
} uwb_uart_config_t;

/**
 * @brief UWB模块设置结构体 (用于配置)
 */
typedef struct {
    uwb_role_t role;          ///< 模块角色
    uint16_t self_address;    ///< 本机地址 (HEX)
    uint16_t master_address;  ///< 主机地址 (HEX) (仅从机模式有效)
    uint16_t slave_addr_0;    ///< 从机0地址 (HEX) (仅主机模式有效)
    uint16_t slave_addr_1;    ///< 从机1地址 (HEX) (仅主机模式有效)
    uint16_t slave_addr_2;    ///< 从机2地址 (HEX) (仅主机模式有效)
    uint8_t  network_id;      ///< 网络ID (0-255)
    uint8_t  ranging_period;  ///< 测距周期 (5-100, 单位 10ms)
    uwb_lpwr_t low_power_mode;///< 低功耗模式
    // 可根据需要添加功率等级等其他设置
} uwb_settings_t;

/**
 * @brief UWB测距数据结构体
 */
typedef struct {
    uint16_t sender_address; ///< 发送方地址
    uint16_t distance_cm;    ///< 距离 (cm)
    int8_t   rssi_dbm;       ///< 信号强度 (dBm)
} uwb_ranging_data_t;

/* ------------------------- 回调函数定义 ------------------------- */

/**
 * @brief 测距数据回调函数指针类型
 * @param data 指向接收到的测距数据结构体的指针
 */
typedef void (*uwb_ranging_callback_t)(const uwb_ranging_data_t* data);

/**
 * @brief AT指令响应回调函数指针类型 (可选，用于异步处理)
 * @param response 指向接收到的AT响应字符串的指针
 */
typedef void (*uwb_at_response_callback_t)(const char* response);


/* ------------------------- 公共API函数声明 ------------------------- */

/**
 * @brief 初始化UWB驱动程序
 * @details 初始化UART通信，创建后台任务处理数据接收和解析，
 * 并设置回调函数。
 * @param[in] uart_config UART配置参数。
 * @param[in] ranging_cb  接收到测距数据时的回调函数。
 * @return esp_err_t
 * - ESP_OK: 成功
 * - ESP_FAIL: 失败 (例如内存分配失败, 任务创建失败)
 * - ESP_ERR_INVALID_ARG: 无效参数
 */
esp_err_t uwb_driver_init(const uwb_uart_config_t* uart_config, uwb_ranging_callback_t ranging_cb);

/**
 * @brief 卸载UWB驱动程序
 * @details 停止后台任务，释放资源，卸载UART驱动。
 * @return esp_err_t
 * - ESP_OK: 成功
 */
esp_err_t uwb_driver_deinit(void);

/**
 * @brief 设置UWB模块的工作模式
 * @details 发送 AT+MODE 指令切换模块工作模式。
 * @param[in] mode 要设置的工作模式 (AT指令或测距)。
 * @return esp_err_t
 * - ESP_OK: 成功
 * - ESP_FAIL: 发送或接收失败
 * - ESP_ERR_TIMEOUT: 等待响应超时
 */
esp_err_t uwb_set_work_mode(uwb_mode_t mode);

/**
 * @brief 配置UWB模块参数
 * @details 自动进入AT模式，发送一系列配置指令，然后复位模块使配置生效。
 * @param[in] settings 包含所有待配置参数的结构体。
 * @return esp_err_t
 * - ESP_OK: 成功
 * - ESP_FAIL: 某个配置步骤失败
 * - ESP_ERR_TIMEOUT: 等待响应超时
 */
esp_err_t uwb_configure_module(const uwb_settings_t* settings);

/**
 * @brief 软件复位UWB模块
 * @details 发送 AT+RST 指令。
 * @return esp_err_t
 * - ESP_OK: 成功
 * - ESP_FAIL: 发送或接收失败
 * - ESP_ERR_TIMEOUT: 等待响应超时
 */
esp_err_t uwb_software_reset(void);

/**
 * @brief 恢复UWB模块出厂设置
 * @details 发送 AT+DEFT 指令。
 * @return esp_err_t
 * - ESP_OK: 成功
 * - ESP_FAIL: 发送或接收失败
 * - ESP_ERR_TIMEOUT: 等待响应超时
 */
esp_err_t uwb_factory_reset(void);

/**
 * @brief 查询UWB模块固件版本
 * @details 发送 AT+VER 指令。
 * @param[out] buffer 存储查询结果的缓冲区。
 * @param[in]  buffer_len 缓冲区大小。
 * @return esp_err_t
 * - ESP_OK: 成功
 * - ESP_FAIL: 发送或接收失败
 * - ESP_ERR_TIMEOUT: 等待响应超时
 */
esp_err_t uwb_query_version(char* buffer, size_t buffer_len);

/**
 * @brief 查询UWB模块所有参数
 * @details 发送 AT+ALL 指令。
 * @param[out] buffer 存储查询结果的缓冲区。
 * @param[in]  buffer_len 缓冲区大小。
 * @return esp_err_t
 * - ESP_OK: 成功
 * - ESP_FAIL: 发送或接收失败
 * - ESP_ERR_TIMEOUT: 等待响应超时
 */
esp_err_t uwb_query_all_params(char* buffer, size_t buffer_len);


#ifdef __cplusplus
}
#endif

#endif // UWB_MK8000_H