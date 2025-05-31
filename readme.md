# ESP32 UWB无线测距模块 (MK8000) AT指令及测距功能

## 简介

本项目是深圳硅传科技的MK8000PATR7.9-GC UWB（超宽带）无线测距模块进行交互。项目实现了通过AT指令配置UWB模块，并接收和解析模块发送的测距数据（包括距离、信号强度和发送方地址）的功能。

项目包含UWB模块驱动程序，以及一个可选的UWB模块模拟器，在没有实际硬件的情况下进行开发和测试。


## 项目结构

### 主要组件

1.  **`main`** (`main/main.c`)
    *   应用程序主入口。
    *   负责系统初始化、UWB驱动初始化、UWB模块模拟器初始化（如果启用）。
    *   定义并应用UWB模块的配置参数。
    *   提供测距数据的回调处理函数。

2.  **`uwb_mk8000_driver`** (`components/uwb_mk8000_driver/`)
    *   **`include/uwb_mk8000.h`**: 驱动的公共API接口、数据结构、枚举和宏定义。
    *   **`uwb_mk8000_core.c`**: 驱动核心逻辑，实现API，管理模块状态，协调AT和UART层，处理和分发接收到的数据。
    *   **`uwb_mk8000_at.c`**: AT指令的构造、同步发送和响应处理。
    *   **`uwb_mk8000_uart.c`**: 底层UART通信的实现，包括初始化、数据发送和异步数据接收任务。

3.  **`uwb_module_simulator`** (`components/uwb_module_simulator/`)
    *   **`include/uwb_module_simulator.h`**: 模拟器的公共API接口。
    *   **`uwb_module_simulator.c`**: UWB模块行为的模拟实现。它通过一个独立的UART接口与`uwb_mk8000_driver`通信，响应AT指令并模拟发送测距数据。

### 目录概览
```
.
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   └── main.c                 # 主应用逻辑
├── components/
│   ├── uwb_mk8000_driver/     # UWB模块驱动组件
│   │   ├── CMakeLists.txt
│   │   ├── include/uwb_mk8000.h
│   │   ├── uwb_mk8000_core.c
│   │   ├── uwb_mk8000_at.c
│   │   └── uwb_mk8000_uart.c
│   └── uwb_module_simulator/  # UWB模块模拟器组件
│       ├── CMakeLists.txt
│       ├── include/uwb_module_simulator.h
│       └── uwb_module_simulator.c
└── readme.md                  
```

## 硬件连接 (与真实MK8000模块)

当不使用模拟器，而是连接真实的MK8000 UWB模块时：
*   **ESP32 `VCC`** <-> **MK8000 `VCC`** 
*   **ESP32 `GND`** <-> **MK8000 `GND`**
*   **ESP32 `UART_TX_PIN`** <-> **MK8000 `RX_PIN`** 
*   **ESP32 `UART_RX_PIN`** <-> **MK8000 `TX_PIN`** 
## 软件流程

1.  **初始化**：
    *   如果启用了模拟模式 (`UWB_DUAL_UART_SIM_MODE = 1`)，则初始化 `uwb_module_simulator` 组件，它会配置模拟器使用的UART端口（例如UART1）并启动模拟器任务。
    *   初始化 `uwb_mk8000_driver` 组件，配置应用层与驱动通信的UART端口（例如UART2），并注册测距数据回调函数。驱动初始化时会建立其UART通信任务。

2.  **模块配置**：
    *   `app_main` 函数定义一组 `uwb_settings_t` 结构体，包含UWB模块的目标工作参数（如角色：主机，本机地址：0x0000，从机地址0/1/2：0x0001/0x0002/0x0003，测距周期等）。
    *   调用 `uwb_configure_module()` API。此函数会：
        *   首先发送 `AT+MODE=0` 使模块（或模拟器）进入AT指令模式。
        *   然后按顺序发送一系列AT指令来设置角色、网络ID、周期、地址等参数。
        *   最后发送 `AT+RST` 来复位模块（或模拟器），使配置生效并通常切换回测距模式。

3.  **数据处理**：
    *   **AT指令响应**：驱动中的 `uwb_at_send_cmd_sync` 发送指令后，会等待模块（或模拟器）的响应。响应数据由 `uwb_mk8000_uart.c` 中的任务接收，经由 `uwb_process_received_data` 传递给 `uwb_at_handle_response_line` 进行处理和同步。
    *   **测距数据**：
        *   在测距模式下，UWB模块（或模拟器）会主动发送测距数据帧。
        *   这些数据帧由驱动的UART接收任务捕获，传递给 `uwb_process_received_data`。
        *   `uwb_process_received_data` 识别出测距数据帧后，调用 `handle_ranging_frame` 进行解析。
        *   `handle_ranging_frame` 从数据帧中提取发送方地址、距离和RSSI，填充到 `uwb_ranging_data_t` 结构体中。
        *   最后，调用在 `uwb_driver_init` 时注册的回调函数（即 `main.c` 中的 `ranging_data_handler`），将解析后的数据传递给应用层。
        *   `ranging_data_handler` 简单地将接收到的测距信息打印到日志。

## 模拟模式

本项目通过 `uwb_mk8000.h` 文件中的 `UWB_DUAL_UART_SIM_MODE` 宏来控制是否启用UWB模块模拟器。

*   **启用模拟模式**：
    *   确保 `UWB_DUAL_UART_SIM_MODE` 定义为 `1`。
    *   在 `main.c` 中，应用UART（例如 `UART_NUM_2`）配置用于与驱动交互，模拟器UART（例如 `UART_NUM_1`）配置用于模拟器自身。
    *   物理连接（或ESP32内部路由，如果引脚配置允许）：
        *   应用UART TX -> 模拟器UART RX
        *   应用UART RX -> 模拟器UART TX
    *   这样，驱动发送的AT指令会被模拟器接收和响应，模拟器生成的测距数据会被驱动接收和处理。

*   **禁用模拟模式** (连接真实硬件模块):
    *   将 `UWB_DUAL_UART_SIM_MODE` 注释掉。
    *   此时，`main.c` 不会初始化模拟器组件。


## AT 指令集参考

UWB模块的具体AT指令集、参数范围和详细，需要参考模块供应商提供的数据手册 
--- 