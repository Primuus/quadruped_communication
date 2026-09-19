# quadruped_communication

> 四足机器人通信/控制转发板固件 —— 把 ROS 2 的关节力矩指令，通过双路 FDCAN 分发给 12 个 RobStride 电机，并接入自研 QLP 串行总线上的遥控器与外设。

运行平台：**STM32H743IIT6** ｜ 实时内核：**FreeRTOS (CMSIS-RTOS2)** ｜ 中间件：**micro-ROS / XRCE-DDS** ｜ 构建：**CMake + arm-none-eabi-gcc**

---

## 目录

- [项目简介](#项目简介)
- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [硬件资源](#硬件资源)
- [软件架构](#软件架构)
- [通信协议](#通信协议)
- [目录结构](#目录结构)
- [构建](#构建)
- [烧录与调试](#烧录与调试)
- [已知问题](#已知问题)
- [路线图](#路线图)
- [相关文档](#相关文档)

---

## 项目简介

本工程是四足机器人上的一块**通信桥接（bridge）与电机控制板**固件。上位机（ROS 2）以 `Float64MultiArray` 形式发布 12 个关节的目标力矩，本板接收后按两条 CAN 总线拆分为 12 帧 RobStride 私有协议报文下发电机；同时通过 QLP 协议接入遥控器、蜂鸣器、数码管等低速外设。

工程按 **CubeMX 生成层（`Core/`）+ 手写应用层（`App/`）** 分层组织，应用层以「一个外设/链路 = 一个任务文件」为原则，便于裁剪与移植。

> 工程内部标识为 `quadruped_ant`（CMake `project()` 名与产物名前缀），仓库目录名为 `quadruped_communication`。

---

## 功能特性

- ✅ **micro-ROS 订阅**：通过 UART7 + DMA 自定义传输运行 micro-ROS 节点，订阅 `/real_joint_effort_controller/commands`（12 维力矩）。
- ✅ **双路 FDCAN 力矩下发**：FDCAN1 / FDCAN2 各 1 Mbps，Classic 帧，扩展 ID，RobStride 私有协议。
- ✅ **电机使能**：上电自动对两路总线各 6 个电机发送使能帧。
- ✅ **RobStride 运控打包**：力矩/位置/速度/KP/KD 的浮点→16bit 线性映射与限幅。
- ✅ **QLP 自研串行协议**：UART8 + DMA(IDLE) 收帧，CRC-16/MODBUS 校验，SOF 快速重同步状态机。
- ✅ **蜂鸣器控制**：按帧尾字节区分短鸣/长鸣，支持多次鸣叫排程。
- ✅ **FreeRTOS 任务/队列模型**：收发解耦，ISR 只投递消息，任务侧解析与下发。
- 🚧 **CAN 反馈接收**（电机位置/速度/电流）—— 未实现。
- 🚧 **IMU 接入**（QLP FC `0x91`）—— 未实现。
- 🚧 **UART7 上的 QLP 收发转发** —— 代码已具备但默认禁用（见[已知问题](#已知问题)）。

---

## 系统架构

```
                        ┌───────────────────────────────────────────────────────┐
   ROS 2 上位机          │                 STM32H743 (quadruped_ant)             │
                        │                                                       │
  rclcpp 发布            │   UART7 @115200                                       │
  /real_joint_effort_    │   (XRCE-DDS DMA transport)                            │
  controller/commands ──►│────────────► microRosTask ──┐                          │
       Float64[12]       │                             │  tau_cmd_callback        │
                        │                             ▼                          │
                        │                   g_tau_cmd (12×double, volatile)       │
                        │                    valid / update_count                │
                        │                             │                          │
                        │            ┌────────────────┴────────────────┐         │
                        │            ▼ (轮询 update_count)             ▼         │
                        │      can1TransTask                    can2TransTask     │
                        │            │                                  │        │
                        │       RobStride 打包                      RobStride 打包 │
                        │            ▼                                  ▼        │
                        │      FDCAN1 @1Mbps                     FDCAN2 @1Mbps    │
                        │      motor id 0~5                      motor id 0~5     │
                        │            └──────────► 12 电机 ◄─────────────┘        │
                        │                        ▲                               │
  遥控器 / 外设           │   UART8 @115200        │                               │
  (QLP 总线) ───────────►│──► uart8RecTask ─► Qlp_ParseBytes ─► OnFrame           │
                        │                                   ├─► beepFrameQueue ─► beepTask ─► BEEP
                        │                                   └─► (uart7FrameQueue，默认禁用)
                        └───────────────────────────────────────────────────────┘
```

---

## 硬件资源

### MCU 与时钟

| 项目 | 配置 |
|---|---|
| MCU | STM32H743IIT6（Cortex-M7, LQFP176） |
| 主频 | 480 MHz（Cortex / AXI / D1CPRE） |
| AHB / APB | 240 MHz / 120 MHz |
| HSE | 25 MHz（PLL1：M=5, N=192, P=2） |
| 电压域 | `PWR_LDO_SUPPLY`，`VOS0` |
| MPU | Region0：0x0 / 4GB / No-Access（CubeMX 默认） |

### 外设分配

| 外设 | 用途 | 参数 | 引脚/说明 |
|---|---|---|---|
| **UART7** | micro-ROS (XRCE-DDS) | 115200, DMA RX/TX | DMA1_Stream1/2，IDLE 事件 |
| **UART8** | QLP 串行总线（遥控器/外设） | 115200, DMA RX | DMA1_Stream0，IDLE 事件 |
| **FDCAN1** | RobStride 电机 1~6 | 1 Mbps, Classic, Ext ID | 运控/使能帧 |
| **FDCAN2** | RobStride 电机 7~12 | 1 Mbps, Classic, Ext ID | 运控/使能帧 |
| **USART1** | 预留 | 1 Mbps | 已初始化，未使用 |
| **UART5** | 预留 | 115200 | 已初始化，未使用 |
| **TIM6** | HAL 时基 | — | `HAL_IncTick` |

### GPIO

| 定义 | 引脚 | 用途 |
|---|---|---|
| `LED0` | PC4 | 状态指示 / 调试 |
| `LED1` | PC5 | 状态指示 |
| `LED2` | PB0 | CAN1 发送活动指示 |
| `LED3` | PB1 | 力矩下发活动指示 |
| `BEEP` | PI1 | 蜂鸣器 |
| `M0` / `M1` / `AUX` | PI7 / PI6 / PI5 | 预留 |

---

## 软件架构

### 任务（`Core/Src/freertos.c`）

| 任务 | 优先级 | 栈 | 状态 | 职责 |
|---|---|---|---|---|
| `microRosTask` | Low | 3 KB | **启用** | micro-ROS 节点初始化与 `rclc_executor_spin_some` |
| `uart8RecTask` | Low | 2 KB | **启用** | QLP 收帧与分发 |
| `can1TransTask` | Low | 2 KB | **启用** | 力矩 → FDCAN1 |
| `can2TransTask` | Low | 2 KB | **启用** | 力矩 → FDCAN2 |
| `beepTask` | Low | 1 KB | **启用** | 蜂鸣器鸣叫排程 |
| `can1RecTask` | Low | 1 KB | 空转 | ❌ 待实现 CAN RX |
| `can2RecTask` | Low | 1 KB | 空转 | ❌ 待实现 CAN RX |
| `imuTask` | Low | 1 KB | 空转 | ❌ 待实现 IMU |
| `uart7RecTask` | Low | 2 KB | 未创建（注释） | QLP over UART7 |
| `uart7TransTask` | Normal | 1 KB | 未创建（注释） | QLP 转发 |
| `ledTask` | Low | 512 B | 未创建（注释） | 指示灯流水 |

FreeRTOS 配置：Tick 1000 Hz，堆 32 KB，抢占式 + 时间片，56 级优先级。

### 消息队列（`App/Src/app_tasks.c`）

| 队列 | 深度 | 元素 | 状态 |
|---|---|---|---|
| `g_uart8RxQueue` | 4 | `AppUart8RxChunk` (300B) | 启用 |
| `g_beepFrameQueue` | 4 | `AppFrameMessage` (264B) | 启用 |
| `g_can1TxQueue` | 8 | `AppCanTxMessage` | 启用 |
| `g_can2TxQueue` | 8 | `AppCanTxMessage` | 启用 |
| `g_uart7RxQueue` | 4 | `AppUart7RxChunk` (64B) | 未创建 |
| `g_uart7FrameQueue` | 4 | `AppFrameMessage` | 未创建 |

### 核心数据流

**控制下行（主链路）**

1. 上位机 → micro-ROS agent → UART7 → `microRosTask`
2. `tau_cmd_callback`：校验 `size == 12` → 写入 `g_tau_cmd.tau[0..11]` → `valid = 1`，`update_count++`
3. `can1TransTask` / `can2TransTask` 轮询 `update_count` 变化 → 关中断快照 → RobStride 打包 → `HAL_FDCAN_AddMessageToTxFifoQ`
4. 两路 CAN 各发 6 帧，覆盖 12 个关节

**外设上行（QLP）**

1. UART8 DMA(IDLE) 收到字节块 → `HAL_UARTEx_RxEventCallback` → 投递 `g_uart8RxQueue`
2. `uart8RecTask` → `Qlp_ParseBytes` 逐字节状态机
3. 校验通过 → `Uart8Task_OnFrame` → 投递 `g_beepFrameQueue`（并按地址转发，转发路径默认禁用）
4. `beepTask` 按帧尾字节决定鸣叫时长

---

## 通信协议

### 1. QLP 串行协议（自研）

详见 [`docs/qlp_protocol_user_manual.md`](docs/qlp_protocol_user_manual.md)。

**帧格式（小端）**

```
| SOF(1) | ADDR(1) | SRC(1) | FC(1) | LEN(2, LE) | DATA(N) | CRC16(2, LE) |
```

- `SOF` = `0x23`（`#`），用于帧起点识别与**快速重同步**
- `LEN` = `DATA` 字节数（不含头尾）
- `CRC16` = CRC-16/MODBUS（poly `0xA001`, init `0xFFFF`），校验范围 `ADDR..DATA`
- 单帧最大 `DATA` = 256 B，最大帧长 = 264 B

**地址编码**：`ADDR = TYPE << 4 | ID`

| TYPE | 设备 | TYPE | 设备 |
|---|---|---|---|
| `0x0` | 广播/保留 | `0x4` | 陀螺仪 |
| `0x1` | 主控板 | `0x5` | 机械臂电机 |
| `0x2` | 底盘电机 | `0x6` | 蜂鸣器 |
| `0x3` | 遥控器 | `0x7` | 数码管 |

**功能码**：`0x00~0x7F` 控制/配置类，`0x80~0xFF` 状态/上报类。常用：

| FC | 名称 | 说明 |
|---|---|---|
| `0x01` | 通用控制 | 单设备控制 |
| `0x02` / `0x03` | 参数写入 / 读取 | 配置 |
| `0x81` | 通用状态上报 | 单设备状态 |
| `0x90` | 遥控器上报 | `mode(u8) + 4×float32 + key(u8)`，LEN=18 |
| `0x91` | 陀螺仪上报 | 姿态角/角速度 |
| `0xA0` | 电机状态上报 | 位置/速度/电流 |

**地址分配**：主控 `0x11`、遥控器 `0x31`、陀螺仪 `0x41`、底盘电机 `0x21~0x2C`、机械臂 `0x51~0x55`、蜂鸣器 `0x61/0x62`、数码管 `0x71`。

### 2. RobStride 电机 CAN 协议

物理层：**Classic CAN，1 Mbps，扩展帧（29-bit ID）**，8 字节 DLC。

**运控帧（COMM_TYPE = `0x01`）**

```
Ext ID : | COMM_TYPE(8) | TORQUE(16, BE) | MOTOR_ID(8) |
         |    0x01      |   torque_u16   |    id       |

Data[8]: | ANGLE(2,BE) | SPEED(2,BE) | KP(2,BE) | KD(2,BE) |
```

浮点→16bit 线性映射（`MotorCan_FloatToUint`），越界钳位：

| 量 | 最小值 | 最大值 |
|---|---|---|
| Torque (T) | -17.0 | 17.0 |
| Position (P) | -12.5 | 12.5 |
| Velocity (V) | -44.0 | 44.0 |
| Kp | 0.0 | 500.0 |
| Kd | 0.0 | 5.0 |

当前为**纯力矩模式**：`angle = speed = kp = kd = 0`。

**使能帧（COMM_TYPE = `0x03`）**

```
Ext ID : | 0x03 | MASTER_ID(8) | MOTOR_ID(8) |     Data[8] = 0x00 × 8
```

`MASTER_ID` 固定为 `0x00`（`CAN_ROBSTRITE_MASTER_ID`）。

### 3. micro-ROS 接口

| 项目 | 值 |
|---|---|
| 节点名 | `mcu_tau_subscriber` |
| 传输层 | UART7 + DMA（`rmw_uros_set_custom_transport`） |
| 话题 | `/real_joint_effort_controller/commands` |
| 类型 | `std_msgs/msg/Float64MultiArray` |
| 数据长度 | 必须为 **12**，否则丢弃 |
| 语义 | 12 个关节的目标力矩（Nm），依次映射到两路 CAN 的电机 |
| 分配器 | 自定义 FreeRTOS 分配器（`microros_allocate` 等） |

---

## 目录结构

```
quadruped_communication/
├── App/
│   ├── Inc/
│   │   ├── app_tasks.h        # 任务/队列/数据结构声明
│   │   ├── qlp_protocol.h     # QLP 协议定义与工具函数
│   │   └── robstrite.h        # RobStride 电机 CAN 定义
│   └── Src/
│       ├── app_tasks.c        # 队列创建、CAN/UART DMA 启动、RxEvent 回调
│       ├── microros_task.c    # micro-ROS 节点、订阅、tau_cmd_callback
│       ├── microros_port.c    # 裸机系统调用垫片（usleep/gettimeofday/原子符号）
│       ├── qlp_protocol.c     # QLP 打包/解析/CRC16
│       ├── robstrite.c        # RobStride 打包（运控/使能）
│       ├── uart7_task.c       # QLP over UART7 收发与 CAN 路由
│       ├── uart8_task.c       # QLP over UART8 收帧与分发
│       ├── can1_task.c        # FDCAN1 力矩下发
│       ├── can2_task.c        # FDCAN2 力矩下发
│       ├── beep_task.c        # 蜂鸣器控制
│       ├── led_task.c         # 指示灯（默认禁用）
│       └── imu_task.c         # IMU 占位
├── Core/                      # CubeMX 生成：HAL/RTOS 初始化
│   ├── Inc/                   # main.h, fdcan.h, usart.h, dma.h, gpio.h, FreeRTOSConfig.h …
│   └── Src/                   # main.c, fdcan.c, usart.c, dma.c, gpio.c, freertos.c …
├── Drivers/                   # STM32H7xx HAL + CMSIS（厂商代码）
├── Middlewares/               # FreeRTOS Kernel + CMSIS-RTOS2
├── micro_ros_stm32cubemx_utils/   # micro-ROS 静态库与移植源（见「已知问题」）
├── docs/
│   └── qlp_protocol_user_manual.md
├── dog.ioc                    # STM32CubeMX 工程配置
├── CMakeLists.txt             # 交叉编译构建脚本
├── STM32H743IITX_FLASH.ld     # Flash 链接脚本
└── STM32H743IITX_RAM.ld       # RAM 链接脚本
```

---

## 构建

### 依赖

- `arm-none-eabi-gcc` 工具链（C11，支持 `-mcpu=cortex-m7`）
- CMake ≥ 3.30
- **micro-ROS 静态库与移植源**：`micro_ros_stm32cubemx_utils/`
  （`libmicroros.a`、`extra_sources/*.c`、`microros_transports/dma_transport.c`）

> ⚠️ 当前仓库中 `micro_ros_stm32cubemx_utils/` 为**空目录**，缺少上述依赖，**源码状态下无法完成链接**。请先补齐该目录（见「已知问题 #1」）再构建。

### 编译

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

产物位于 `build/`：

- `quadruped_ant.elf`
- `quadruped_ant.hex`
- `quadruped_ant.bin`

`CMAKE_BUILD_TYPE` 支持 `Debug`（`-Og -g`）/ `Release`（`-Ofast`）/ `MinSizeRel`（`-Os`）。

---

## 烧录与调试

**STM32CubeProgrammer（SWD）**

```bash
STM32_Programmer_CLI -c port=SWD -w build/quadruped_ant.hex -v -rst
```

**OpenOCD**

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
        -c "program build/quadruped_ant.elf verify reset exit"
```

**运行调试（LED 编码）**

`microros_task.c` 在初始化各阶段失败时以 LED 闪烁模式提示：

| 现象 | 含义 |
|---|---|
| LED0 周期闪 | 分配器设置失败 |
| LED1 周期闪 | `rclc_support_init` 失败 |
| LED2 周期闪 | `rclc_node_init_default` 失败 |
| LED3 周期闪 | `rclc_executor_init` 失败 |
| LED0+LED1 同时闪 | `rclc_executor_add_subscription` 失败 |
| LED3 快闪 | 每次成功下发一轮力矩 |

> 若 micro-ROS agent 未启动，节点会持续重试连接，属正常现象。

---

## 已知问题

> 在二次开发前请务必阅读本节。

1. **🔴 缺少 micro-ROS 依赖，无法构建。**
   `CMakeLists.txt` 引用 `micro_ros_stm32cubemx_utils/` 下的静态库与源文件，但该目录为空。需重新拉取
   [micro_ros_stm32cubemx](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils) 并生成 `libmicroros.a`。

2. **🔴 CAN2 力矩索引错误，`tau[6..11]` 被丢弃。**
   `can1_task.c` 与 `can2_task.c` 均以 `tau_start_index = 0` 发送，导致 CAN2 复用了 CAN1 的前 6 个力矩，
   后 6 个关节的力矩被静默忽略。CAN2 应使用起始索引 `6U`。

3. **🟠 CAN 接收通路未实现。** 全工程无 `HAL_FDCAN_RxFifo0/1Callback`，`can1RecTask`/`can2RecTask` 为空转，
   未配置 CAN 滤波器 → **无法获取电机位置/速度/电流反馈**，无法闭环与状态上报。

4. **🟠 IMU 未接入。** `imuTask` 为空壳，QLP FC `0x91` 未实现。

5. **🟠 UART7 的 QLP 链路默认禁用。** `g_uart7RxQueue` / `g_uart7FrameQueue` 未创建，
   `AppTasks_StartUart7DmaReceive()` 为空函数，`uart7RecTask`/`uart7TransTask` 未创建，
   相关路由（QLP addr → CAN1/CAN2）为死代码。

6. **🟡 UART7 职责冲突。** UART7 同时被 micro-ROS 传输层与 `uart7_task.c` 的 QLP 收发使用；
   若两者同时启用会互相干扰，需明确复用仲裁或改用预留的 UART5/USART1。

7. **🟡 `g_tau_cmd` 存在撕裂读风险。** 生产者在非临界区逐个写 `tau[]` 后才自增 `update_count`，
   消费者「判断 + 关中断拷贝」非原子，可能读到半新半旧数组。建议改用双缓冲或消息队列传递。

8. **🟡 队列满时静默丢帧。** 所有 `osMessageQueuePut` 使用零超时且忽略返回值，无丢帧统计。

9. **🟡 工程卫生。**
   - `cmake-build-debug/` 为 Windows(CLion) 遗留产物（cache 指向 `D:\_fw123\quadruped_ant`），且包含 `.elf/.hex/.bin`。
   - `CMakeLists.txt` 使用 `file(GLOB_RECURSE)`，新增源文件不会自动触发重新配置。
   - 编译选项 `-mfpu=fpv4-sp-d16`（单精度）与 M7 的 fpv5 双精度不符；而 `tau` 使用 `double`，全部走软浮点。
   - `can1_task.c` 与 `can2_task.c` 高度重复，可抽取公共模块。
   - `freertos.c` 中的 `__weak` 任务桩已被 `App/` 强符号覆盖，属冗余代码。

---

## 路线图

- [ ] 补齐 `micro_ros_stm32cubemx_utils` 依赖，恢复可构建状态
- [ ] 修复 CAN2 力矩起始索引（`0U → 6U`）
- [ ] 实现 CAN 接收：滤波器 + `RxFifo` 回调 + 电机状态解码
- [ ] QLP FC `0xA0` 电机状态上报
- [ ] IMU 接入与 FC `0x91` 上报
- [ ] 明确 UART7 复用策略（micro-ROS 专用 / QLP 迁移至 UART5）
- [ ] 用双缓冲/队列替换 `g_tau_cmd` 的 `volatile` + 临界区方案
- [ ] 抽取 CAN1/CAN2 公共发送模块，消除重复
- [ ] 清理构建产物与死代码，补充 CI

---

## 相关文档

- **QLP 串行通信协议用户手册** — [`docs/qlp_protocol_user_manual.md`](docs/qlp_protocol_user_manual.md)
- STM32H743 参考手册 / 数据手册（ST 官方）
- micro-ROS 文档 — <https://micro.ros.org/>
- RobStride 电机 CAN 协议（厂商资料）

---

## 许可证

本仓库暂未声明开源许可证。`Drivers/`（STM32 HAL/CMSIS）与 `Middlewares/`（FreeRTOS）分别遵循其原始许可，
参见对应目录下的 `LICENSE` 文件。
