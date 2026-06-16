# 工创 2026 — STM32 工程源码说明

本目录为 STM32F407 主控端代码, 基于 MDK-ARM (Keil) 开发, 负责底盘运动控制、物料抓取/放置、传感器数据采集及 Jetson 通信。

---

## 模块概览

### 底盘与运动控制

| 文件 | 说明 |
|---|---|
| **Chassis.c/h** | 麦克纳姆轮底盘控制。4 轮逆运动学分解, 支持速度模式 (连续运动)、位置模式 (余弦 S 形速度曲线定距移动、斜线移动) 和旋转模式 (Yaw 闭环原地转弯)。依赖 Emm_V5 驱动电机、HWT101CT 反馈 Yaw 角。 |
| **Emm_V5.c/h** | Emm_V5.0 闭环步进驱动器通信协议 (张大头闭环伺服)。USART6 串口收发, 中断接收帧解析。提供速度/位置/回零/使能/同步运动全部指令, 以及 PID 参数、回零参数等配置函数。下半部分为 MMCL 指令队列缓存函数。 |
| **pid.c/h** | 位置式 PID 闭环控制。Yaw 角专用接口, 含角度最短路径 wrap、积分分离、抗积分饱和。TIM2 提供 50ms 控制节拍, PID 输出驱动底盘旋转。 |
| **Servo.c/h** | 三路舵机 PWM 驱动 (TIM4, 50Hz)。通道: 夹爪 (PD12)、转盘 (PD13)、云台 (PD14)。支持立即角度控制和余弦 S 形正弦控速平滑过渡 (`Servo_MoveTo` + `Servo_Update`)。 |

### 物料操作

| 文件 | 说明 |
|---|---|
| **Grab.c/h** | 物料抓取/暂存模块。Z 轴升降 (Emm_V5 ID=5, 绝对位置模式) + 夹爪开合 (Servo)。非阻塞状态机 (`Grab_Process`), 支持物料盘抓取、加工区抓取、暂存三种序列。上电自动执行 Z 轴碰撞回零。 |
| **Place.c/h** | 物料放置模块。从暂存点取回物料 → 搬运到放置点释放。非阻塞状态机, 包含 PTZ 对准、Z 轴升降、夹爪开合等并行步骤。支持普通放置 (`Place_Start`) 和码垛放置 (`Place_StartPallet`)。 |
| **Position.c/h** | 物料视觉定位模块。与 Jetson 配合实现迭代闭环对准: 请求识别 → 获取图像坐标 → 计算像素偏差 → 换算 mm → 底盘移动 → 重新检测直到收敛。支持物料盘对准 (`AlignToTray`)、色环对准 (`AlignToRing`) 和车身三轴校准 (`BodyCalib`)。含发散检测 (误差扩大 2× 自动终止) 和超时保护。 |

### 通信与传感器

| 文件 | 说明 |
|---|---|
| **jetsondata.c/h** | Jetson 通信模块 (USART2, 双向)。STM32 发送任务码 (扫码/识别物料盘/识别物料/识别色环/车身校准), Jetson 返回结果帧 (二维码/物料盘坐标/抓取指令/色环坐标/校准数据)。DMA 循环接收 + 帧解析状态机。 |
| **hwt101ct.c/h** | HWT101CT 姿态传感器驱动 (UART4, WIT Normal Protocol)。解析 11 字节协议帧, 输出加速度/角速度/角度/磁场/四元数。支持 Yaw 软件置零、输出内容/回传速率/波特率配置。移植自 JY901 wit_c_sdk。 |
| **bt04.c/h** | DX-BT04 蓝牙模块 (USART5, 经典蓝牙 2.0)。无线调试透传, 支持 Byte/Bytes/String/Printf 四种发送方式, 环形队列接收。 |
| **Screen.c/h** | TJC 串口屏通信 (USART3, 仅发送)。TJC 协议 (0xFF 0xFF 0xFF 终止符)。支持设置文本控件 (`SetTxt`)、数值控件 (`SetVal`) 和二维码显示 (6 位数字格式化为 "123+456")。 |

### 基础设施

| 文件 | 说明 |
|---|---|
| **community.c/h** | 通用串口通信模块。封装 DMA 循环接收 → 环形队列 + 阻塞发送的完整流程。bt04、hwt101ct、jetsondata 均基于此模块, 避免各模块重复实现收发代码。支持 Printf 格式化输出。 |
| **delay_us.c/h** | 微秒级延时。基于 SysTick 计数器轮询实现 (STM32F407 @168MHz), 单次最大 900μs, 自动分段。 |

---

## 依赖关系

```
main
 ├─ Chassis ────── Emm_V5, hwt101ct
 ├─ Grab ───────── Emm_V5, Servo
 ├─ Place ──────── Emm_V5, Servo, Grab.h (复用宏定义)
 ├─ Position ───── jetsondata, Chassis
 ├─ pid ────────── hwt101ct, Chassis
 ├─ jetsondata ─── community
 ├─ hwt101ct ───── community
 ├─ bt04 ───────── community
 ├─ Screen ─────── (USART3, HAL)
 ├─ Servo ──────── (TIM4, HAL)
 └─ delay_us ───── (SysTick)
```

## 硬件接口分配

| 外设 | 引脚 | 用途 |
|---|---|---|
| USART1 | PA9/PA10 | Emm_V5 步进电机 (4路) |
| USART2  | PA2/PA3 | Jetson 通信 |
| USART3 | PB10/PB11 | TJC 串口屏 |
| UART4  | PA0/PA1 | HWT101CT 姿态传感器 |
| UART5  | PC12/PD2 | DX-BT04 蓝牙模块 |
| TIM2   | — | PID 50ms 控制节拍 |
| TIM4 CH1 | PD12 | 夹爪舵机 |
| TIM4 CH2 | PD13 | 转盘舵机 |
| TIM4 CH3 | PD14 | 云台舵机 |

## 关键设计

- **非阻塞状态机**: Grab 和 Place 使用 `HAL_GetTick` 计时 + 轮询到位标志, 不阻塞主循环, 可随时响应紧急停止。
- **余弦 S 形速度曲线**: Chassis 位置模式、Servo 正弦控速均使用 `(1-cos(π·t/T))/2` 曲线, 实现零起零止平滑运动。
- **同步运动**: Emm_V5 的 snF (同步标志) + `Synchronous_motion` 使四轮同步触发, 5ms 间隔防粘包。
- **迭代闭环定位**: Position 模块每轮移动后重新请求 Jetson 检测, 直到像素误差 ≤ 容差 (3px), 最多 5 轮, 含发散保护。
- **DMA 循环接收 + 环形队列**: community 模块统一实现, 零 CPU 开销持续接收数据, 在主循环中批量处理。
