/**
  ******************************************************************************
  * @file    hwt101ct.c
  * @brief   HWT101CT 姿态传感器驱动实现 (WIT Normal Protocol)
  * @note
  *          - 使用 UART4 DMA1_Stream2 循环接收 (由 CubeMX 配置)
  *          - 底层收发委托给 community 通用串口模块
  *          - 协议帧: 0x55 + Type + 8字节数据 + Checksum (11字节)
  *          - 在主循环中调用 HWT101CT_Process() 处理 DMA 数据
  *          - 移植自 JY901 wit_c_sdk, 适配 STM32F4 HAL + DMA
  * @ref  http://wit-motion.cn / https://github.com/WITMOTION/WitStandardProtocol_JY901
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hwt101ct.h"
#include <string.h>

/* ============================================================
 * 全局实例定义
 * ============================================================ */
HWT101CT_t hwt101ct;

/* ============================================================
 * 内部静态函数
 * ============================================================ */

/**
 * @brief  计算累加和校验 (Normal Protocol)
 * @param  data  数据指针
 * @param  len   数据长度
 * @return 累加和的低 8 位
 */
static uint8_t HWT_Checksum(const uint8_t *data, uint32_t len)
{
    uint8_t sum = 0;
    for (uint32_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief  将 2 字节小端数据转为有符号 int16_t
 * @note   Normal 协议中使用小端序 (低字节在前)
 */
static inline int16_t HWT_ReadInt16(const uint8_t *p)
{
    uint16_t val = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    if (val >= 32768) {
        return (int16_t)(val - 65536);
    }
    return (int16_t)val;
}

/**
 * @brief  解析 11 字节 Normal 协议帧, 更新传感器数据
 * @param  frame  完整的 11 字节帧 (已校验通过)
 */
static void HWT_ParseFrame(const uint8_t *frame)
{
    uint8_t  type = frame[1];
    int16_t  d0, d1, d2, d3;

    d0 = HWT_ReadInt16(&frame[2]);
    d1 = HWT_ReadInt16(&frame[4]);
    d2 = HWT_ReadInt16(&frame[6]);
    d3 = HWT_ReadInt16(&frame[8]);

    switch (type) {
    case HWT_FRAME_TIME:   /* 0x50  时间 */
        /* d0=年月, d1=日时, d2=分秒, d3=毫秒 — 一般不用 */
        break;

    case HWT_FRAME_ACC:    /* 0x51  加速度 + 温度 */
        hwt101ct.data.acc[0] = (float)d0 / 32768.0f * 16.0f;
        hwt101ct.data.acc[1] = (float)d1 / 32768.0f * 16.0f;
        hwt101ct.data.acc[2] = (float)d2 / 32768.0f * 16.0f;
        hwt101ct.data.temp    = d3;
        break;

    case HWT_FRAME_GYRO:   /* 0x52  角速度 */
        hwt101ct.data.gyro[0] = (float)d0 / 32768.0f * 2000.0f;
        hwt101ct.data.gyro[1] = (float)d1 / 32768.0f * 2000.0f;
        hwt101ct.data.gyro[2] = (float)d2 / 32768.0f * 2000.0f;
        break;

    case HWT_FRAME_ANGLE:  /* 0x53  角度 + 版本号 */
        hwt101ct.data.angle[0] = (float)d0 / 32768.0f * 180.0f;  /* Roll  */
        hwt101ct.data.angle[1] = (float)d1 / 32768.0f * 180.0f;  /* Pitch */
        hwt101ct.data.angle[2] = (float)d2 / 32768.0f * 180.0f;  /* Yaw   */
        hwt101ct.data.version   = (uint16_t)d3;
        break;

    case HWT_FRAME_MAG:    /* 0x54  磁场 */
        hwt101ct.data.mag[0] = d0;
        hwt101ct.data.mag[1] = d1;
        hwt101ct.data.mag[2] = d2;
        break;

    case HWT_FRAME_QUATER: /* 0x59  四元数 */
        hwt101ct.data.quat[0] = (float)d0 / 32768.0f;
        hwt101ct.data.quat[1] = (float)d1 / 32768.0f;
        hwt101ct.data.quat[2] = (float)d2 / 32768.0f;
        hwt101ct.data.quat[3] = (float)d3 / 32768.0f;
        break;

    /* 以下帧类型根据需要可自行扩展 */
    case HWT_FRAME_PORT:   /* 0x55  D0-D3 */
    case HWT_FRAME_PRESS:  /* 0x56  气压/高度 */
    case HWT_FRAME_GPS:    /* 0x57  GPS */
    case HWT_FRAME_VELOCITY: /* 0x58  速度/GPS高度 */
    case HWT_FRAME_GSA:    /* 0x5A  GPS 精度 */
    case HWT_FRAME_REGVALUE: /* 0x5F  寄存器应答 */
        break;

    default:
        break;
    }

    hwt101ct.data.frame_count++;
}

/**
 * @brief  帧解析状态机: 喂入一个字节, 内部处理帧同步/校验/解析
 * @param  byte  从串口接收到的一个字节
 *
 * Normal 协议帧格式 (11 字节):
 *   [0]: 0x55  帧头
 *   [1]: Type  数据类型
 *   [2-9]: 4 路 int16 数据 (小端)
 *   [10]: Checksum (前 10 字节累加和低 8 位)
 */
static void HWT_FeedByte(uint8_t byte)
{
    /* 帧头不对 → 丢弃首字节, 左移窗口 */
    if (hwt101ct.rx_idx == 0 && byte != HWT_FRAME_HEADER) {
        return;
    }

    hwt101ct.rx_buf[hwt101ct.rx_idx++] = byte;

    if (hwt101ct.rx_idx >= HWT_FRAME_LEN) {
        hwt101ct.rx_idx = 0;

        /* 校验 */
        if (HWT_Checksum(hwt101ct.rx_buf, 10) == hwt101ct.rx_buf[10]) {
            HWT_ParseFrame(hwt101ct.rx_buf);
        } else {
            hwt101ct.data.error_count++;
        }
    }
}

/* ============================================================
 * 公开 API 实现
 * ============================================================ */

/**
 * @brief  初始化 HWT101CT 传感器
 * @param  huart  UART 句柄 (如 &huart4)
 * @param  rsw    输出内容选择 (如 HWT_RSW_IMU)
 * @param  rrate  回传速率 (如 HWT_RRATE_10HZ)
 *
 * 调用时机: main() 中, HAL 初始化 + 外设初始化完成后
 *
 * 执行流程:
 *   1. 初始化 CommPort_t (启动 DMA 循环接收)
 *   2. 向传感器发送解锁 + 输出内容 + 回传速率配置命令
 */
void HWT101CT_Init(UART_HandleTypeDef *huart, uint16_t rsw, uint8_t rrate)
{
    /* 清零全局实例 */
    memset(&hwt101ct, 0, sizeof(hwt101ct));

    hwt101ct.rsw   = (uint8_t)rsw;
    hwt101ct.rrate = rrate;

    /* 初始化 DMA 接收 */
    CommPort_Init(&hwt101ct.port, huart);

    /* 等待 DMA 就绪 */
    HAL_Delay(10);

    /* 配置传感器输出内容 */
    HWT101CT_SetContent(rsw);
    HAL_Delay(5);

    /* 配置回传速率 */
    HWT101CT_SetRate(rrate);
    HAL_Delay(5);

    /* 保存参数到 Flash (掉电不丢失) */
    HWT101CT_SendCommand(HWT_REG_SAVE, HWT_SAVE_PARAM);
}

/**
 * @brief  将当前朝向设为 Yaw 0° (软件置零)
 * @note
 *   传感器有备份电池, 断电后 Yaw 保留上一次的值.
 *   解决方案: 上电后传感器稳定 → 调用此函数记录当前 raw Yaw 为偏移量,
 *   之后 HWT101CT_GetYaw() 自动扣除偏移, 始终返回相对于置零时刻的角度.
 */
void HWT101CT_ZeroYaw(void)
{
    hwt101ct.yaw_offset = hwt101ct.data.angle[2];
}

/**
 * @brief  处理 DMA 接收数据 + 解析帧
 * @note   在主循环 while(1) 中周期性调用 (如 100Hz+)
 *
 * 执行流程:
 *   1. CommPort_Process(): 将 DMA 缓冲数据转入环形队列
 *   2. 从环形队列逐字节读取, 喂入帧解析状态机
 */
void HWT101CT_Process(void)
{
    /* 将 DMA 新数据转入环形队列 */
    CommPort_Process(&hwt101ct.port);

    /* 逐字节读入帧解析器 */
    while (CommPort_Available(&hwt101ct.port) > 0) {
        uint8_t byte = CommPort_ReadByte(&hwt101ct.port);
        HWT_FeedByte(byte);
    }
}

/* ============================================================
 * 数据读取函数
 * ============================================================ */

float HWT101CT_GetAccX(void)  { return hwt101ct.data.acc[0]; }
float HWT101CT_GetAccY(void)  { return hwt101ct.data.acc[1]; }
float HWT101CT_GetAccZ(void)  { return hwt101ct.data.acc[2]; }
float HWT101CT_GetGyroX(void) { return hwt101ct.data.gyro[0]; }
float HWT101CT_GetGyroY(void) { return hwt101ct.data.gyro[1]; }
float HWT101CT_GetGyroZ(void) { return hwt101ct.data.gyro[2]; }
float HWT101CT_GetRoll(void)  { return hwt101ct.data.angle[0]; }
float HWT101CT_GetPitch(void) { return hwt101ct.data.angle[1]; }
float HWT101CT_GetYaw(void)   { return hwt101ct.data.angle[2] - hwt101ct.yaw_offset; }
int16_t HWT101CT_GetTemp(void) { return hwt101ct.data.temp; }

void HWT101CT_GetAcc(float acc[3])   { memcpy(acc,   hwt101ct.data.acc,   3 * sizeof(float)); }
void HWT101CT_GetGyro(float gyro[3]) { memcpy(gyro,  hwt101ct.data.gyro,  3 * sizeof(float)); }
void HWT101CT_GetAngle(float angle[3]) { memcpy(angle, hwt101ct.data.angle, 3 * sizeof(float)); }
void HWT101CT_GetQuat(float quat[4]) { memcpy(quat,  hwt101ct.data.quat,  4 * sizeof(float)); }

/* ============================================================
 * 命令发送函数
 * ============================================================ */

/**
 * @brief  通过 Normal 协议写寄存器
 * @param  reg_addr  寄存器地址
 * @param  data      寄存器值
 *
 * Normal 协议写寄存器帧: [0xFF, 0xAA, RegAddr, DataL, DataH]
 */
void HWT101CT_SendCommand(uint8_t reg_addr, uint16_t data)
{
    uint8_t cmd[5];
    cmd[0] = 0xFF;
    cmd[1] = 0xAA;
    cmd[2] = reg_addr;
    cmd[3] = (uint8_t)(data & 0xFF);        /* DataL */
    cmd[4] = (uint8_t)((data >> 8) & 0xFF); /* DataH */

    CommPort_SendBytes(&hwt101ct.port, cmd, 5);
}

/**
 * @brief  通过 Normal 协议读寄存器
 * @param  reg_addr  起始寄存器地址
 *
 * Normal 协议读寄存器帧: [0xFF, 0xAA, 0x27, RegAddrL, RegAddrH]
 * 传感器应答: 0x5F 类型数据包
 */
void HWT101CT_ReadRegister(uint8_t reg_addr)
{
    uint8_t cmd[5];
    cmd[0] = 0xFF;
    cmd[1] = 0xAA;
    cmd[2] = 0x27;
    cmd[3] = reg_addr;           /* RegAddrL */
    cmd[4] = 0x00;               /* RegAddrH (寄存器地址 < 256 时恒为 0) */

    CommPort_SendBytes(&hwt101ct.port, cmd, 5);
}

/* ============================================================
 * 快速配置命令
 * ============================================================ */

/**
 * @brief  设置输出内容 (RSW)
 * @param  rsw  输出内容位掩码 (如 HWT_RSW_IMU)
 * @note   写入 RSW 寄存器前需要先解锁 (KEY = 0xB588)
 *
 * 常用组合:
 *   HWT_RSW_ANGLE_ONLY — 只输出角度
 *   HWT_RSW_IMU        — 加速度 + 角速度 + 角度 + 磁场
 *   HWT_RSW_ALL        — 以上 + 四元数
 */
void HWT101CT_SetContent(uint16_t rsw)
{
    HWT101CT_SendCommand(HWT_REG_KEY, HWT_KEY_UNLOCK);
    HAL_Delay(1);
    HWT101CT_SendCommand(HWT_REG_RSW, rsw);
    hwt101ct.rsw = (uint8_t)rsw;
}

/**
 * @brief  设置回传速率 (RRATE)
 * @param  rrate  回传速率 (如 HWT_RRATE_10HZ = 0x06)
 */
void HWT101CT_SetRate(uint8_t rrate)
{
    HWT101CT_SendCommand(HWT_REG_KEY, HWT_KEY_UNLOCK);
    HAL_Delay(1);
    HWT101CT_SendCommand(HWT_REG_RRATE, rrate);
    hwt101ct.rrate = rrate;
}

/**
 * @brief  设置 UART 波特率 (BAUD)
 * @param  baud_index  波特率索引 (如 HWT_BAUD_115200 = 6)
 * @note   修改后传感器立即以新波特率工作,
 *         需要同步修改 MCU 端 UART 配置
 */
void HWT101CT_SetBaud(uint8_t baud_index)
{
    HWT101CT_SendCommand(HWT_REG_KEY, HWT_KEY_UNLOCK);
    HAL_Delay(1);
    HWT101CT_SendCommand(HWT_REG_BAUD, baud_index);
}
