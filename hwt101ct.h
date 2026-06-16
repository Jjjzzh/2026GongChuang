/**
  ******************************************************************************
  * @file    hwt101ct.h
  * @brief   HWT101CT 姿态传感器驱动 (WIT Normal Protocol)
  * @note
  *          - 硬件接口: UART4 (PA0=TX, PA1=RX) + DMA1_Stream2 循环接收
  *          - 协议: WIT Normal Protocol (0x55 帧头, 11字节数据包)
  *          - 波特率: 默认 115200-8-N-1
  *          - 底层收发通过 community 通用串口模块
  *          - 移植自 JY901 wit_c_sdk, 适配 STM32F4 HAL + DMA
  * @ref  http://wit-motion.cn
  ******************************************************************************
  */
#ifndef __HWT101CT_H__
#define __HWT101CT_H__

#include "community.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 寄存器地址定义 (与 wit_c_sdk REG.h 完全一致)
 * ============================================================ */
#define HWT_REG_SAVE        0x00
#define HWT_REG_CALSW       0x01
#define HWT_REG_RSW         0x02
#define HWT_REG_RRATE       0x03
#define HWT_REG_BAUD        0x04
#define HWT_REG_AXOFFSET    0x05
#define HWT_REG_AYOFFSET    0x06
#define HWT_REG_AZOFFSET    0x07
#define HWT_REG_GXOFFSET    0x08
#define HWT_REG_GYOFFSET    0x09
#define HWT_REG_GZOFFSET    0x0A
#define HWT_REG_HXOFFSET    0x0B
#define HWT_REG_HYOFFSET    0x0C
#define HWT_REG_HZOFFSET    0x0D
#define HWT_REG_D0MODE      0x0E
#define HWT_REG_D1MODE      0x0F
#define HWT_REG_D2MODE      0x10
#define HWT_REG_D3MODE      0x11
#define HWT_REG_IICADDR     0x1A
#define HWT_REG_LEDOFF      0x1B
#define HWT_REG_BANDWIDTH   0x1F
#define HWT_REG_GYRORANGE   0x20
#define HWT_REG_ACCRANGE    0x21
#define HWT_REG_ORIENT      0x23
#define HWT_REG_AXIS6       0x24
#define HWT_REG_VERSION     0x2E
#define HWT_REG_YYMM        0x30
#define HWT_REG_DDHH        0x31
#define HWT_REG_MMSS        0x32
#define HWT_REG_MS          0x33
#define HWT_REG_AX          0x34
#define HWT_REG_AY          0x35
#define HWT_REG_AZ          0x36
#define HWT_REG_GX          0x37
#define HWT_REG_GY          0x38
#define HWT_REG_GZ          0x39
#define HWT_REG_HX          0x3A
#define HWT_REG_HY          0x3B
#define HWT_REG_HZ          0x3C
#define HWT_REG_Roll        0x3D
#define HWT_REG_Pitch       0x3E
#define HWT_REG_Yaw         0x3F
#define HWT_REG_TEMP        0x40
#define HWT_REG_D0Status    0x41
#define HWT_REG_D1Status    0x42
#define HWT_REG_D2Status    0x43
#define HWT_REG_D3Status    0x44
#define HWT_REG_q0          0x51
#define HWT_REG_q1          0x52
#define HWT_REG_q2          0x53
#define HWT_REG_q3          0x54
#define HWT_REG_KEY         0x69
#define HWT_REG_ACCSENSOR   0x70
#define HWT_REG_CHIPIDL     0x8D
#define HWT_REG_CHIPIDH     0x8E

/* ============================================================
 * 帧类型标识 (Byte1)
 * ============================================================ */
#define HWT_FRAME_TIME      0x50
#define HWT_FRAME_ACC       0x51
#define HWT_FRAME_GYRO      0x52
#define HWT_FRAME_ANGLE     0x53
#define HWT_FRAME_MAG       0x54
#define HWT_FRAME_PORT      0x55
#define HWT_FRAME_PRESS     0x56
#define HWT_FRAME_GPS       0x57
#define HWT_FRAME_VELOCITY  0x58
#define HWT_FRAME_QUATER    0x59
#define HWT_FRAME_GSA       0x5A
#define HWT_FRAME_REGVALUE  0x5F

/* ============================================================
 * 输出内容选择 RSW (按位组合)
 * ============================================================ */
#define HWT_RSW_TIME    0x01
#define HWT_RSW_ACC     0x02
#define HWT_RSW_GYRO    0x04
#define HWT_RSW_ANGLE   0x08
#define HWT_RSW_MAG     0x10
#define HWT_RSW_PORT    0x20
#define HWT_RSW_PRESS   0x40
#define HWT_RSW_GPS     0x80
#define HWT_RSW_V       0x100
#define HWT_RSW_Q       0x200
#define HWT_RSW_GSA     0x400

/* 常用组合 */
#define HWT_RSW_IMU     (HWT_RSW_ACC | HWT_RSW_GYRO | HWT_RSW_ANGLE | HWT_RSW_MAG)
#define HWT_RSW_ANGLE_ONLY  (HWT_RSW_ANGLE)
#define HWT_RSW_ALL     (HWT_RSW_ACC | HWT_RSW_GYRO | HWT_RSW_ANGLE | HWT_RSW_MAG | HWT_RSW_Q)

/* ============================================================
 * 回传速率 RRATE
 * ============================================================ */
#define HWT_RRATE_02HZ  0x01
#define HWT_RRATE_05HZ  0x02
#define HWT_RRATE_1HZ   0x03
#define HWT_RRATE_2HZ   0x04
#define HWT_RRATE_5HZ   0x05
#define HWT_RRATE_10HZ  0x06
#define HWT_RRATE_20HZ  0x07
#define HWT_RRATE_50HZ  0x08
#define HWT_RRATE_100HZ 0x09
#define HWT_RRATE_125HZ 0x0A
#define HWT_RRATE_200HZ 0x0B
#define HWT_RRATE_ONCE  0x0C
#define HWT_RRATE_NONE  0x0D

/* ============================================================
 * 波特率索引 BAUD
 * ============================================================ */
#define HWT_BAUD_4800    1
#define HWT_BAUD_9600    2
#define HWT_BAUD_19200   3
#define HWT_BAUD_38400   4
#define HWT_BAUD_57600   5
#define HWT_BAUD_115200  6
#define HWT_BAUD_230400  7
#define HWT_BAUD_460800  8
#define HWT_BAUD_921600  9

/* ============================================================
 * 其他常量
 * ============================================================ */
#define HWT_FRAME_LEN       11   /* Normal 协议数据包固定长度 */
#define HWT_FRAME_HEADER    0x55 /* 帧头 */
#define HWT_KEY_UNLOCK      0xB588
#define HWT_SAVE_PARAM      0x00
#define HWT_SAVE_SWRST      0xFF

/* 校准模式 */
#define HWT_CAL_NORMAL      0x00
#define HWT_CAL_GYROACC     0x01
#define HWT_CAL_MAG         0x02
#define HWT_CAL_MAGMM       0x07
#define HWT_CAL_REFANGLE    0x08
#define HWT_CAL_MAG2STEP    0x09
#define HWT_CAL_HEXAHEDRON  0x12

/* ============================================================
 * 传感器数据结构
 * ============================================================ */
typedef struct {
    float    acc[3];           /* 加速度 (g)    */
    float    gyro[3];          /* 角速度 (°/s)  */
    float    angle[3];         /* 角度 (°)      */
    int16_t  mag[3];           /* 磁场 (uT)     */
    int16_t  temp;             /* 温度          */
    float    quat[4];          /* 四元数        */
    uint16_t version;          /* 固件版本      */

    uint32_t frame_count;      /* 有效帧计数    */
    uint32_t error_count;      /* 错误帧计数    */
} HWT101CT_Data_t;

typedef struct {
    CommPort_t      port;      /* DMA 接收端口           */
    HWT101CT_Data_t data;      /* 解析后的传感器数据      */
    uint8_t         rsw;       /* 输出内容配置 (缓存)     */
    uint8_t         rrate;     /* 回传速率 (缓存)         */
    uint8_t         rx_buf[HWT_FRAME_LEN];  /* 帧接收缓冲 */
    uint8_t         rx_idx;    /* 帧缓冲写入位置 */
    float    yaw_offset;    /* Yaw 软件置零偏移 */
} HWT101CT_t;

/* ============================================================
 * 全局实例 (声明)
 * ============================================================ */
extern HWT101CT_t hwt101ct;

/* ============================================================
 * API 函数声明
 * ============================================================ */

/* --- 初始化 --- */
void HWT101CT_Init(UART_HandleTypeDef *huart, uint16_t rsw, uint8_t rrate);

/* --- 数据处理 (在主循环中调用) --- */
void HWT101CT_Process(void);

/* --- 校准 --- */
void HWT101CT_ZeroYaw(void);  /* 将当前朝向设为 0° */

/* --- 数据读取 --- */
float    HWT101CT_GetAccX(void);
float    HWT101CT_GetAccY(void);
float    HWT101CT_GetAccZ(void);
float    HWT101CT_GetGyroX(void);
float    HWT101CT_GetGyroY(void);
float    HWT101CT_GetGyroZ(void);
float    HWT101CT_GetRoll(void);
float    HWT101CT_GetPitch(void);
float    HWT101CT_GetYaw(void);
int16_t  HWT101CT_GetTemp(void);
void     HWT101CT_GetAcc(float acc[3]);
void     HWT101CT_GetGyro(float gyro[3]);
void     HWT101CT_GetAngle(float angle[3]);
void     HWT101CT_GetQuat(float quat[4]);

/* --- 命令发送 (写寄存器 / 读寄存器) --- */
void HWT101CT_SendCommand(uint8_t reg_addr, uint16_t data);
void HWT101CT_ReadRegister(uint8_t reg_addr);

/* --- 快速配置命令 --- */
void HWT101CT_SetContent(uint16_t rsw);
void HWT101CT_SetRate(uint8_t rrate);
void HWT101CT_SetBaud(uint8_t baud_index);

#ifdef __cplusplus
}
#endif

#endif /* __HWT101CT_H__ */
