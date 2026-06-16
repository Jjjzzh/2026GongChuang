/**
  ******************************************************************************
  * @file    jetsondata.h
  * @brief   Jetson 通信模块 (USART2, 双向)
  * @note
  *          - 硬件接口: USART2 (PA2=TX, PA3=RX) + DMA1_Stream5 循环接收
  *          - 波特率: 115200-8-N-1
  *          - 帧格式: [0x66][任务码][参数][校验位(STM32端不处理)]
  *          - 底层收发通过 community 通用串口模块
  ******************************************************************************
  */
#ifndef __JETSONDATA_H__
#define __JETSONDATA_H__

#include "community.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 常量
 * ============================================================ */
#define JETSON_FRAME_HEADER   0x66    /* 帧头 */
#define JETSON_FRAME_TAIL_SEND  0xFF  /* 发送帧尾 (STM32→Jetson) */
#define JETSON_FRAME_TAIL_RECV  0x99  /* 接收帧尾 (Jetson→STM32) */

#define JETSON_RECV_BUF_SIZE  16     /* 接收帧缓冲 */

/* ============================================================
 * 发送任务码 (STM32→Jetson)
 * ============================================================ */
#define JETSON_TASK_SCAN      0x01    /* 扫码任务 */
#define JETSON_TASK_TRAY      0x02    /* 识别物料盘 */
#define JETSON_TASK_MATERIAL  0x03    /* 识别物料 (按颜色) */
#define JETSON_TASK_RING      0x04    /* 识别色环 */
#define JETSON_TASK_CALIB     0x05    /* 车身校准 (色环XY+Z) */

/* ============================================================
 * 任务参数定义
 * ============================================================ */
/* --- 0x03 识别物料: 颜色参数 --- */
#define JETSON_MATERIAL_RED    0x01    /* 红色物料 */
#define JETSON_MATERIAL_GREEN  0x02    /* 绿色物料 */
#define JETSON_MATERIAL_BLUE   0x03    /* 蓝色物料 */

/* --- 0x04 识别色环: 颜色参数 --- */
#define JETSON_RING_TASK_RED    0x01   /* 红色色环 */
#define JETSON_RING_TASK_GREEN  0x02   /* 绿色色环 */
#define JETSON_RING_TASK_BLUE   0x03   /* 蓝色色环 */

/* ============================================================
 * 接收命令码 (Jetson→STM32) 与 Jetson 端 STM32ResponseSender 配对
 * ============================================================ */
#define JETSON_RECV_QRCODE     0x01    /* 二维码数据 */
#define JETSON_RECV_TRAY       0x02    /* 物料盘中心坐标 */
#define JETSON_RECV_GRAB       0x03    /* 抓取指令 */
#define JETSON_RECV_RING       0x04    /* 目标色环坐标 */
#define JETSON_RECV_RING_CALIB 0x05    /* 色环校准数据 */

/* 色环颜色 */
#define JETSON_RING_RED   0x01
#define JETSON_RING_GREEN 0x02
#define JETSON_RING_BLUE  0x03

/* ============================================================
 * 接收帧长度 (含帧头帧尾)
 * ============================================================ */
#define JETSON_LEN_QRCODE      8      /* [0x66,0x01, ch0~ch5, 0x99] */
#define JETSON_LEN_TRAY        7      /* [0x66,0x02, X_H,X_L, Y_H,Y_L, 0x99] */
#define JETSON_LEN_GRAB        4      /* [0x66,0x03, 0x01, 0x99] */
#define JETSON_LEN_RING        7      /* [0x66,0x04, X_H,X_L, Y_H,Y_L, 0x99] */
#define JETSON_LEN_RING_CALIB  9      /* [0x66,0x05, X_H,X_L, Y_H,Y_L, Z_H,Z_L, 0x99] */

/* ============================================================
 * 数据类型标志 (对应 5 种接收命令)
 * ============================================================ */
#define JETSON_FLAG_QRCODE      0x0001
#define JETSON_FLAG_TRAY        0x0002
#define JETSON_FLAG_GRAB        0x0004
#define JETSON_FLAG_RING        0x0008
#define JETSON_FLAG_RING_CALIB  0x0010

/* ============================================================
 * 接收数据结构体
 * ============================================================ */
typedef struct {
    /* --- 二维码 --- */
    uint8_t  qr_chars[6];

    /* --- 物料盘 --- */
    uint16_t tray_x;
    uint16_t tray_y;

    /* --- 抓取 --- */
    uint8_t  grab_flag;

    /* --- 目标色环 --- */
    uint16_t ring_x;
    uint16_t ring_y;

    /* --- 色环校准 --- */
    uint16_t calib_x;
    uint16_t calib_y;
    int16_t  calib_z;       /* 右Y - 左Y, 有符号 */

    /* --- 状态 --- */
    uint16_t update_flags;  /* JETSON_FLAG_xxx 组合 */
    uint32_t frame_count;
    uint32_t error_count;
} JetsonRxData_t;

/* ============================================================
 * 全局实例 (声明)
 * ============================================================ */
extern CommPort_t jetson_port;
extern JetsonRxData_t jetson_rx;

/* ============================================================
 * API 函数声明
 * ============================================================ */

/* --- 初始化 --- */
void JetsonData_Init(UART_HandleTypeDef *huart);

/* --- 数据处理 (在主循环中调用) --- */
void JetsonData_Process(void);

/* --- 发送函数 (STM32→Jetson) --- */
void JetsonData_SendTask(uint8_t task_code, uint8_t param);
void JetsonData_SendBytes(const uint8_t *data, uint16_t len);

/* --- 接收原始字节 --- */
uint16_t JetsonData_Available(void);
uint8_t  JetsonData_ReadByte(void);
uint16_t JetsonData_ReadBytes(uint8_t *buf, uint16_t max_len);

/* --- 接收解析后数据 --- */
uint8_t  JetsonData_IsUpdated(uint16_t flag);
void     JetsonData_ClearFlags(void);

/* 二维码 */
const uint8_t* JetsonData_GetQRChars(void);

/* 物料盘 */
uint16_t JetsonData_GetTrayX(void);
uint16_t JetsonData_GetTrayY(void);

/* 抓取 */
uint8_t  JetsonData_IsGrab(void);

/* 目标色环 */
uint16_t JetsonData_GetRingX(void);
uint16_t JetsonData_GetRingY(void);

/* 色环校准 */
uint16_t JetsonData_GetCalibX(void);
uint16_t JetsonData_GetCalibY(void);
int16_t  JetsonData_GetCalibZ(void);

#ifdef __cplusplus
}
#endif

#endif /* __JETSONDATA_H__ */
