/**
  ******************************************************************************
  * @file    bt04.h
  * @brief   DX-BT04 蓝牙模块驱动 (经典蓝牙 2.0, AT 指令兼容)
  * @note
  *          - 硬件接口: USART5 (PC12=TX, PD2=RX) + DMA1_Stream0 循环接收
  *          - 波特率: 9600-8-N-1
  *          - 用途: 无线调试透传
  *          - 底层收发通过 community 通用串口模块
  ******************************************************************************
  */
#ifndef __BT04_H__
#define __BT04_H__

#include "community.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 全局实例 (声明)
 * ============================================================ */
extern CommPort_t bt04_port;

/* ============================================================
 * API 函数声明
 * ============================================================ */

/* --- 初始化 --- */
void BT04_Init(UART_HandleTypeDef *huart);

/* --- 数据处理 (在主循环中调用) --- */
void BT04_Process(void);

/* --- 发送函数 --- */
void BT04_SendByte(uint8_t byte);
void BT04_SendBytes(const uint8_t *data, uint16_t len);
void BT04_SendString(const char *str);
void BT04_Printf(const char *fmt, ...);

/* --- 接收函数 --- */
uint16_t BT04_Available(void);
uint8_t  BT04_ReadByte(void);
uint16_t BT04_ReadBytes(uint8_t *buf, uint16_t max_len);
uint16_t BT04_ReadLine(uint8_t *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* __BT04_H__ */
