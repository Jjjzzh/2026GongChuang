/**
  ******************************************************************************
  * @file    bt04.c
  * @brief   DX-BT04 蓝牙模块驱动实现
  * @note
  *          - 使用 USART5 DMA 循环接收 (DMA1_Stream0)
  *          - 底层收发委托给 community 通用串口模块
  *          - 支持 printf 格式化无线输出
  *          - 在主循环中调用 BT04_Process() 处理 DMA 数据
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bt04.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 * 全局实例定义
 * ============================================================ */
CommPort_t bt04_port;

/* ============================================================
 * 公开 API 实现 (薄封装)
 * ============================================================ */

/**
 * @brief  初始化蓝牙模块
 * @param  huart  USART5 句柄
 */
void BT04_Init(UART_HandleTypeDef *huart)
{
    CommPort_Init(&bt04_port, huart);
}

/**
 * @brief  处理 DMA 接收到的新数据
 * @note   在主循环中周期性调用
 */
void BT04_Process(void)
{
    CommPort_Process(&bt04_port);
}

/* --- 发送函数 --- */

void BT04_SendByte(uint8_t byte)
{
    CommPort_SendByte(&bt04_port, byte);
}

void BT04_SendBytes(const uint8_t *data, uint16_t len)
{
    CommPort_SendBytes(&bt04_port, data, len);
}

void BT04_SendString(const char *str)
{
    CommPort_SendString(&bt04_port, str);
}

void BT04_Printf(const char *fmt, ...)
{
    static char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    CommPort_SendString(&bt04_port, buf);
}

/* --- 接收函数 --- */

uint16_t BT04_Available(void)
{
    return CommPort_Available(&bt04_port);
}

uint8_t BT04_ReadByte(void)
{
    return CommPort_ReadByte(&bt04_port);
}

uint16_t BT04_ReadBytes(uint8_t *buf, uint16_t max_len)
{
    return CommPort_ReadBytes(&bt04_port, buf, max_len);
}

uint16_t BT04_ReadLine(uint8_t *buf, uint16_t max_len)
{
    return CommPort_ReadLine(&bt04_port, buf, max_len);
}
