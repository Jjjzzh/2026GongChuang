/**
  ******************************************************************************
  * @file    community.c
  * @brief   通用串口通信模块实现
  * @note
  *          - DMA 循环接收数据 → 环形队列
  *          - 阻塞发送, 支持 printf 格式化
  *          - 在主循环中调用 CommPort_Process() 处理 DMA 数据
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "community.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 * 内部静态函数
 * ============================================================ */

/**
 * @brief  向环形队列写入一个字节
 */
static void CommPort_QueuePush(CommPort_t *port, uint8_t byte)
{
    if (port->rx_count < COMM_QUEUE_SIZE) {
        port->rx_queue[port->rx_head] = byte;
        port->rx_head++;
        if (port->rx_head >= COMM_QUEUE_SIZE) {
            port->rx_head = 0;
        }
        port->rx_count++;
    }
}

/**
 * @brief  从环形队列读取一个字节
 * @return 队列为空返回 0
 */
static uint8_t CommPort_QueuePop(CommPort_t *port)
{
    uint8_t byte = 0;
    if (port->rx_count > 0) {
        byte = port->rx_queue[port->rx_tail];
        port->rx_tail++;
        if (port->rx_tail >= COMM_QUEUE_SIZE) {
            port->rx_tail = 0;
        }
        port->rx_count--;
    }
    return byte;
}

/* ============================================================
 * 公开 API 实现
 * ============================================================ */

/**
 * @brief  初始化通用串口
 * @param  port   CommPort_t 指针
 * @param  huart  UART 句柄 (如 &huart5)
 */
void CommPort_Init(CommPort_t *port, UART_HandleTypeDef *huart)
{
    port->huart       = huart;
    port->hdma_rx     = huart->hdmarx;
    port->dma_rd_idx  = 0;
    port->rx_head     = 0;
    port->rx_tail     = 0;
    port->rx_count    = 0;

    /* 清零缓冲区 */
    for (uint16_t i = 0; i < COMM_DMA_BUF_SIZE; i++) {
        port->rx_dma_buf[i] = 0;
    }
    for (uint16_t i = 0; i < COMM_QUEUE_SIZE; i++) {
        port->rx_queue[i] = 0;
    }

    /* 启动 DMA 循环接收 (仅当该串口配置了 DMA 时) */
    if (port->hdma_rx != NULL) {
        HAL_UART_Receive_DMA(port->huart,
                             port->rx_dma_buf,
                             COMM_DMA_BUF_SIZE);
    }
}

/**
 * @brief  处理 DMA 接收到的新数据, 写入环形队列
 * @note   在主循环中周期性调用
 */
void CommPort_Process(CommPort_t *port)
{
    uint32_t wr_idx;

    /* 无 DMA 的端口 (如纯发送) 跳过 */
    if (port->hdma_rx == NULL) return;

    /* 计算 DMA 当前写入位置 */
    wr_idx = COMM_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(port->hdma_rx);

    /* 将新字节全部写入环形队列 */
    while (port->dma_rd_idx != wr_idx) {
        CommPort_QueuePush(port, port->rx_dma_buf[port->dma_rd_idx]);
        port->dma_rd_idx++;
        if (port->dma_rd_idx >= COMM_DMA_BUF_SIZE) {
            port->dma_rd_idx = 0;
        }
    }
}

/* ============================================================
 * 发送函数
 * ============================================================ */

/**
 * @brief  发送单个字节 (阻塞)
 */
void CommPort_SendByte(CommPort_t *port, uint8_t byte)
{
    HAL_UART_Transmit(port->huart, &byte, 1, 10);
}

/**
 * @brief  发送字节数组 (阻塞)
 */
void CommPort_SendBytes(CommPort_t *port, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return;
    HAL_UART_Transmit(port->huart, (uint8_t *)data, len, 1000);
}

/**
 * @brief  发送字符串 (阻塞)
 */
void CommPort_SendString(CommPort_t *port, const char *str)
{
    if (str == NULL) return;
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0) return;
    HAL_UART_Transmit(port->huart, (uint8_t *)str, len, 1000);
}

/**
 * @brief  格式化打印到串口 (printf 风格)
 * @note   使用 vsnprintf, 最大单次输出 256 字节
 *
 * 使用示例:
 *   CommPort_Printf(&port, "Yaw=%.2f\r\n", yaw);
 */
void CommPort_Printf(CommPort_t *port, const char *fmt, ...)
{
    static char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    CommPort_SendString(port, buf);
}

/* ============================================================
 * 接收函数
 * ============================================================ */

/**
 * @brief  返回环形队列中可读的字节数
 */
uint16_t CommPort_Available(CommPort_t *port)
{
    return port->rx_count;
}

/**
 * @brief  从环形队列读取一个字节
 * @return 队列为空返回 0
 */
uint8_t CommPort_ReadByte(CommPort_t *port)
{
    return CommPort_QueuePop(port);
}

/**
 * @brief  从环形队列读取多个字节
 * @param  port     CommPort_t 指针
 * @param  buf      输出缓冲区
 * @param  max_len  最大读取字节数
 * @return 实际读取的字节数
 */
uint16_t CommPort_ReadBytes(CommPort_t *port, uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    while (count < max_len && port->rx_count > 0) {
        buf[count++] = CommPort_QueuePop(port);
    }
    return count;
}

/**
 * @brief  读取一行 (以 '\n' 结尾)
 * @param  port     CommPort_t 指针
 * @param  buf      输出缓冲区
 * @param  max_len  最大长度 (含 '\0')
 * @return 读取的字节数 (不含 '\0')
 * @note   行尾的 \r\n 会被替换为 '\0'
 */
uint16_t CommPort_ReadLine(CommPort_t *port, uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    while (count < max_len - 1 && port->rx_count > 0) {
        uint8_t ch = CommPort_QueuePop(port);
        if (ch == '\n') {
            /* 去掉前一个 \r */
            if (count > 0 && buf[count - 1] == '\r') {
                count--;
            }
            break;
        }
        buf[count++] = ch;
    }
    buf[count] = '\0';
    return count;
}
