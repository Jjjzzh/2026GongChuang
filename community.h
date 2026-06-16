/**
  ******************************************************************************
  * @file    community.h
  * @brief   通用串口通信模块 (DMA 循环接收 + 环形队列 + 阻塞发送)
  * @note
  *          - 替代各模块重复的串口收发代码
  *          - DMA 循环接收自动存入环形队列
  *          - 支持 printf 格式化发送
  ******************************************************************************
  */
#ifndef __COMMUNITY_H__
#define __COMMUNITY_H__

#include "usart.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 常量
 * ============================================================ */
#define COMM_DMA_BUF_SIZE   256     /* DMA 循环缓冲区大小 */
#define COMM_QUEUE_SIZE     256     /* 环形接收队列大小 */

/* ============================================================
 * 通用串口句柄结构体
 * ============================================================ */
typedef struct {
    UART_HandleTypeDef *huart;                      /* UART 句柄 */
    DMA_HandleTypeDef  *hdma_rx;                    /* DMA 句柄 */
    uint8_t  rx_dma_buf[COMM_DMA_BUF_SIZE];         /* DMA 循环接收缓冲 */
    uint32_t dma_rd_idx;                            /* DMA 缓冲读取索引 */

    /* 环形接收队列 */
    uint8_t  rx_queue[COMM_QUEUE_SIZE];             /* 接收数据环形队列 */
    uint16_t rx_head;                               /* 队列写指针 */
    uint16_t rx_tail;                               /* 队列读指针 */
    uint16_t rx_count;                              /* 队列中可读字节数 */
} CommPort_t;

/* ============================================================
 * API 函数声明
 * ============================================================ */

/* --- 初始化 --- */
void CommPort_Init(CommPort_t *port, UART_HandleTypeDef *huart);

/* --- 数据处理 (在主循环中调用) --- */
void CommPort_Process(CommPort_t *port);

/* --- 发送函数 (阻塞) --- */
void CommPort_SendByte(CommPort_t *port, uint8_t byte);
void CommPort_SendBytes(CommPort_t *port, const uint8_t *data, uint16_t len);
void CommPort_SendString(CommPort_t *port, const char *str);
void CommPort_Printf(CommPort_t *port, const char *fmt, ...);

/* --- 接收函数 (从环形队列读取) --- */
uint16_t CommPort_Available(CommPort_t *port);
uint8_t  CommPort_ReadByte(CommPort_t *port);
uint16_t CommPort_ReadBytes(CommPort_t *port, uint8_t *buf, uint16_t max_len);
uint16_t CommPort_ReadLine(CommPort_t *port, uint8_t *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* __COMMUNITY_H__ */
