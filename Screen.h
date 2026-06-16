/**
  ******************************************************************************
  * @file    Screen.h
  * @brief   TJC 串口屏通信模块 (USART3, 仅发送)
  * @note
  *          - 串口屏只用于显示，STM32 只发不收，无需 DMA
  *          - 协议: 命令帧以 0xFF 0xFF 0xFF 结尾
  *          - CubeMX 中只需配置 USART3 (TX 即可，RX 可省略)
  *          - 波特率 115200-8-N-1
  ******************************************************************************
  */
#ifndef __SCREEN_H__
#define __SCREEN_H__

#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * API 函数声明
 * ============================================================ */

void Screen_Init(UART_HandleTypeDef *huart);

void Screen_SendString(const char *str);              /* 原始字符串 + 0xFF*3     */
void Screen_SetTxt(const char *obj, const char *txt);  /* obj.txt="txt"           */
void Screen_SetVal(const char *obj, int val);          /* obj.val=num             */
void Screen_ShowQR(const uint8_t qr_chars[6]);         /* 显示 "123+456" 到 QR.txt */

#ifdef __cplusplus
}
#endif

#endif /* __SCREEN_H__ */
