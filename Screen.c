/**
  ******************************************************************************
  * @file    Screen.c
  * @brief   TJC 串口屏通信模块实现 (USART3, 仅发送)
  * @note
  *          - 使用 HAL_UART_Transmit 阻塞发送
  *          - TJC 协议: 命令以 0xFF 0xFF 0xFF 结尾
  *
  *          使用示例:
  *            Screen_Init(&huart3);
  *            Screen_ShowQR(chars);   // QR.txt="123+456"
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Screen.h"
#include <stdio.h>
#include <string.h>

/* ============================================================
 * 内部变量
 * ============================================================ */
static UART_HandleTypeDef *screen_huart;

/* TJC 协议终止符 */
static const uint8_t TJC_TERM[3] = {0xFF, 0xFF, 0xFF};

/* ============================================================
 * 公开 API 实现
 * ============================================================ */

/**
 * @brief  初始化串口屏
 * @param  huart  USART3 句柄 (CubeMX 生成)
 */
void Screen_Init(UART_HandleTypeDef *huart)
{
    screen_huart = huart;
}

/**
 * @brief  发送原始字符串 + 0xFF 0xFF 0xFF
 * @param  str  要发送的字符串 (如 "QR.txt=\"hello\"")
 */
void Screen_SendString(const char *str)
{
    if (str == NULL || screen_huart == NULL) return;

    uint16_t len = (uint16_t)strlen(str);
    if (len == 0) return;

    HAL_UART_Transmit(screen_huart, (uint8_t *)str, len, 1000);
    HAL_UART_Transmit(screen_huart, (uint8_t *)TJC_TERM, 3, 1000);
}

/**
 * @brief  设置文本控件内容
 * @param  obj  控件名 (如 "QR", "t0")
 * @param  txt  要显示的文本
 * @note   发送: obj.txt="txt" + 0xFF*3
 */
void Screen_SetTxt(const char *obj, const char *txt)
{
    if (obj == NULL || txt == NULL) return;

    char buf[128];
    int len = snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", obj, txt);
    if (len > 0 && len < (int)sizeof(buf)) {
        Screen_SendString(buf);
    }
}

/**
 * @brief  设置数值控件
 * @param  obj  控件名 (如 "n0", "h0")
 * @param  val  数值
 * @note   发送: obj.val=val + 0xFF*3
 */
void Screen_SetVal(const char *obj, int val)
{
    if (obj == NULL) return;

    char buf[64];
    int len = snprintf(buf, sizeof(buf), "%s.val=%d", obj, val);
    if (len > 0 && len < (int)sizeof(buf)) {
        Screen_SendString(buf);
    }
}

/**
 * @brief  格式化并显示二维码数据
 * @param  qr_chars  Jetson 发来的 6 个数字字符 (如 '1','2','3','4','5','6')
 * @note   显示效果: QR.txt="123+456"
 */
void Screen_ShowQR(const uint8_t qr_chars[6])
{
    if (qr_chars == NULL) return;

    char buf[16];
    snprintf(buf, sizeof(buf), "%c%c%c+%c%c%c",
             qr_chars[0], qr_chars[1], qr_chars[2],
             qr_chars[3], qr_chars[4], qr_chars[5]);
    Screen_SetTxt("QR", buf);
}
