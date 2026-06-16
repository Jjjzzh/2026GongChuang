/**
  ******************************************************************************
  * @file    jetsondata.c
  * @brief   Jetson 通信模块实现
  * @note
  *          - 使用 USART1 DMA 循环接收 (DMA2_Stream2)
  *          - 发送任务码: [0x66][任务码][参数][0xFF]
  *          - 接收结果帧: [0x66][命令码][数据...][0x99]
  *          - 底层收发委托给 community 通用串口模块
  *          - 在主循环中调用 JetsonData_Process() 处理 DMA 数据
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "jetsondata.h"
#include <string.h>

/* ============================================================
 * 全局实例定义
 * ============================================================ */
CommPort_t jetson_port;
JetsonRxData_t jetson_rx;

/* ============================================================
 * 帧解析状态 (内部)
 * ============================================================ */
static uint8_t  rx_buf[JETSON_RECV_BUF_SIZE];  /* 帧组装缓冲 */
static uint8_t  rx_idx;                         /* 缓冲索引 */
static uint8_t  rx_expected_len;                /* 当前帧期望长度 */

/* ============================================================
 * 内部静态函数声明
 * ============================================================ */
static uint8_t  JetsonData_GetFrameLen(uint8_t cmd);
static void     JetsonData_ParseFrame(uint8_t *data, uint8_t len);
static uint16_t JetsonData_ReadU16(uint8_t *data, uint8_t offset);

/* ============================================================
 * 内部函数实现
 * ============================================================ */

/**
 * @brief  根据命令码返回期望帧长度 (含帧头帧尾)
 */
static uint8_t JetsonData_GetFrameLen(uint8_t cmd)
{
    switch (cmd) {
    case JETSON_RECV_QRCODE:      return JETSON_LEN_QRCODE;
    case JETSON_RECV_TRAY:        return JETSON_LEN_TRAY;
    case JETSON_RECV_GRAB:        return JETSON_LEN_GRAB;
    case JETSON_RECV_RING:        return JETSON_LEN_RING;
    case JETSON_RECV_RING_CALIB:  return JETSON_LEN_RING_CALIB;
    default:                       return 0;
    }
}

/**
 * @brief  从字节数组读取大端序 uint16_t
 */
static uint16_t JetsonData_ReadU16(uint8_t *data, uint8_t offset)
{
    return (uint16_t)((data[offset] << 8) | data[offset + 1]);
}

/**
 * @brief  解析接收帧, 存入 jetson_rx
 * @param  data  帧数据 (不含帧头, 从命令码开始)
 * @param  len   完整帧长 (含帧头帧尾)
 */
static void JetsonData_ParseFrame(uint8_t *data, uint8_t len)
{
    uint8_t cmd = data[0];  /* 命令码 (data[0] = 帧缓冲区第1字节) */

    switch (cmd) {

    case JETSON_RECV_QRCODE:  /* [0x66,0x01, ch0~ch5, 0x99] */
        jetson_rx.qr_chars[0] = data[1];
        jetson_rx.qr_chars[1] = data[2];
        jetson_rx.qr_chars[2] = data[3];
        jetson_rx.qr_chars[3] = data[4];
        jetson_rx.qr_chars[4] = data[5];
        jetson_rx.qr_chars[5] = data[6];
        jetson_rx.update_flags |= JETSON_FLAG_QRCODE;
        break;

    case JETSON_RECV_TRAY:  /* [0x66,0x02, X_H,X_L, Y_H,Y_L, 0x99] */
        jetson_rx.tray_x = JetsonData_ReadU16(data, 1);
        jetson_rx.tray_y = JetsonData_ReadU16(data, 3);
        jetson_rx.update_flags |= JETSON_FLAG_TRAY;
        break;

    case JETSON_RECV_GRAB:  /* [0x66,0x03, 0x01, 0x99] */
        jetson_rx.grab_flag = data[1];
        jetson_rx.update_flags |= JETSON_FLAG_GRAB;
        break;

    case JETSON_RECV_RING:  /* [0x66,0x04, X_H,X_L, Y_H,Y_L, 0x99] */
        jetson_rx.ring_x     = JetsonData_ReadU16(data, 1);
        jetson_rx.ring_y     = JetsonData_ReadU16(data, 3);
        jetson_rx.update_flags |= JETSON_FLAG_RING;
        break;

    case JETSON_RECV_RING_CALIB:  /* [0x66,0x05, X_H,X_L, Y_H,Y_L, Z_H,Z_L, 0x99] */
        jetson_rx.calib_x = JetsonData_ReadU16(data, 1);
        jetson_rx.calib_y = JetsonData_ReadU16(data, 3);
        jetson_rx.calib_z = (int16_t)JetsonData_ReadU16(data, 5);
        jetson_rx.update_flags |= JETSON_FLAG_RING_CALIB;
        break;

    default:
        break;
    }
}

/* ============================================================
 * 公开 API 实现
 * ============================================================ */

/**
 * @brief  初始化 Jetson 通信模块
 * @param  huart  USART2 句柄
 */
void JetsonData_Init(UART_HandleTypeDef *huart)
{
    CommPort_Init(&jetson_port, huart);

    /* 清零帧解析状态 */
    rx_idx           = 0;
    rx_expected_len  = 0;
    memset(rx_buf, 0, sizeof(rx_buf));

    /* 清零接收数据 */
    memset(&jetson_rx, 0, sizeof(jetson_rx));
}

/**
 * @brief  处理 DMA 接收到的新数据 + 逐字节帧解析
 * @note   在主循环中周期性调用
 *
 * 帧解析流程 (与 HWT101CT_Process 同模式):
 *   CommPort_Process → 逐字节从环形队列读取 → 状态机解析
 *
 * 状态机:
 *   等待 0x66 → 读命令码 → 查表得帧长 → 收齐字节 → 验 0x99 → 解析
 */
void JetsonData_Process(void)
{
    uint8_t byte;

    /* DMA 数据 → 环形队列 */
    CommPort_Process(&jetson_port);

    /* 从队列逐字节读取并解析帧 */
    while (CommPort_Available(&jetson_port) > 0) {
        byte = CommPort_ReadByte(&jetson_port);

        /* --- 等待帧头 0x66 --- */
        if (rx_idx == 0) {
            if (byte == JETSON_FRAME_HEADER) {
                rx_buf[0] = byte;
                rx_idx = 1;
                rx_expected_len = 0;
            }
            continue;
        }

        /* --- 收到命令码 (第 2 字节), 确定帧长 --- */
        if (rx_idx == 1) {
            rx_expected_len = JetsonData_GetFrameLen(byte);
            if (rx_expected_len == 0) {
                /* 未知命令码, 丢弃, 重新找帧头 */
                rx_idx = 0;
                continue;
            }
        }

        /* --- 填充数据 --- */
        rx_buf[rx_idx] = byte;
        rx_idx++;

        /* --- 收满一帧 --- */
        if (rx_idx >= rx_expected_len) {
            /* 校验帧尾 0x99 */
            if (rx_buf[rx_expected_len - 1] == JETSON_FRAME_TAIL_RECV) {
                JetsonData_ParseFrame(&rx_buf[1], rx_expected_len);
                jetson_rx.frame_count++;
            } else {
                jetson_rx.error_count++;
            }
            rx_idx = 0;  /* 重置, 准备接收下一帧 */
        }

        /* 防止越界 */
        if (rx_idx >= JETSON_RECV_BUF_SIZE) {
            rx_idx = 0;
            jetson_rx.error_count++;
        }
    }
}

/* ============================================================
 * 发送函数 (STM32→Jetson)
 * ============================================================ */

/**
 * @brief  发送任务码到 Jetson
 * @note   帧格式: [0x66][task_code][param][0xFF]
 */
void JetsonData_SendTask(uint8_t task_code, uint8_t param)
{
    uint8_t frame[4];
    frame[0] = JETSON_FRAME_HEADER;
    frame[1] = task_code;
    frame[2] = param;
    frame[3] = JETSON_FRAME_TAIL_SEND;

    CommPort_SendBytes(&jetson_port, frame, 4);
}

/**
 * @brief  发送字节数组到 Jetson (阻塞)
 */
void JetsonData_SendBytes(const uint8_t *data, uint16_t len)
{
    CommPort_SendBytes(&jetson_port, data, len);
}

/* ============================================================
 * 接收原始字节
 * ============================================================ */

uint16_t JetsonData_Available(void)
{
    return CommPort_Available(&jetson_port);
}

uint8_t JetsonData_ReadByte(void)
{
    return CommPort_ReadByte(&jetson_port);
}

uint16_t JetsonData_ReadBytes(uint8_t *buf, uint16_t max_len)
{
    return CommPort_ReadBytes(&jetson_port, buf, max_len);
}

/* ============================================================
 * 接收解析后数据 — 访问器
 * ============================================================ */

uint8_t JetsonData_IsUpdated(uint16_t flag)
{
    return (jetson_rx.update_flags & flag) ? 1 : 0;
}

void JetsonData_ClearFlags(void)
{
    jetson_rx.update_flags = 0;
}

const uint8_t* JetsonData_GetQRChars(void)
{
    return jetson_rx.qr_chars;
}

uint16_t JetsonData_GetTrayX(void)
{
    return jetson_rx.tray_x;
}

uint16_t JetsonData_GetTrayY(void)
{
    return jetson_rx.tray_y;
}

uint8_t JetsonData_IsGrab(void)
{
    return jetson_rx.grab_flag;
}

uint16_t JetsonData_GetRingX(void)
{
    return jetson_rx.ring_x;
}

uint16_t JetsonData_GetRingY(void)
{
    return jetson_rx.ring_y;
}

uint16_t JetsonData_GetCalibX(void)
{
    return jetson_rx.calib_x;
}

uint16_t JetsonData_GetCalibY(void)
{
    return jetson_rx.calib_y;
}

int16_t JetsonData_GetCalibZ(void)
{
    return jetson_rx.calib_z;
}
