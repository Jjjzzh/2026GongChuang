#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"
#include "tim.h"

/**********************************************************
*** 舵机驱动模块
*** TIM4 CH1=PD12, CH2=PD13, CH3=PD14
*** PWM 50Hz, 周期20ms, 1us分辨率
**********************************************************/

/* ==================== 云台舵机 (PD14 / TIM4_CH3) ==================== */
#define PTZ_PULSE_MIN     500     /* 物理最小脉宽 (us)       */
#define PTZ_PULSE_MAX     2500    /* 物理最大脉宽 (us)       */
#define PTZ_PULSE_MID     1500    /* 中位脉宽 (us)           */
#define PTZ_ANGLE_MIN     55      /* 软件角度下限             */
#define PTZ_ANGLE_MAX     235     /* 软件角度上限             */

/* ==================== 夹爪舵机 (PD12 / TIM4_CH1) ==================== */
#define SERVO_GRAB_PULSE_MIN  500
#define SERVO_GRAB_PULSE_MAX  2500
#define SERVO_GRAB_PULSE_MID  1500
#define SERVO_GRAB_ANGLE_MAX  180

/* ==================== 转盘舵机 (PD13 / TIM4_CH2) ==================== */
#define TURNTABLE_PULSE_MIN   500
#define TURNTABLE_PULSE_MAX   2500
#define TURNTABLE_ANGLE_MIN   0
#define TURNTABLE_ANGLE_MAX   240
#define TURNTABLE_POS_HOME    120    /* 初始位置 */
#define TURNTABLE_POS_0       0
#define TURNTABLE_POS_120     120
#define TURNTABLE_POS_240     240

/* 舵机通道选择 */
typedef enum {
    SERVO_GRAB     = 1,   /* PD12 / TIM4_CH1 - 夹爪舵机     */
    SERVO_TURNTABLE = 2,  /* PD13 / TIM4_CH2 - 转盘舵机     */
    SERVO_PTZ      = 3    /* PD14 / TIM4_CH3 - 云台舵机     */
} ServoCh_t;

void Servo_Init(void);
void Servo_SetPulse(ServoCh_t ch, uint16_t pulse_us);
void Servo_SetAngle(ServoCh_t ch, uint16_t angle);

/* 正弦控速: 平滑过渡到目标角度, duration_ms 控制全程耗时 */
void Servo_MoveTo(ServoCh_t ch, uint16_t target_angle, uint16_t duration_ms);
void Servo_Update(void);       /* 主循环中周期性调用 */
uint8_t Servo_IsDone(ServoCh_t ch); /* 查询是否到位 */

#endif

