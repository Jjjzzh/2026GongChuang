/**
  ******************************************************************************
  * @file    Chassis.h
  * @brief   麦克纳姆轮底盘控制模块
  * @note
  *          - 4 轮麦克纳姆轮，归一化运动学
  *          - 速度模式: 直接发 Emm_V5_Vel_Control
  *          - 位置模式: 余弦 S 形速度曲线，不依赖 Emm_V5 内部梯形
  *          - 旋转模式: 原地转弯, Yaw 闭环修正
  *
  *          轮组布局 (俯视):
  *
  *              前
  *          M1 ─── M2      M1=前左, M2=前右
  *          │   X   │      M3=后左, M4=后右
  *          M3 ─── M4
  *              后
  ******************************************************************************
  */
#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 电机地址 (Emm_V5 CAN ID, 按实际接线修改)
 * ============================================================ */
#define CHASSIS_ADDR_M1     1   /* 前左 Front-Left    */
#define CHASSIS_ADDR_M2     2   /* 前右 Front-Right   */
#define CHASSIS_ADDR_M3     3   /* 后左 Rear-Left     */
#define CHASSIS_ADDR_M4     4   /* 后右 Rear-Right    */

/* ============================================================
 * 运动轴 (位置模式用)
 * ============================================================ */
#define CHASSIS_DIR_X       0   /* 前/后 */
#define CHASSIS_DIR_Y       1   /* 左/右 */

/* ============================================================
 * 底盘物理参数
 *
 *  每mm脉冲 = (细分 × 步数) / (π × 轮径mm × 传动比)
 *          = (16 × 200) / (3.1416 × 76.2 × 1) ≈ 13.37
 *
 *  轮径 3英寸 = 76.2mm,  16细分,  1.8°步进电机(200步/转),  直驱
 * ============================================================ */
#define CHASSIS_MICROSTEP       16
#define CHASSIS_STEP_PER_REV    200
#define CHASSIS_WHEEL_DIA_MM    (3.0f * 25.4f)

#define CHASSIS_PULSE_PER_MM  \
    ((float)(CHASSIS_MICROSTEP * CHASSIS_STEP_PER_REV) / (3.1416f * CHASSIS_WHEEL_DIA_MM))

#define CHASSIS_WHEEL_CIRC_MM   (3.1416f * CHASSIS_WHEEL_DIA_MM)  /* 轮周长 mm */

/* ============================================================
 * 数据类型
 * ============================================================ */
typedef struct {
    float vx;   /* 前向 (+前)  */
    float vy;   /* 横向 (+左)  */
    float vw;   /* 旋转  (+逆时针) */
} ChassisVel_t;

typedef struct {
    float m1;   /* 前左轮 */
    float m2;   /* 前右轮 */
    float m3;   /* 后左轮 */
    float m4;   /* 后右轮 */
} WheelSpeed_t;

/* ============================================================
 * API
 * ============================================================ */

/* 运动学分解 */
void Chassis_Kinematics(const ChassisVel_t *vel, WheelSpeed_t *wheel);

/* 速度模式: 连续运动 */
void Chassis_VelMove(int vx, int vy, int vw, uint16_t speed, uint8_t accel);

/* 位置模式: 定距移动, 余弦 S 形速度曲线 */
void Chassis_PosMove(float distance, uint8_t axis, uint16_t v_max, uint16_t dt_ms);

/* 斜线模式: 同时沿 X/Y 移动, 余弦 S 形 */
void Chassis_PosMoveDiag(float dx, float dy, uint16_t v_max, uint16_t dt_ms);

/* 旋转模式: 原地转弯, 余弦 S 形 + Yaw 闭环 */
void Chassis_Rotate(float angle_deg, uint16_t v_max, uint16_t dt_ms);

/* 停止 */
void Chassis_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H__ */
