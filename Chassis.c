/**
  ******************************************************************************
  * @file    Chassis.c
  * @brief   麦克纳姆轮底盘控制模块实现
  * @note
  *          - 速度模式: 运动学分解 → 4 轮同步触发
  *          - 位置模式: 余弦 S 形速度曲线 → 逐段发速度指令
  *          - 旋转模式: S 曲线 + Yaw 闭环, 内部带 HWT101CT_Process
  *          - 指令 snF=true 扣住, sync 同步触发, 5ms 间隔防粘包
  *          - 最低速度 10 RPM, 克服静摩擦
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

/* Includes ------------------------------------------------------------------*/
#include "Chassis.h"
#include "Emm_V5.h"
#include "hwt101ct.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================
 * 内部辅助
 * ============================================================ */

/* 电机安装方向补偿: M1/M3 正装, M2/M4 反装 */
static const int8_t Chassis_Dir[5] = { 0, 1, -1, 1, -1 };

/* 起步助推 (RPM), 线性衰减到 0, 克服静摩擦 */
#define CHASSIS_BOOST_RPM  15.0f
#define CHASSIS_BOOST_END  0.15f   /* 助推在 15% 进度处归零 */

/**
 * @brief  发速度指令给单个电机 (snF=true, 扣住等 sync)
 * @param  signed_rpm  正值=底盘前方, 负值=底盘后方 (补偿前)
 */
static void Chassis_SendVel(uint8_t addr, float signed_rpm, uint8_t accel)
{
    signed_rpm *= (float)Chassis_Dir[addr];
    if (signed_rpm >= 0) {
        Emm_V5_Vel_Control(addr, 0, (uint16_t)(signed_rpm), accel, true);
    } else {
        Emm_V5_Vel_Control(addr, 1, (uint16_t)(-signed_rpm), accel, true);
    }
}

/**
 * @brief  发 4 轮速度指令 (扣住) → sync 同步触发
 */
static void Chassis_Send4Wheels(const WheelSpeed_t *w, float rpm, uint8_t accel)
{
    Chassis_SendVel(CHASSIS_ADDR_M1, w->m1 * rpm, accel);  HAL_Delay(5);
    Chassis_SendVel(CHASSIS_ADDR_M2, w->m2 * rpm, accel);  HAL_Delay(5);
    Chassis_SendVel(CHASSIS_ADDR_M3, w->m3 * rpm, accel);  HAL_Delay(5);
    Chassis_SendVel(CHASSIS_ADDR_M4, w->m4 * rpm, accel);
    HAL_Delay(2);
    Emm_V5_Synchronous_motion(0x00);
}

/* ============================================================
 * 公开 API 实现
 * ============================================================ */

/**
 * @brief  麦克纳姆轮归一化逆运动学 (X 型辊子布局)
 *
 *     M1 (前左) = Vx - Vy - Vω
 *     M2 (前右) = Vx + Vy + Vω
 *     M3 (后左) = Vx + Vy - Vω
 *     M4 (后右) = Vx - Vy + Vω
 */
void Chassis_Kinematics(const ChassisVel_t *vel, WheelSpeed_t *wheel)
{
    if (vel == NULL || wheel == NULL) return;

    wheel->m1 = vel->vx - vel->vy - vel->vw;
    wheel->m2 = vel->vx + vel->vy + vel->vw;
    wheel->m3 = vel->vx + vel->vy - vel->vw;
    wheel->m4 = vel->vx - vel->vy + vel->vw;
}

/**
 * @brief  速度模式: 底盘连续运动
 * @param  vx/vy/vw  运动分量 (RPM)
 * @param  speed     已废弃, 传 0
 * @param  accel     加减速 (0=直接启动)
 */
void Chassis_VelMove(int vx, int vy, int vw, uint16_t speed, uint8_t accel)
{
    (void)speed;

    ChassisVel_t vel;
    vel.vx = (float)vx;
    vel.vy = (float)vy;
    vel.vw = (float)vw;

    WheelSpeed_t w;
    Chassis_Kinematics(&vel, &w);
    Chassis_Send4Wheels(&w, 1.0f, accel);
}

/**
 * @brief  位置模式: 定距移动 (余弦 S 形, 最低 10 RPM)
 * @param  distance  距离 (mm), 正=前/左, 负=后/右
 * @param  axis      CHASSIS_DIR_X 或 CHASSIS_DIR_Y
 * @param  v_max     峰值转速 (RPM)
 * @param  dt_ms     控制周期 (ms)
 */
void Chassis_PosMove(float distance, uint8_t axis, uint16_t v_max, uint16_t dt_ms)
{
    if (v_max == 0 || dt_ms == 0) return;

    float dir = (distance >= 0) ? 1.0f : -1.0f;
    float D   = (distance >= 0) ? distance : -distance;
    if (D < 0.1f) return;

    float v_linear_max = v_max / 60.0f * CHASSIS_WHEEL_CIRC_MM;
    float T = 2.0f * D / v_linear_max;

    ChassisVel_t vel;
    vel.vx = (axis == CHASSIS_DIR_X) ? dir : 0.0f;
    vel.vy = (axis == CHASSIS_DIR_Y) ? dir : 0.0f;
    vel.vw = 0.0f;

    WheelSpeed_t w;
    Chassis_Kinematics(&vel, &w);

    float dt = dt_ms / 1000.0f;
    for (float t = 0; t < T; t += dt) {
        float progress = t / T;
        float ratio = (1.0f - cosf(2.0f * M_PI * progress)) / 2.0f;
        float v_cur = ratio * v_max;
        /* 起步助推: 线性衰减, 之后 S 曲线接管 */
        if (progress < CHASSIS_BOOST_END) {
            v_cur += CHASSIS_BOOST_RPM * (1.0f - progress / CHASSIS_BOOST_END);
        }
        Chassis_Send4Wheels(&w, v_cur, 0);
        HAL_Delay(dt_ms);
    }

    Chassis_Send4Wheels(&w, 0.0f, 0);
}

/**
 * @brief  斜线模式: 同时沿 X/Y 移动 (余弦 S 形, 最低 10 RPM)
 * @param  dx       X轴距离 (mm), 正=前, 负=后
 * @param  dy       Y轴距离 (mm), 正=左, 负=右
 * @param  v_max    峰值转速 (RPM)
 * @param  dt_ms    控制周期 (ms)
 */
void Chassis_PosMoveDiag(float dx, float dy, uint16_t v_max, uint16_t dt_ms)
{
    if (v_max == 0 || dt_ms == 0) return;

    float D = sqrtf(dx * dx + dy * dy);
    if (D < 0.1f) return;

    /* 单位方向向量 */
    float ux = dx / D;
    float uy = dy / D;

    float v_linear_max = v_max / 60.0f * CHASSIS_WHEEL_CIRC_MM;
    float T = 2.0f * D / v_linear_max;

    ChassisVel_t vel;
    vel.vx = ux;
    vel.vy = uy;
    vel.vw = 0.0f;

    WheelSpeed_t w;
    Chassis_Kinematics(&vel, &w);

    float dt = dt_ms / 1000.0f;
    for (float t = 0; t < T; t += dt) {
        float progress = t / T;
        float ratio = (1.0f - cosf(2.0f * M_PI * progress)) / 2.0f;
        float v_cur = ratio * v_max;
        if (progress < CHASSIS_BOOST_END) {
            v_cur += CHASSIS_BOOST_RPM * (1.0f - progress / CHASSIS_BOOST_END);
        }
        Chassis_Send4Wheels(&w, v_cur, 0);
        HAL_Delay(dt_ms);
    }

    Chassis_Send4Wheels(&w, 0.0f, 0);
}

/**
 * @brief  原地旋转 (余弦 S 形 + Yaw 闭环, 最低 10 RPM)
 * @param  angle_deg  目标角度 (°), 正=逆时针, 负=顺时针
 * @param  v_max      峰值转速 (RPM)
 * @param  dt_ms      控制周期 (ms)
 */
void Chassis_Rotate(float angle_deg, uint16_t v_max, uint16_t dt_ms)
{
    if (v_max == 0 || dt_ms == 0) return;

    float total = (angle_deg >= 0) ? angle_deg : -angle_deg;
    if (total < 0.5f) return;

    float dir = (angle_deg >= 0) ? 1.0f : -1.0f;

    ChassisVel_t vel;
    vel.vx = 0.0f;
    vel.vy = 0.0f;
    vel.vw = dir;

    WheelSpeed_t w;
    Chassis_Kinematics(&vel, &w);

    float traveled = 0.0f;
    float yaw_prev = HWT101CT_GetYaw();

    while (traveled < total) {

        HWT101CT_Process();

        float yaw_cur = HWT101CT_GetYaw();
        float delta   = yaw_cur - yaw_prev;

        while (delta >  180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;

        if ((dir > 0 && delta > 0) || (dir < 0 && delta < 0)) {
            traveled += (delta >= 0) ? delta : -delta;
        }
        if (traveled > total) traveled = total;

        yaw_prev = yaw_cur;

        float alpha = traveled / total;
        float ratio = (1.0f - cosf(2.0f * M_PI * alpha)) / 2.0f;
        float v_cur = ratio * v_max;
        /* 起步助推: 线性衰减, 之后 S 曲线接管 */
        if (alpha < CHASSIS_BOOST_END) {
            v_cur += CHASSIS_BOOST_RPM * (1.0f - alpha / CHASSIS_BOOST_END);
        }

        Chassis_Send4Wheels(&w, v_cur, 0);
        HAL_Delay(dt_ms);
    }

    Chassis_Send4Wheels(&w, 0.0f, 0);
}

/**
 * @brief  立即停止 4 轮 (snF=true + sync)
 */
void Chassis_Stop(void)
{
    Emm_V5_Stop_Now(CHASSIS_ADDR_M1, true);  HAL_Delay(5);
    Emm_V5_Stop_Now(CHASSIS_ADDR_M2, true);  HAL_Delay(5);
    Emm_V5_Stop_Now(CHASSIS_ADDR_M3, true);  HAL_Delay(5);
    Emm_V5_Stop_Now(CHASSIS_ADDR_M4, true);
    HAL_Delay(2);
    Emm_V5_Synchronous_motion(0x00);
}
