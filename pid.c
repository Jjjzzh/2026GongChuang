/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID 闭环控制模块实现
  * @note
  *          - TIM2 中断只置标志位, 实际 PID 计算在主循环中执行
  *            (避免在 ISR 中阻塞发送电机指令)
  *          - Yaw 角误差自动处理 ±180° 环绕 (最短路径)
  *          - 位置式 PID + 积分分离 + 抗积分饱和
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "hwt101ct.h"
#include "Chassis.h"
#include <math.h> 

/* ============================================================
 * 全局 PID 实例
 * ============================================================ */
PID_t pid_yaw;

/* 控制节拍标志: TIM2 ISR 置 1, 主循环处理并清零 */
volatile uint8_t pid_yaw_tick = 0;

/* 控制周期 (秒) */
#define PID_YAW_DT   0.05f    /* 50ms */

/* 到位死区: |v|<1RPM 时直接停止, 防止小值抖动 */
#define PID_YAW_DEADBAND_RPM  1.0f

/* Yaw 到位容差 (°): 误差在此范围内认为已到位, 停止输出 */
#define PID_YAW_TOLERANCE_DEG 1.0f

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

/**
 * @brief  计算 Yaw 角最短路径误差 (±180° wrap)
 * @param  target   目标角度 (°)
 * @param  current  当前角度 (°)
 * @return 最短路径误差, 范围 [-180, 180]
 *
 * 示例: target=170°, current=-170°
 *       err = 170 - (-170) = 340 → 340 - 360 = -20° (正确, 逆时针转20°)
 */
static float PID_YawError(float target, float current)
{
    float err = target - current;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/* ============================================================
 * 公开 API 实现
 * ============================================================ */

/**
 * @brief  PID 结构体初始化
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max, float ki_thresh)
{
    if (pid == NULL) return;

    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->target     = 0.0f;
    pid->feedback   = 0.0f;
    pid->error      = 0.0f;
    pid->error_prev = 0.0f;
    pid->integral   = 0.0f;
    pid->output     = 0.0f;
    pid->out_min    = out_min;
    pid->out_max    = out_max;
    pid->ki_thresh  = ki_thresh;
    pid->enabled    = 1;
}

/**
 * @brief  PID 单次计算 (位置式)
 */
float PID_Compute(PID_t *pid, float target, float feedback, float dt)
{
    /* 暂停模式: 清空状态, 输出 0 */
    if (!pid->enabled) {
        pid->integral   = 0.0f;
        pid->error_prev = 0.0f;
        pid->output     = 0.0f;
        return 0.0f;
    }

    pid->target   = target;
    pid->feedback = feedback;
    pid->error    = target - feedback;

    /* ---- P 项 ---- */
    float p_term = pid->kp * pid->error;

    /* ---- I 项 (积分分离) ---- */
    if (fabsf(pid->error) <= pid->ki_thresh) {
        pid->integral += pid->error * dt;
    } else {
        pid->integral = 0.0f;   /* 大偏差时清积分, 防止饱和 */
    }
    float i_term = pid->ki * pid->integral;

    /* ---- D 项 (微分先行于偏差, 避免设定值突变引起的微分冲击) ---- */
    float d_term = 0.0f;
    if (dt > 1e-6f) {
        d_term = pid->kd * (pid->error - pid->error_prev) / dt;
    }
    pid->error_prev = pid->error;

    /* ---- 合成输出 ---- */
    pid->output = p_term + i_term + d_term;

    /* ---- 输出限幅 + 抗积分饱和 (back-calculation) ---- */
    if (pid->output > pid->out_max) {
        /* 超上限: 回退积分项, 钳位输出 */
        float excess = pid->output - pid->out_max;
        if (pid->ki > 1e-6f && fabsf(pid->integral) > 1e-6f) {
            pid->integral -= excess / pid->ki;
        }
        pid->output = pid->out_max;
    } else if (pid->output < pid->out_min) {
        float excess = pid->output - pid->out_min;
        if (pid->ki > 1e-6f && fabsf(pid->integral) > 1e-6f) {
            pid->integral -= excess / pid->ki;
        }
        pid->output = pid->out_min;
    }

    return pid->output;
}

/**
 * @brief  重置 PID 内部状态
 */
void PID_Reset(PID_t *pid)
{
    if (pid == NULL) return;
    pid->error      = 0.0f;
    pid->error_prev = 0.0f;
    pid->integral   = 0.0f;
    pid->output     = 0.0f;
}

/**
 * @brief  使能 / 暂停 PID
 */
void PID_Enable(PID_t *pid, uint8_t en)
{
    if (pid == NULL) return;
    if (!en) {
        PID_Reset(pid);
    }
    pid->enabled = en;
}

/**
 * @brief  在线修改 PID 参数
 */
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/* ============================================================
 * Yaw 角专用接口
 * ============================================================ */

/**
 * @brief  初始化 Yaw 角 PID (使用默认参数)
 */
void PID_YawInit(void)
{
    PID_Init(&pid_yaw,
             PID_YAW_KP, PID_YAW_KI, PID_YAW_KD,
             PID_YAW_OUT_MIN, PID_YAW_OUT_MAX,
             PID_YAW_KI_THRESH);

    /* 初始化目标为当前角度, 避免启动瞬间跳变 */
    pid_yaw.target = HWT101CT_GetYaw();
}

/**
 * @brief  设置 Yaw 目标角度
 */
void PID_SetYawTarget(float target_deg)
{
    /* 规范化到 [-180, 180] */
    while (target_deg >  180.0f) target_deg -= 360.0f;
    while (target_deg < -180.0f) target_deg += 360.0f;

    pid_yaw.target  = target_deg;
    pid_yaw.enabled = 1;
}

/**
 * @brief  Yaw 角 PID 循环 (由主循环在 pid_yaw_tick 置位时调用)
 *
 *   流程:
 *     ① 读 HWT101CT Yaw 角
 *     ② 计算最短路径角度误差 (处理 ±180° wrap)
 *     ③ PID 计算 → 旋转速度 vw (RPM)
 *     ④ 输出到 Chassis_VelMove
 *
 *   到位死区:
 *     误差 < PID_YAW_TOLERANCE_DEG 且 |vw| < PID_YAW_DEADBAND_RPM → 停止
 */
void PID_YawLoop(void)
{
    /* ---- 1. 读当前 Yaw ---- */
    float yaw = HWT101CT_GetYaw();

    /* ---- 2. 计算最短路径误差 ---- */
    float err = PID_YawError(pid_yaw.target, yaw);

    /* ---- 3. PID 计算 ---- */
    float vw = PID_Compute(&pid_yaw, pid_yaw.target, yaw, PID_YAW_DT);
    /* 用实际误差覆盖 PID 内部的 raw error (Yaw 需要 wrap 处理) */
    pid_yaw.error = err;

    /* ---- 4. 到位判断: 误差小 + 输出小 → 停止 ---- */
    float abs_err = fabsf(err);
    float abs_vw  = fabsf(vw);

    if (abs_err <= PID_YAW_TOLERANCE_DEG && abs_vw < PID_YAW_DEADBAND_RPM) {
        /* 已到位, 停止旋转 */
        Chassis_VelMove(0, 0, 0, 0, 0);
        pid_yaw.output = 0.0f;
        return;
    }

    /* ---- 5. 输出: vx=0, vy=0, vw=PID输出 ---- */
    int vw_int = (int)(vw);
    Chassis_VelMove(0, 0, vw_int, 0, 0);
}

/* ============================================================
 * TIM2 中断回调 (HAL 框架)
 * ============================================================ */

/**
 * @brief  TIM 周期中断回调
 * @note   HAL_TIM_IRQHandler → HAL_TIM_PeriodElapsedCallback
 *         所有定时器共用此回调, 需判断 htim 实例
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        pid_yaw_tick = 1;
    }
}
