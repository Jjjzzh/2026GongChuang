/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID 闭环控制模块
  * @note
  *          - 定时器: TIM2 提供 50ms (20Hz) 控制节拍
  *          - Yaw 角闭环: HWT101CT 反馈 → PID → 底盘旋转速度 vw
  *          - 位置式 PID (Position Form), 带积分分离 + 输出限幅
  *
  *          PID 公式:
  *            u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
  *
  *          积分分离:
  *            当 |e| > ki_thresh 时 Ki=0, 防止大偏差下积分饱和
  ******************************************************************************
  */
#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 数据类型
 * ============================================================ */

/** PID 控制器句柄 */
typedef struct {
    float kp;            /* 比例系数 */
    float ki;            /* 积分系数 */
    float kd;            /* 微分系数 */

    float target;        /* 目标值 (设定点) */
    float feedback;      /* 当前反馈值 */
    float error;         /* 当前误差 */
    float error_prev;    /* 上一次误差 (微分用) */
    float integral;      /* 积分累积 */
    float output;        /* PID 输出 */

    float out_min;       /* 输出下限 */
    float out_max;       /* 输出上限 */
    float ki_thresh;     /* 积分分离阈值: |error| > 此值 → Ki=0 */

    uint8_t enabled;     /* 使能标志: 1=运行, 0=暂停 */
} PID_t;

/* ============================================================
 * Yaw 角 PID 默认参数
 *
 *   调参参考:
 *     Kp 增大 → 响应快, 过大则震荡
 *     Ki 增大 → 消除静差, 过大则超调
 *     Kd 增大 → 抑制超调, 过大则对噪声敏感
 *
 *   20Hz 控制周期下, 从旋转速度 vw → 角度增量,
 *   建议从较小的 Kp 开始逐步增大
 * ============================================================ */
#define PID_YAW_KP              20.0f    /* 比例: 1°误差 → 20 RPM */
#define PID_YAW_KI              0.5f     /* 积分: 缓慢消除静差   */
#define PID_YAW_KD              2.0f     /* 微分: 抑制过冲       */
#define PID_YAW_OUT_MAX         300.0f   /* 输出上限 (RPM)      */
#define PID_YAW_OUT_MIN        -300.0f   /* 输出下限 (RPM)      */
#define PID_YAW_KI_THRESH       15.0f    /* >15°偏差时关闭积分   */

/* ============================================================
 * 全局变量 (声明)
 * ============================================================ */
extern PID_t pid_yaw;

/** PID 控制节拍标志
 *  TIM2 ISR 每 50ms 置 1, 主循环检测后调用 PID_YawLoop() 并清零
 */
extern volatile uint8_t pid_yaw_tick;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief  PID 结构体初始化
 * @param  pid   PID 句柄指针
 * @param  kp/ki/kd  PID 系数
 * @param  out_min/max  输出限幅
 * @param  ki_thresh    积分分离阈值
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max, float ki_thresh);

/**
 * @brief  PID 单次计算 (位置式)
 * @param  pid       PID 句柄指针
 * @param  target    目标值
 * @param  feedback  当前反馈值
 * @param  dt        控制周期 (秒)
 * @return PID 输出值 (已限幅)
 *
 * @note   每次控制周期调用一次
 *         若 pid->enabled == 0, 则重置积分/微分状态后返回 0
 */
float PID_Compute(PID_t *pid, float target, float feedback, float dt);

/**
 * @brief  重置 PID 内部状态 (积分、微分历史)
 */
void PID_Reset(PID_t *pid);

/**
 * @brief  使能 / 暂停 PID
 */
void PID_Enable(PID_t *pid, uint8_t en);

/**
 * @brief  在线修改 PID 参数
 */
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);

/* ---- Yaw 角专用接口 ---- */

/**
 * @brief  初始化 Yaw 角 PID (使用默认参数)
 * @note   在 main() 中调用, TIM2 启动之前
 */
void PID_YawInit(void);

/**
 * @brief  设置 Yaw 目标角度
 * @param  target_deg  目标偏航角 (°), 范围 ±180°
 * @note   内部自动处理角度 wrap (跨越 ±180° 的短路径)
 */
void PID_SetYawTarget(float target_deg);

/**
 * @brief  Yaw 角 PID 循环 (由 TIM2 中断回调调用)
 * @note   50ms 周期执行:
 *           ① 读 HWT101CT Yaw
 *           ② PID 计算 → vw
 *           ③ Chassis_VelMove(0, 0, vw, 0, accel)
 */
void PID_YawLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
