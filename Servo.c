#include "Servo.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================
 * 正弦控速: 每通道运动状态
 * ============================================================ */
typedef struct {
    uint16_t start_pulse;    /* 起始脉宽 (us)        */
    uint16_t target_pulse;   /* 目标脉宽 (us)        */
    uint32_t start_tick;     /* 起始时刻 (ms)        */
    uint16_t duration_ms;    /* 全程耗时 (ms)        */
    uint8_t  running;        /* 正在运动中            */
} ServoMove_t;

static ServoMove_t servo_move[4];  /* ch=1,2,3, 索引0未用 */

/**
  * @brief  舵机初始化 - 启动 PWM 并归中
  * @retval None
  */
void Servo_Init(void)
{
    /* 启动 TIM4 三路 PWM */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    /* 夹爪归中, 转盘初始 120°, 云台初始 55° */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_GRAB_PULSE_MID);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1500);  /* 转盘 120° */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 907);   /* 云台 55° */
}

/**
  * @brief  设置舵机脉宽（通道独立限幅）
  * @param  ch       舵机通道
  * @param  pulse_us 脉宽 us (自动限幅到该通道范围)
  * @retval None
  */
void Servo_SetPulse(ServoCh_t ch, uint16_t pulse_us)
{
    switch (ch) {
    case SERVO_GRAB:
        if (pulse_us < SERVO_GRAB_PULSE_MIN) pulse_us = SERVO_GRAB_PULSE_MIN;
        if (pulse_us > SERVO_GRAB_PULSE_MAX) pulse_us = SERVO_GRAB_PULSE_MAX;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
        break;
    case SERVO_TURNTABLE:
        if (pulse_us < TURNTABLE_PULSE_MIN) pulse_us = TURNTABLE_PULSE_MIN;
        if (pulse_us > TURNTABLE_PULSE_MAX) pulse_us = TURNTABLE_PULSE_MAX;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse_us);
        break;
    case SERVO_PTZ:
        if (pulse_us < 907)  pulse_us = 907;   /* PTZ_ANGLE_MIN → 55°  */
        if (pulse_us > 2241) pulse_us = 2241;  /* PTZ_ANGLE_MAX → 235° */
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_us);
        break;
    }
}

/**
  * @brief  设置舵机角度（通道独立映射）
  * @param  ch    舵机通道
  * @param  angle 角度 (自动限幅到该通道范围, 线性映射到脉宽)
  * @retval None
  */
void Servo_SetAngle(ServoCh_t ch, uint16_t angle)
{
    uint16_t pulse;

    switch (ch) {
    case SERVO_GRAB:
        if (angle > SERVO_GRAB_ANGLE_MAX) angle = SERVO_GRAB_ANGLE_MAX;
        pulse = SERVO_GRAB_PULSE_MIN
              + (uint32_t)angle * (SERVO_GRAB_PULSE_MAX - SERVO_GRAB_PULSE_MIN)
              / SERVO_GRAB_ANGLE_MAX;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
        break;
    case SERVO_TURNTABLE:
        if (angle > TURNTABLE_ANGLE_MAX) angle = TURNTABLE_ANGLE_MAX;
        pulse = TURNTABLE_PULSE_MIN
              + (uint32_t)angle * (TURNTABLE_PULSE_MAX - TURNTABLE_PULSE_MIN)
              / TURNTABLE_ANGLE_MAX;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse);
        break;
    case SERVO_PTZ:
        if (angle < PTZ_ANGLE_MIN) angle = PTZ_ANGLE_MIN;
        if (angle > PTZ_ANGLE_MAX) angle = PTZ_ANGLE_MAX;
        pulse = PTZ_PULSE_MIN
              + (uint32_t)angle * (PTZ_PULSE_MAX - PTZ_PULSE_MIN)
              / 270;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse);
        break;
    }
}

/* ============================================================
 * 正弦控速 API
 * ============================================================ */

/**
 * @brief  内部辅助: 角度→脉宽 (复用 Servo_SetAngle 的映射)
 */
static uint16_t Servo_AngleToPulse(ServoCh_t ch, uint16_t angle)
{
    switch (ch) {
    case SERVO_GRAB:
        if (angle > SERVO_GRAB_ANGLE_MAX) angle = SERVO_GRAB_ANGLE_MAX;
        return SERVO_GRAB_PULSE_MIN
             + (uint32_t)angle * (SERVO_GRAB_PULSE_MAX - SERVO_GRAB_PULSE_MIN)
             / SERVO_GRAB_ANGLE_MAX;
    case SERVO_TURNTABLE:
        if (angle > TURNTABLE_ANGLE_MAX) angle = TURNTABLE_ANGLE_MAX;
        return TURNTABLE_PULSE_MIN
             + (uint32_t)angle * (TURNTABLE_PULSE_MAX - TURNTABLE_PULSE_MIN)
             / TURNTABLE_ANGLE_MAX;
    case SERVO_PTZ:
        if (angle < PTZ_ANGLE_MIN) angle = PTZ_ANGLE_MIN;
        if (angle > PTZ_ANGLE_MAX) angle = PTZ_ANGLE_MAX;
        return PTZ_PULSE_MIN
             + (uint32_t)angle * (PTZ_PULSE_MAX - PTZ_PULSE_MIN)
             / 270;
    default:
        return 1500;
    }
}

/**
 * @brief  正弦控速移动到目标角度
 * @param  ch           舵机通道
 * @param  target_angle 目标角度 (自动限幅)
 * @param  duration_ms  全程耗时 (ms), 0=立即到位
 * @note   调用后需在主循环中周期性调用 Servo_Update()
 *
 *   位置曲线: p(t) = start + Δp·(1-cos(π·t/T))/2
 *   速度曲线: v(t) = Δp·π/(2T)·sin(π·t/T)  → 先升后降, 零起零止
 */
void Servo_MoveTo(ServoCh_t ch, uint16_t target_angle, uint16_t duration_ms)
{
    if (ch < SERVO_GRAB || ch > SERVO_PTZ) return;

    uint16_t target_pulse = Servo_AngleToPulse(ch, target_angle);

    /* 读取当前 PWM 比较值作为起始值 */
    uint16_t cur_pulse;
    switch (ch) {
    case SERVO_GRAB: cur_pulse = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1); break;
    case SERVO_TURNTABLE: cur_pulse = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2); break;
    case SERVO_PTZ:  cur_pulse = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3); break;
    default:         return;
    }

    /* 无需移动 */
    if (cur_pulse == target_pulse || duration_ms == 0) {
        Servo_SetPulse(ch, target_pulse);
        servo_move[ch].running = 0;
        return;
    }

    servo_move[ch].start_pulse  = cur_pulse;
    servo_move[ch].target_pulse = target_pulse;
    servo_move[ch].start_tick   = HAL_GetTick();
    servo_move[ch].duration_ms  = duration_ms;
    servo_move[ch].running      = 1;
}

/**
 * @brief  舵机正弦控速更新 (在主循环中周期性调用)
 * @note   无需定时器, 用 HAL_GetTick 计时
 *         调用频率 ≥ 20Hz 即可保证平滑
 */
void Servo_Update(void)
{
    for (uint8_t ch = SERVO_GRAB; ch <= SERVO_PTZ; ch++) {
        if (!servo_move[ch].running) continue;

        uint32_t elapsed = HAL_GetTick() - servo_move[ch].start_tick;
        uint16_t T       = servo_move[ch].duration_ms;

        /* 已到终点 */
        if (elapsed >= T) {
            Servo_SetPulse((ServoCh_t)ch, servo_move[ch].target_pulse);
            servo_move[ch].running = 0;
            continue;
        }

        /* 余弦 S 曲线插值 */
        float progress = (float)elapsed / T;
        float cosine   = (1.0f - cosf(M_PI * progress)) / 2.0f;
        float delta    = (float)((int32_t)servo_move[ch].target_pulse
                               - (int32_t)servo_move[ch].start_pulse);
        uint16_t pulse = (uint16_t)((float)servo_move[ch].start_pulse
                                  + delta * cosine);

        Servo_SetPulse((ServoCh_t)ch, pulse);
    }
}

/**
 * @brief  查询舵机是否已完成正弦控速
 * @return 1=已到位/未运行, 0=运动中
 */
uint8_t Servo_IsDone(ServoCh_t ch)
{
    if (ch < SERVO_GRAB || ch > SERVO_PTZ) return 1;
    return !servo_move[ch].running;
}

