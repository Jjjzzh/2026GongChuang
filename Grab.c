/**
  ******************************************************************************
  * @file    Grab.c
  * @brief   物料抓取/暂存模块实现
  * @note
  *          - 非阻塞状态机: HAL_GetTick 计时, 不阻塞主循环
  *          - Z轴: Emm_V5_Pos_Control 绝对位置模式
  *          - 夹爪: Servo_MoveTo 正弦控速
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Grab.h"
#include "Emm_V5.h" 
#include "Servo.h"
#include "main.h"

/* ============================================================
 * 内部状态
 * ============================================================ */
static GrabState_t grab_state    = GRAB_STATE_IDLE;
static uint32_t    grab_timer    = 0;       /* 等待计时的起始 tick       */
static uint32_t    grab_wait_ms  = 0;       /* 当前等待时长 (ms)        */
static uint8_t     grab_is_pick  = 0;       /* 0=暂存(张开), 1=抓取(闭合)*/
static uint32_t    grab_z_target_deg = 0;   /* 本次 Z 轴目标角度        */

/* ============================================================
 * 内部辅助
 * ============================================================ */

/**
 * @brief  检查延时是否到达
 */
static inline uint8_t Grab_Timeout(void)
{
    return (HAL_GetTick() - grab_timer >= grab_wait_ms) ? 1 : 0;
}

/**
 * @brief  开始计时
 */
static inline void Grab_StartTimer(uint32_t ms)
{
    grab_timer   = HAL_GetTick();
    grab_wait_ms = ms;
}

/**
 * @brief  发送 Z 轴绝对位置指令
 * @param  target_deg  目标角度 (°), 0=回零
 * @param  dir         方向: GRAB_Z_DIR_DOWN / GRAB_Z_DIR_UP
 */
static void Grab_Z_Goto(uint32_t target_deg, uint8_t dir)
{
    uint32_t pulse = GRAB_DEG_TO_PULSE(target_deg);
    Emm_V5_Pos_Control(GRAB_MOTOR_ID, dir, GRAB_Z_SPEED, GRAB_Z_ACCEL,
                       pulse, true,   /* raF=true: 绝对位置  */
                       false);        /* snF=false: 立即执行 */
}

/* ============================================================
 * 公开 API 实现
 * ============================================================ */

/**
 * @brief  初始化: 触发 Z 轴回零, 完成后张开夹爪
 */
void Grab_Init(void)
{
    grab_state   = GRAB_STATE_HOME_START;
    grab_is_pick = 0;
}

/**
 * @brief  状态机推进 (主循环中非阻塞调用)
 */
void Grab_Process(void)
{
    switch (grab_state) {

    /* ---- Z 轴回零 ---- */
    case GRAB_STATE_HOME_START:
        /* 永久保存碰撞回零参数 (svF=true) */
        Emm_V5_Origin_Modify_Params(GRAB_MOTOR_ID,
            true,                       /* svF: 永久保存    */
            GRAB_Z_HOME_MODE,           /* 回零模式 2=碰撞  */
            0,                          /* 回零方向 CW      */
            GRAB_Z_HOME_VEL,            /* 回零速度         */
            GRAB_Z_HOME_TIMEOUT_MS,     /* 回零超时         */
            GRAB_Z_COLLIDE_VEL,         /* 碰撞转速         */
            GRAB_Z_COLLIDE_MA,          /* 碰撞电流 400mA   */
            GRAB_Z_COLLIDE_MS,          /* 碰撞检测时间     */
            false);                     /* 不上电自动回零   */
        HAL_Delay(10);
        /* 读当前状态后触发回零 */
        Emm_V5_Read_Sys_Params(GRAB_MOTOR_ID, S_FLAG);
        HAL_Delay(5);
        Emm_V5_Origin_Trigger_Return(GRAB_MOTOR_ID, GRAB_Z_HOME_MODE, false);
        Grab_StartTimer(GRAB_Z_HOME_TIMEOUT_MS);
        grab_state = GRAB_STATE_HOME_WAIT;
        break;

    /* ---- 等待回零完成 ---- */
    case GRAB_STATE_HOME_WAIT:
        /* 每 100ms 查询一次状态 */
        {
            static uint32_t home_poll_timer = 0;
            if (HAL_GetTick() - home_poll_timer >= 100) {
                home_poll_timer = HAL_GetTick();
                Emm_V5_Read_Sys_Params(GRAB_MOTOR_ID, S_FLAG);
            }
        }
        if (Emm_V5_IsHomed()) {
            /* 回零完成: 当前位置清零, 张开夹爪, 进入空闲 */
            Emm_V5_Reset_CurPos_To_Zero(GRAB_MOTOR_ID);
            Servo_MoveTo(SERVO_GRAB, GRIP_OPEN_DEG, GRIP_MOVE_MS);
            grab_state = GRAB_STATE_IDLE;
        }
        if (Grab_Timeout()) {
            /* 超时: 中断回零, 强制进入空闲 */
            Emm_V5_Origin_Interrupt(GRAB_MOTOR_ID);
            grab_state = GRAB_STATE_IDLE;
        }
        break;

    case GRAB_STATE_IDLE:
    case GRAB_STATE_DONE:
        break;

    /* ---- Z 轴下降 ---- */
    case GRAB_STATE_Z_DOWN:
        Grab_Z_Goto(grab_z_target_deg, GRAB_Z_DIR_DOWN);
        Grab_StartTimer(GRAB_Z_ESTIMATE_MS(grab_z_target_deg));
        grab_state = GRAB_STATE_WAIT_DOWN;
        break;

    /* ---- 等待下降到位 (轮询到位标志, 超时保护) ---- */
    case GRAB_STATE_WAIT_DOWN:
        {
            static uint32_t poll_timer = 0;
            if (HAL_GetTick() - poll_timer >= 100) {
                poll_timer = HAL_GetTick();
                Emm_V5_Read_Sys_Params(GRAB_MOTOR_ID, S_FLAG);
            }
        }
        if (Emm_V5_GetMotorFlag() & 0x08) {  /* bit3: 到位 */
            grab_state = GRAB_STATE_GRIP_ACT;
        }
        if (Grab_Timeout()) {                 /* 超时保护 */
            grab_state = GRAB_STATE_GRIP_ACT;
        }
        break;

    /* ---- 夹爪动作 (闭合 / 暂存先开小口) ---- */
    case GRAB_STATE_GRIP_ACT:
        if (grab_is_pick) {
            Servo_MoveTo(SERVO_GRAB, GRIP_CLOSE_DEG, GRIP_MOVE_MS);
        } else {
            Servo_MoveTo(SERVO_GRAB, GRIP_PARTIAL_OPEN_DEG, GRIP_MOVE_MS);
        }
        Grab_StartTimer(GRIP_MOVE_MS + 50);
        grab_state = GRAB_STATE_WAIT_ACT;
        break;

    /* ---- 等待夹爪到位 ---- */
    case GRAB_STATE_WAIT_ACT:
        if (Grab_Timeout()) {
            grab_state = GRAB_STATE_Z_UP;
        }
        break;

    /* ---- Z 轴上升回零 ---- */
    case GRAB_STATE_Z_UP:
        Grab_Z_Goto(0, GRAB_Z_DIR_UP);
        Grab_StartTimer(GRAB_Z_ESTIMATE_MS(grab_z_target_deg));
        grab_state = GRAB_STATE_WAIT_UP;
        break;

    /* ---- 等待上升到位 (轮询到位标志, 超时保护) ---- */
    case GRAB_STATE_WAIT_UP:
        {
            static uint32_t poll_timer = 0;
            if (HAL_GetTick() - poll_timer >= 100) {
                poll_timer = HAL_GetTick();
                Emm_V5_Read_Sys_Params(GRAB_MOTOR_ID, S_FLAG);
            }
        }
        if ((Emm_V5_GetMotorFlag() & 0x08) || Grab_Timeout()) {
            /* 暂存还需要最后完全张开, 抓取直接完成 */
            grab_state = grab_is_pick ? GRAB_STATE_DONE : GRAB_STATE_GRIP_FINAL;
        }
        break;

    /* ---- 暂存最终: PTZ回180° + 完全张开夹爪 (并行, 不等PTZ) ---- */
    case GRAB_STATE_GRIP_FINAL:
        Servo_MoveTo(SERVO_PTZ, PTZ_RETURN_DEG, GRIP_MOVE_MS);  /* fire & forget */
        Servo_MoveTo(SERVO_GRAB, GRIP_OPEN_DEG, GRIP_MOVE_MS);
        Grab_StartTimer(GRIP_MOVE_MS + 50);
        grab_state = GRAB_STATE_WAIT_FINAL;
        break;

    /* ---- 等待最终张开到位 ---- */
    case GRAB_STATE_WAIT_FINAL:
        if (Grab_Timeout()) {
            grab_state = GRAB_STATE_DONE;
        }
        break;

    default:
        grab_state = GRAB_STATE_IDLE;
        break;
    }
}

/**
 * @brief  触发物料盘抓取
 */
void Grab_StartPickTray(void)
{
    if (grab_state != GRAB_STATE_IDLE &&
        grab_state != GRAB_STATE_DONE) {
        return;
    }
    grab_is_pick      = 1;
    grab_z_target_deg = TRAY_Z_DOWN_DEG;
    grab_state        = GRAB_STATE_Z_DOWN;
}

/**
 * @brief  触发加工区抓取
 */
void Grab_StartPickProcess(void)
{
    if (grab_state != GRAB_STATE_IDLE &&
        grab_state != GRAB_STATE_DONE) {
        return;
    }
    grab_is_pick      = 1;
    grab_z_target_deg = PROCESS_Z_DOWN_DEG;
    grab_state        = GRAB_STATE_Z_DOWN;
}

/**
 * @brief  触发暂存
 */
void Grab_StartStore(void)
{
    if (grab_state != GRAB_STATE_IDLE &&
        grab_state != GRAB_STATE_DONE) {
        return;  /* 忙 */
    }
    grab_is_pick      = 0;
    grab_z_target_deg = STORE_Z_DOWN_DEG;
    grab_state        = GRAB_STATE_Z_DOWN;
}

/**
 * @brief  查询是否忙
 */
uint8_t Grab_IsBusy(void)
{
    return (grab_state != GRAB_STATE_IDLE &&
            grab_state != GRAB_STATE_DONE) ? 1 : 0;
}

/**
 * @brief  获取当前状态
 */
GrabState_t Grab_GetState(void)
{
    return grab_state;
}

/**
 * @brief  紧急停止: 电机急停 + 夹爪张开
 */
void Grab_EmergencyStop(void)
{
    Emm_V5_Stop_Now(GRAB_MOTOR_ID, false);
    Servo_MoveTo(SERVO_GRAB, GRIP_OPEN_DEG, GRIP_MOVE_MS);
    grab_state   = GRAB_STATE_IDLE;
    grab_is_pick = 0;
}
