/**
  ******************************************************************************
  * @file    Place.c
  * @brief   物料放置模块实现
  * @note
  *          - 非阻塞状态机: HAL_GetTick 计时, Servo_IsDone 判断舵机到位
  *          - Z轴: Emm_V5_Pos_Control 绝对位置模式
  *          - 并行步骤: 同时发起动作, 等两者各自完成后推进
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Place.h"
#include "Emm_V5.h"
#include "Servo.h"
#include "main.h"

/* ============================================================
 * 内部状态
 * ============================================================ */
static PlaceState_t place_state      = PLACE_STATE_IDLE;
static uint32_t     place_timer      = 0;
static uint32_t     place_wait_ms    = 0;
static uint32_t     place_z_target_deg = 0;  /* 本次放置目标高度 */

/* ============================================================
 * 内部辅助
 * ============================================================ */

static inline uint8_t Place_Timeout(void)
{
    return (HAL_GetTick() - place_timer >= place_wait_ms) ? 1 : 0;
}

static inline void Place_StartTimer(uint32_t ms)
{
    place_timer   = HAL_GetTick();
    place_wait_ms = ms;
}

/** 发送 Z 轴绝对位置指令 */
static void Place_Z_Goto(uint32_t target_deg, uint8_t dir)
{
    uint32_t pulse = GRAB_DEG_TO_PULSE(target_deg);
    Emm_V5_Pos_Control(GRAB_MOTOR_ID, dir, GRAB_Z_SPEED, GRAB_Z_ACCEL,
                       pulse, true, false);
}

/* ============================================================
 * 公开 API
 * ============================================================ */

void Place_Init(void)
{
    place_state = PLACE_STATE_IDLE;
}

void Place_Process(void)
{
    switch (place_state) {

    case PLACE_STATE_IDLE:
    case PLACE_STATE_DONE:
        break;

    /* ---- ① PTZ→暂存点 ‖ 夹爪→15° (同时发起) ---- */
    case PLACE_STATE_STEP1:
        Servo_MoveTo(SERVO_PTZ,  STORE_YAW_DEG,         GRIP_MOVE_MS);
        Servo_MoveTo(SERVO_GRAB, GRIP_PARTIAL_OPEN_DEG, GRIP_MOVE_MS);
        place_state = PLACE_STATE_WAIT_STEP1;
        Place_StartTimer(GRIP_MOVE_MS + 100);
        break;

    /* ---- 等待两者完成 (舵机 + 超时兜底) ---- */
    case PLACE_STATE_WAIT_STEP1:
        if (Servo_IsDone(SERVO_PTZ) && Servo_IsDone(SERVO_GRAB)) {
            place_state = PLACE_STATE_Z_DOWN_STORE;
        } else if (Place_Timeout()) {
            /* 超时兜底, 继续下一步 */
            place_state = PLACE_STATE_Z_DOWN_STORE;
        }
        break;

    /* ---- ② Z→暂存高度 ---- */
    case PLACE_STATE_Z_DOWN_STORE:
        Place_Z_Goto(STORE_Z_DOWN_DEG, GRAB_Z_DIR_DOWN);
        Place_StartTimer(GRAB_Z_ESTIMATE_MS(STORE_Z_DOWN_DEG));
        place_state = PLACE_STATE_WAIT_Z_DOWN;
        break;

    case PLACE_STATE_WAIT_Z_DOWN:
        if (Place_Timeout()) {
            place_state = PLACE_STATE_CLAW_CLOSE;
        }
        break;

    /* ---- ③ 夹爪闭合 ---- */
    case PLACE_STATE_CLAW_CLOSE:
        Servo_MoveTo(SERVO_GRAB, GRIP_CLOSE_DEG, GRIP_MOVE_MS);
        Place_StartTimer(GRIP_MOVE_MS + 50);
        place_state = PLACE_STATE_WAIT_CLAW;
        break;

    case PLACE_STATE_WAIT_CLAW:
        if (Place_Timeout()) {
            place_state = PLACE_STATE_Z_UP;
        }
        break;

    /* ---- ④ Z回零 ---- */
    case PLACE_STATE_Z_UP:
        Place_Z_Goto(0, GRAB_Z_DIR_UP);
        Place_StartTimer(GRAB_Z_ESTIMATE_MS(STORE_Z_DOWN_DEG));
        place_state = PLACE_STATE_WAIT_Z_UP;
        break;

    case PLACE_STATE_WAIT_Z_UP:
        if (Place_Timeout()) {
            place_state = PLACE_STATE_PTZ_180;
        }
        break;

    /* ---- ⑤ PTZ→180° ---- */
    case PLACE_STATE_PTZ_180:
        Servo_MoveTo(SERVO_PTZ, PTZ_RETURN_DEG, GRIP_MOVE_MS);
        Place_StartTimer(GRIP_MOVE_MS + 100);
        place_state = PLACE_STATE_WAIT_PTZ;
        break;

    case PLACE_STATE_WAIT_PTZ:
        if (Servo_IsDone(SERVO_PTZ) || Place_Timeout()) {
            place_state = PLACE_STATE_Z_DOWN_PLACE;
        }
        break;

    /* ---- ⑥ Z→放置高度 ---- */
    case PLACE_STATE_Z_DOWN_PLACE:
        Place_Z_Goto(place_z_target_deg, GRAB_Z_DIR_DOWN);
        Place_StartTimer(GRAB_Z_ESTIMATE_MS(place_z_target_deg));
        place_state = PLACE_STATE_WAIT_Z_DOWN2;
        break;

    case PLACE_STATE_WAIT_Z_DOWN2:
        if (Place_Timeout()) {
            place_state = PLACE_STATE_RELEASE;
        }
        break;

    /* ---- ⑦ 夹爪张开 ‖ Z回零 (同时发起) ---- */
    case PLACE_STATE_RELEASE:
        Servo_MoveTo(SERVO_GRAB, GRIP_OPEN_DEG, GRIP_MOVE_MS);
        Place_Z_Goto(0, GRAB_Z_DIR_UP);
        Place_StartTimer(GRAB_Z_ESTIMATE_MS(place_z_target_deg));
        place_state = PLACE_STATE_WAIT_RELEASE;
        break;

    /* ---- ⑧ 等待两者完成 ---- */
    case PLACE_STATE_WAIT_RELEASE:
        if (Servo_IsDone(SERVO_GRAB) && Place_Timeout()) {
            place_state = PLACE_STATE_DONE;
        }
        break;

    default:
        place_state = PLACE_STATE_IDLE;
        break;
    }
}

/**
 * @brief  触发放置序列 (使用 PLACE_Z_DOWN_DEG)
 */
void Place_Start(void)
{
    if (place_state != PLACE_STATE_IDLE &&
        place_state != PLACE_STATE_DONE) {
        return;
    }
    place_z_target_deg = PLACE_Z_DOWN_DEG;
    place_state        = PLACE_STATE_STEP1;
}

/**
 * @brief  触发码垛放置序列 (使用 PALLET_Z_DOWN_DEG)
 */
void Place_StartPallet(void)
{
    if (place_state != PLACE_STATE_IDLE &&
        place_state != PLACE_STATE_DONE) {
        return;
    }
    place_z_target_deg = PALLET_Z_DOWN_DEG;
    place_state        = PLACE_STATE_STEP1;
}

/**
 * @brief  查询状态机是否忙
 * @return 1=运行中, 0=空闲/完成
 */
uint8_t Place_IsBusy(void)
{
    return (place_state != PLACE_STATE_IDLE &&
            place_state != PLACE_STATE_DONE) ? 1 : 0;
}

/**
 * @brief  获取当前状态
 */
PlaceState_t Place_GetState(void)
{
    return place_state;
}

/**
 * @brief  紧急停止: 电机急停 + 夹爪张开
 */
void Place_EmergencyStop(void)
{
    Emm_V5_Stop_Now(GRAB_MOTOR_ID, false);
    Servo_MoveTo(SERVO_GRAB, GRIP_OPEN_DEG, GRIP_MOVE_MS);
    place_state = PLACE_STATE_IDLE;
}
