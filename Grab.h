/**
  ******************************************************************************
  * @file    Grab.h
  * @brief   物料抓取/暂存模块 (Z轴升降 + 夹爪开合)
  * @note
  *          - 依赖: Emm_V5 (Z轴步进电机 ID=5), Servo (夹爪舵机)
  *          - 不负责: PTZ舵机(外部对准)、底盘Yaw(外部对准)
  *          - 非阻塞状态机, Grab_Process() 在主循环中推进
  *          - Z轴使用 Emm_V5 位置模式 (绝对位置)
  *
  *          上电: Z轴硬件自动回零, 程序仅初始化夹爪张开
  *
  *          序列:
  *            抓取: Z下降 → 夹爪闭合 → Z上升回零
  *            暂存: Z下降 → 夹爪开15°(松开放下) → Z上升回零 → 夹爪完全张开
  ******************************************************************************
  */
#ifndef __GRAB_H__
#define __GRAB_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 电机参数
 * ============================================================ */
#define GRAB_MOTOR_ID           5       /* Z轴升降电机 Emm_V5 地址      */

/* 脉冲换算: 1转 = 360° = 16细分 × 200步 = 3200 脉冲 */
#define GRAB_PULSE_PER_REV      3200
#define GRAB_DEG_TO_PULSE(deg)  ((uint32_t)((uint32_t)(deg) * GRAB_PULSE_PER_REV / 360))

/* ============================================================
 * 运动参数 (占位值, 标定后修改)
 * ============================================================ */

/* 升降统一速度 (RPM) */
#define GRAB_Z_SPEED            200

/* 加减速 (0=默认) */
#define GRAB_Z_ACCEL            0

/* 方向: 下降(CCW) / 上升回零(CW), 互反 */
#define GRAB_Z_DIR_DOWN         0       /* CCW */
#define GRAB_Z_DIR_UP           1       /* CW  (回零方向) */

/* 回零参数 */
#define GRAB_Z_HOME_MODE        2       /* 回零模式: 0=近点 1=限位 2=碰撞 3=无开关 */
#define GRAB_Z_HOME_VEL         50      /* 回零速度 (RPM) */
#define GRAB_Z_HOME_TIMEOUT_MS  15000   /* 回零超时 (ms) */
#define GRAB_Z_COLLIDE_VEL      200     /* 碰撞回零转速 (RPM) */
#define GRAB_Z_COLLIDE_MA       400     /* 碰撞回零电流 (mA) */
#define GRAB_Z_COLLIDE_MS       500     /* 碰撞检测时间 (ms) */

/* Z轴下降角度 (°), 不同场景高度不同 */
#define TRAY_Z_DOWN_DEG         720     /* 物料盘抓取高度 (占位) */
#define PROCESS_Z_DOWN_DEG      540     /* 加工区抓取高度 (占位) */
#define STORE_Z_DOWN_DEG        360     /* 暂存高度 (占位) */

/* ============================================================
 * 夹爪舵机 (占位值, 标定后修改)
 * ============================================================ */
#define GRIP_CLOSE_DEG          0       /* 闭合角度 */
#define GRIP_OPEN_DEG           90      /* 完全张开角度 */
#define GRIP_PARTIAL_OPEN_DEG   15      /* 暂存放松角度 (只开小口) */

/* 夹爪动作耗时 (ms), 正弦控速 */
#define GRIP_MOVE_MS            300

/* ============================================================
 * 暂存点 Yaw (供外部 Position 模块引用)
 * ============================================================ */
#define STORE_YAW_DEG           110     /* 暂存点 PTZ 角度 */
#define PTZ_RETURN_DEG          235     /* 暂存完成后 PTZ 回正角度 */

/* ============================================================
 * 运动时间估算
 *
 *   T(ms) = angle(°) / (RPM / 60 × 360) × 1000
 *         = angle × 1000 / (RPM × 6)
 *
 *   加余量补偿加减速段
 * ============================================================ */
#define GRAB_Z_MARGIN_MS        500     /* 运动余量 (ms) */
#define GRAB_Z_ESTIMATE_MS(deg) ((uint32_t)((deg) * 1000 / (GRAB_Z_SPEED * 6)) + GRAB_Z_MARGIN_MS)

/* ============================================================
 * 状态枚举
 * ============================================================ */
typedef enum {
    GRAB_STATE_HOME_START = 0,  /* 触发 Z 轴回零         */
    GRAB_STATE_HOME_WAIT,       /* 等待回零完成           */
    GRAB_STATE_IDLE,            /* 空闲                   */
    GRAB_STATE_Z_DOWN,          /* 发送下降位置指令       */
    GRAB_STATE_WAIT_DOWN,       /* 等待 Z 轴到达目标      */
    GRAB_STATE_GRIP_ACT,        /* 夹爪动作 (闭合/张开)   */
    GRAB_STATE_WAIT_ACT,        /* 等待夹爪动作完成       */
    GRAB_STATE_Z_UP,            /* 发送上升回零指令       */
    GRAB_STATE_WAIT_UP,         /* 等待 Z 轴到达零点      */
    GRAB_STATE_GRIP_FINAL,      /* 暂存: 回零后完全张开   */
    GRAB_STATE_WAIT_FINAL,      /* 等待最终夹爪动作完成   */
    GRAB_STATE_DONE
} GrabState_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief  初始化: 夹爪张开到安全位
 * @note   上电时 Z 轴已由硬件自动回零, 此处仅初始化夹爪
 */
void Grab_Init(void);

/**
 * @brief  状态机推进 (主循环中非阻塞调用)
 */
void Grab_Process(void);

/**
 * @brief  触发物料盘抓取: Z下降(TRAY_Z_DOWN_DEG) → 闭合 → Z上升回零
 */
void Grab_StartPickTray(void);

/**
 * @brief  触发加工区抓取: Z下降(PROCESS_Z_DOWN_DEG) → 闭合 → Z上升回零
 */
void Grab_StartPickProcess(void);

/**
 * @brief  触发暂存: Z下降(STORE_Z_DOWN_DEG) → 开15°放下 → Z↑回零
 *                                       → PTZ回180° ‖ 完全张开
 */
void Grab_StartStore(void);

/**
 * @brief  查询状态机是否忙
 * @return 1=运行中, 0=空闲/完成
 */
uint8_t Grab_IsBusy(void);

/**
 * @brief  获取当前状态
 */
GrabState_t Grab_GetState(void);

/**
 * @brief  紧急停止并回到安全姿态
 */
void Grab_EmergencyStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __GRAB_H__ */
