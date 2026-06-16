/**
  ******************************************************************************
  * @file    Place.h
  * @brief   物料放置模块 (取回暂存物料 → 搬运到放置点释放)
  * @note
  *          - 依赖: Grab.h (宏定义复用), Emm_V5, Servo
  *          - 非阻塞状态机, Place_Process() 在主循环中推进
  *
  *          序列:
  *            ① PTZ→暂存点  ‖ 夹爪→15°        (并行, 等两者)
  *            ② Z→暂存高度                      (取回物料)
  *            ③ 夹爪闭合
  *            ④ Z回零
  *            ⑤ PTZ→180°
  *            ⑥ Z→放置高度
  *            ⑦ 夹爪张开 ‖ Z回零               (并行, 等两者)
  *            ⑧ 完成
  ******************************************************************************
  */
 
#ifndef __PLACE_H__
#define __PLACE_H__

#include <stdint.h>
#include "Grab.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 放置参数 (占位值, 标定后修改)
 * ============================================================ */

/* 最终放置 Z 轴下降角度 (°) */
#define PLACE_Z_DOWN_DEG        180     /* 放置高度 (占位) */
#define PALLET_Z_DOWN_DEG       120     /* 码垛放置高度 (占位) */

/* ============================================================
 * 状态枚举
 * ============================================================ */
typedef enum {
    PLACE_STATE_IDLE = 0,
    PLACE_STATE_STEP1,          /* 发起 PTZ→暂存点 + 夹爪→15° */
    PLACE_STATE_WAIT_STEP1,     /* 等待两者完成                 */
    PLACE_STATE_Z_DOWN_STORE,   /* Z→暂存高度 (取回物料)       */
    PLACE_STATE_WAIT_Z_DOWN,    /* 等待 Z 到位                 */
    PLACE_STATE_CLAW_CLOSE,     /* 夹爪闭合                    */
    PLACE_STATE_WAIT_CLAW,      /* 等待夹爪到位                */
    PLACE_STATE_Z_UP,           /* Z→回零                      */
    PLACE_STATE_WAIT_Z_UP,      /* 等待 Z 到位                 */
    PLACE_STATE_PTZ_180,        /* PTZ→180°                    */
    PLACE_STATE_WAIT_PTZ,       /* 等待 PTZ 到位               */
    PLACE_STATE_Z_DOWN_PLACE,   /* Z→放置高度                  */
    PLACE_STATE_WAIT_Z_DOWN2,   /* 等待 Z 到位                 */
    PLACE_STATE_RELEASE,        /* 夹爪张开 ‖ Z回零 (并行)    */
    PLACE_STATE_WAIT_RELEASE,   /* 等待两者完成                */
    PLACE_STATE_DONE
} PlaceState_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief  初始化放置模块
 */
void Place_Init(void);

/**
 * @brief  状态机推进 (主循环中非阻塞调用)
 */
void Place_Process(void);

/**
 * @brief  触发放置序列 (使用 PLACE_Z_DOWN_DEG)
 */
void Place_Start(void);

/**
 * @brief  触发码垛放置序列 (使用 PALLET_Z_DOWN_DEG)
 */
void Place_StartPallet(void);

/**
 * @brief  查询状态机是否忙
 */
uint8_t Place_IsBusy(void);

/**
 * @brief  获取当前状态
 */
PlaceState_t Place_GetState(void);

/**
 * @brief  紧急停止
 */
void Place_EmergencyStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __PLACE_H__ */
