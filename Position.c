/**
  ******************************************************************************
  * @file    Position.c
  * @brief   物料视觉定位模块实现
  * @note
  *          - 迭代闭环: 移动 → 重检测 → 修正, 直到误差收敛
  *          - 发散保护: 误差扩大 2× 自动终止
  *          - 阻塞调用, 完成定位后才返回
  *
  *          流程图:
  *            ┌──────────────────────┐
  *            │ 发 TASK_TRAY 请求识别 │←──────────────┐
  *            └─────────┬────────────┘                │
  *                      │ 等待 Jetson 响应 (超时 3s)    │
  *                      │ 读 tray_x, tray_y           │
  *              ┌───────┴───────┐                     │
  *              │ err ≤ 容差 ?  │                     │
  *              └───────┬───────┘                     │
  *               Yes    │ No                          │
  *                │     └── err↑ ? ──Yes──→ return -3 │
  *                │          │No                      │
  *                │     px→mm → Chassis_PosMove       │
  *                │     先 X 后 Y ─────────────────────┘
  *           return 0
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Position.h"
#include "jetsondata.h"
#include "main.h" 

/* ============================================================
 * 内部静态函数
 * ============================================================ */

/**
 * @brief  等待 Jetson 返回物料盘坐标, 带超时
 * @param  timeout_ms  超时时间 (ms)
 * @return  0  收到 JETSON_FLAG_TRAY
 *         -1  超时
 * @note   在等待循环中持续调用 JetsonData_Process 处理 DMA 数据
 */
static int Position_WaitTray(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (!JetsonData_IsUpdated(JETSON_FLAG_TRAY)) {
        JetsonData_Process();

        if (HAL_GetTick() - start >= timeout_ms) {
            return -1;
        }
    }
    return 0;
}

/* ============================================================
 * 公开 API 实现
 * ============================================================ */

/**
 * @brief  物料盘视觉定位: 迭代闭环对准预设目标
 */
int Position_AlignToTray(uint16_t v_max, uint16_t dt_ms)
{
    /* 上一次误差的绝对值 (用于发散检测) */
    int16_t prev_abs_x = INT16_MAX;
    int16_t prev_abs_y = INT16_MAX;

    for (int iter = 0; iter < POSITION_MAX_ITER; iter++) {

        /* ---- 1. 请求 Jetson 检测物料盘 ---- */
        JetsonData_ClearFlags();
        JetsonData_SendTask(JETSON_TASK_TRAY, 0);

        /* ---- 2. 等待坐标数据 ---- */
        if (Position_WaitTray(POSITION_TIMEOUT_MS) != 0) {
            return -1;  /* Jetson 超时无响应 */
        }

        /* ---- 3. 读取当前物料盘坐标 ---- */
        int16_t cur_x = (int16_t)JetsonData_GetTrayX();
        int16_t cur_y = (int16_t)JetsonData_GetTrayY();
        JetsonData_ClearFlags();

        /* ---- 4. 计算像素偏差 ---- */
        int16_t err_x = (int16_t)POSITION_TARGET_X - cur_x;
        int16_t err_y = (int16_t)POSITION_TARGET_Y - cur_y;

        int16_t abs_x = (err_x >= 0) ? err_x : -err_x;
        int16_t abs_y = (err_y >= 0) ? err_y : -err_y;

        /* ---- 5. 到位判断 ---- */
        if (abs_x <= POSITION_TOLERANCE_PX &&
            abs_y <= POSITION_TOLERANCE_PX) {
            return 0;  /* 已到目标位置 */
        }

        /* ---- 6. 发散检测 (跳过第一轮, 无历史数据) ---- */
        if (iter > 0) {
            if (abs_x > prev_abs_x * 2 || abs_y > prev_abs_y * 2) {
                return -3;  /* 误差扩大, 方向映射可能反了 */
            }
        }
        prev_abs_x = abs_x;
        prev_abs_y = abs_y;

        /* ---- 7. 像素 → mm 换算 (含方向) ---- */
        float mm_x = (float)err_x * POSITION_SCALE_MM_PER_PX
                     * (float)POSITION_CAMX_SIGN;
        float mm_y = (float)err_y * POSITION_SCALE_MM_PER_PX
                     * (float)POSITION_CAMY_SIGN;

        /* ---- 8. 移动底盘 (先 X 后 Y) ---- */
        if (abs_x > POSITION_TOLERANCE_PX) {
            Chassis_PosMove(mm_x, POSITION_CAMX_AXIS, v_max, dt_ms);
        }
        if (abs_y > POSITION_TOLERANCE_PX) {
            Chassis_PosMove(mm_y, POSITION_CAMY_AXIS, v_max, dt_ms);
        }

    } /* for iter */

    return -2;  /* 超过最大迭代次数, 仍未到位 */
}

/**
 * @brief  等待 Jetson 返回色环校准数据, 带超时
 * @param  timeout_ms  超时时间 (ms)
 * @return  0  收到 JETSON_FLAG_RING_CALIB
 *         -1  超时
 */
static int Position_WaitCalib(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (!JetsonData_IsUpdated(JETSON_FLAG_RING_CALIB)) {
        JetsonData_Process();

        if (HAL_GetTick() - start >= timeout_ms) {
            return -1;
        }
    }
    return 0;
}

/**
 * @brief  车身校准: 三轴迭代闭环
 *
 * 每轮迭代:
 *   ① 发 JETSON_TASK_CALIB 请求色环校准
 *   ② 等 Jetson 返回 calib_x, calib_y, calib_z
 *   ③ 若三轴均在容差内 → return 0
 *   ④ 发散检测
 *   ⑤ 先纠正 Z (旋转,  因为旋转会改变 XY 视野)
 *   ⑥ 再纠正 XY (平移)
 *   ⑦ 回到①
 */
int Position_BodyCalib(uint16_t v_max, uint16_t dt_ms)
{
    int16_t prev_abs_x = INT16_MAX;
    int16_t prev_abs_y = INT16_MAX;
    int16_t prev_abs_z = INT16_MAX;

    for (int iter = 0; iter < POSITION_MAX_ITER; iter++) {

        /* ---- 1. 请求 Jetson 色环校准 ---- */
        JetsonData_ClearFlags();
        JetsonData_SendTask(JETSON_TASK_CALIB, 0);

        /* ---- 2. 等待校准数据 ---- */
        if (Position_WaitCalib(POSITION_TIMEOUT_MS) != 0) {
            return -1;  /* Jetson 超时 */
        }

        /* ---- 3. 读取校准数据 ---- */
        int16_t cx = (int16_t)JetsonData_GetCalibX();
        int16_t cy = (int16_t)JetsonData_GetCalibY();
        int16_t cz = JetsonData_GetCalibZ();
        JetsonData_ClearFlags();

        /* ---- 4. 三轴偏差 ---- */
        int16_t err_x = (int16_t)CALIB_TARGET_X - cx;
        int16_t err_y = (int16_t)CALIB_TARGET_Y - cy;

        int16_t abs_x = (err_x >= 0) ? err_x : -err_x;
        int16_t abs_y = (err_y >= 0) ? err_y : -err_y;
        int16_t abs_z = (cz    >= 0) ? cz    : -cz;

        /* ---- 5. 到位判断 ---- */
        if (abs_z <= CALIB_Z_TOLERANCE_PX &&
            abs_x <= POSITION_TOLERANCE_PX   &&
            abs_y <= POSITION_TOLERANCE_PX) {
            return 0;
        }

        /* ---- 6. 发散检测 ---- */
        if (iter > 0) {
            if (abs_z > prev_abs_z * 2 ||
                abs_x > prev_abs_x * 2 ||
                abs_y > prev_abs_y * 2) {
                return -3;
            }
        }
        prev_abs_z = abs_z;
        prev_abs_x = abs_x;
        prev_abs_y = abs_y;

        /* ---- 7. 先纠 Z (旋转): calib_z → 角度 ---- */
        if (abs_z > CALIB_Z_TOLERANCE_PX) {
            float angle = (float)cz * CALIB_Z_TO_ANGLE
                        * (float)CALIB_Z_SIGN;
            Chassis_Rotate(angle, v_max, dt_ms);
        }

        /* ---- 8. 再纠 XY (平移) ---- */
        if (abs_x > POSITION_TOLERANCE_PX) {
            float mm_x = (float)err_x * POSITION_SCALE_MM_PER_PX
                       * (float)POSITION_CAMX_SIGN;
            Chassis_PosMove(mm_x, POSITION_CAMX_AXIS, v_max, dt_ms);
        }
        if (abs_y > POSITION_TOLERANCE_PX) {
            float mm_y = (float)err_y * POSITION_SCALE_MM_PER_PX
                       * (float)POSITION_CAMY_SIGN;
            Chassis_PosMove(mm_y, POSITION_CAMY_AXIS, v_max, dt_ms);
        }

    } /* for iter */

    return -2;  /* 达到最大迭代次数 */
}

/**
 * @brief  等待 Jetson 返回色环坐标, 带超时
 * @param  timeout_ms  超时时间 (ms)
 * @return  0  收到 JETSON_FLAG_RING
 *         -1  超时
 */
static int Position_WaitRing(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (!JetsonData_IsUpdated(JETSON_FLAG_RING)) {
        JetsonData_Process();

        if (HAL_GetTick() - start >= timeout_ms) {
            return -1;
        }
    }
    return 0;
}

/**
 * @brief  色环对准: 迭代闭环将指定颜色色环移至图像中心
 */
int Position_AlignToRing(uint8_t ring_color, uint16_t v_max, uint16_t dt_ms)
{
    int16_t prev_abs_x = INT16_MAX;
    int16_t prev_abs_y = INT16_MAX;

    for (int iter = 0; iter < POSITION_MAX_ITER; iter++) {

        /* ---- 1. 请求 Jetson 识别色环 ---- */
        JetsonData_ClearFlags();
        JetsonData_SendTask(JETSON_TASK_RING, ring_color);

        /* ---- 2. 等待色环坐标 ---- */
        if (Position_WaitRing(POSITION_TIMEOUT_MS) != 0) {
            return -1;
        }

        /* ---- 3. 读取色环坐标 ---- */
        int16_t rx = (int16_t)JetsonData_GetRingX();
        int16_t ry = (int16_t)JetsonData_GetRingY();
        JetsonData_ClearFlags();

        /* ---- 4. 计算像素偏差 ---- */
        int16_t err_x = (int16_t)POSITION_TARGET_X - rx;
        int16_t err_y = (int16_t)POSITION_TARGET_Y - ry;

        int16_t abs_x = (err_x >= 0) ? err_x : -err_x;
        int16_t abs_y = (err_y >= 0) ? err_y : -err_y;

        /* ---- 5. 到位判断 ---- */
        if (abs_x <= POSITION_TOLERANCE_PX &&
            abs_y <= POSITION_TOLERANCE_PX) {
            return 0;
        }

        /* ---- 6. 发散检测 ---- */
        if (iter > 0) {
            if (abs_x > prev_abs_x * 2 || abs_y > prev_abs_y * 2) {
                return -3;
            }
        }
        prev_abs_x = abs_x;
        prev_abs_y = abs_y;

        /* ---- 7. 像素 → mm ---- */
        float mm_x = (float)err_x * POSITION_SCALE_MM_PER_PX
                   * (float)POSITION_CAMX_SIGN;
        float mm_y = (float)err_y * POSITION_SCALE_MM_PER_PX
                   * (float)POSITION_CAMY_SIGN;

        /* ---- 8. 移动底盘 ---- */
        if (abs_x > POSITION_TOLERANCE_PX) {
            Chassis_PosMove(mm_x, POSITION_CAMX_AXIS, v_max, dt_ms);
        }
        if (abs_y > POSITION_TOLERANCE_PX) {
            Chassis_PosMove(mm_y, POSITION_CAMY_AXIS, v_max, dt_ms);
        }

    } /* for iter */

    return -2;
}
