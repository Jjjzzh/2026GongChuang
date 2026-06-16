/**
  ******************************************************************************
  * @file    Position.h
  * @brief   物料视觉定位模块
  * @note
  *          - 依赖: Chassis (底盘移动) + JetsonData (图像坐标)
  *          - 控制策略: 迭代闭环, 每次移动后重新检测, 直至误差收敛
  *          - 发散检测: 误差扩大 2× 以上自动终止
  *
  *          坐标映射 (由相机安装方向决定):
  *            图像右 (+X) → 车尾 (-X),  tray_x → 底盘 X 轴, 反向
  *            图像上 (-Y) → 车右 (-Y),  tray_y → 底盘 Y 轴, 同向 (下=左)
  ******************************************************************************
  */
#ifndef __POSITION_H__
#define __POSITION_H__

#include <stdint.h>
#include "Chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 预设目标坐标 (摄像头像素坐标系)
 *
 * 物料盘应被移动到的目标图像位置
 * 默认 (320, 240) = 640×480 图像中心
 * 根据实际相机安装和抓取位置标定后修改
 * ============================================================ */
#define POSITION_TARGET_X       320
#define POSITION_TARGET_Y       240

/* ============================================================
 * 像素 → 毫米换算系数
 *
 * 取决于相机安装高度和视场角 (FOV)
 * 典型值: 640×480 @ 30cm 高度 ≈ 200mm 视场 → 0.31 mm/px
 * 实测标定后修改
 * ============================================================ */
#define POSITION_SCALE_MM_PER_PX    0.3f

/* ============================================================
 * 轴映射: 摄像头像素轴 → 底盘运动轴
 *
 *   由相机安装方向决定:
 *     图像右 (+X) = 车尾 (-X)  → tray_x 映射底盘 X 轴, SIGN=-1
 *     图像下 (+Y) = 车左 (+Y)  → tray_y 映射底盘 Y 轴, SIGN=+1
 *
 *   POSITION_CAMX_SIGN / POSITION_CAMY_SIGN:
 *     +1 = err>0 时向底盘正方向移动
 *     -1 = err>0 时向底盘负方向移动
 * ============================================================ */
#define POSITION_CAMX_AXIS    CHASSIS_DIR_X   /* 图像水平 → 底盘前后 (右=后) */
#define POSITION_CAMX_SIGN    (-1)            /* camera+X = chassis-X      */
#define POSITION_CAMY_AXIS    CHASSIS_DIR_Y   /* 图像垂直 → 底盘左右 (下=左) */
#define POSITION_CAMY_SIGN    1               /* camera+Y = chassis+Y      */

/* ============================================================
 * 定位控制参数 (物料定位 & 车身校准共用)
 * ============================================================ */
#define POSITION_TOLERANCE_PX   3       /* XY 到位容差 (像素) */
#define POSITION_MAX_ITER       5       /* 最大迭代次数 */
#define POSITION_TIMEOUT_MS     3000    /* 等待 Jetson 响应超时 (ms) */

/* ============================================================
 * 车身校准 — XY 预设目标坐标 (独立于物料定位)
 *
 * 车身校准时色环中心应处于的图像位置
 * 根据色环在图像中的期望位置标定后修改
 * ============================================================ */
#define CALIB_TARGET_X          320     /* 色环中心 X */
#define CALIB_TARGET_Y          240     /* 色环中心 Y */

/* ============================================================
 * 车身校准 — Z 轴 (旋转) 参数
 *
 * calib_z = 右环Y - 左环Y (像素)
 * Z ≠ 0 表示车身存在旋转偏差, 需通过原地转弯消除
 *
 * ── 方向符号推导 (由相机安装方向决定) ──
 *   已知: 图像下 (+Y) = 车左, 图像上 (-Y) = 车右
 *
 *   车身 CCW (正偏航) 时:
 *     右环 (车右) 相对相机前移 → 图像中上移 (Y↓)
 *     左环 (车左) 相对相机后移 → 图像中下移 (Y↑)
 *     → calib_z = 右环Y - 左环Y < 0
 *
 *   修正: calib_z < 0 → 需 CW (负角度) 转回
 *     angle = calib_z × SCALE × SIGN
 *           = 负值    × 0.5   × (-1)  = 正值 (CCW) ✓
 *   → CALIB_Z_SIGN = -1
 *
 * CALIB_Z_TO_ANGLE: calib_z 每像素对应的旋转角度 (°)
 *   取决于两色环之间的水平像素间距
 *   间距 ≈ 120px 时, 1px Y差 ≈ atan(1/120) ≈ 0.48° ≈ 0.5°
 * ============================================================ */
#define CALIB_Z_TO_ANGLE        0.5f    /* °/px */
#define CALIB_Z_SIGN            (-1)    /* 由相机方向导出, 无需标定 */
#define CALIB_Z_TOLERANCE_PX    2       /* Z 轴到位容差 (像素) */

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief  物料盘视觉定位: 迭代闭环对准预设目标
 * @param  v_max  底盘峰值转速 (RPM), 典型 200~400
 * @param  dt_ms  底盘控制周期 (ms), 典型 10~30
 * @return  0  已到位 (误差 ≤ 容差)
 *         -1  超时 (Jetson 未响应)
 *         -2  超过最大迭代次数 (可能摩擦/视野问题)
 *         -3  发散 (误差增大, 方向映射可能错误)
 *
 * @note   阻塞调用, 整个定位过程耗时取决于误差大小和迭代次数
 *
 * 使用示例:
 *   int ret = Position_AlignToTray(300, 20);
 *   if (ret == 0) {
 *       // 定位成功, 可以抓取
 *   } else {
 *       // ret = -1/-2/-3, 查错
 *   }
 */
int Position_AlignToTray(uint16_t v_max, uint16_t dt_ms);

/**
 * @brief  车身校准: 三轴迭代闭环 (Z旋转 + XY平移)
 * @param  v_max  底盘峰值转速 (RPM)
 * @param  dt_ms  底盘控制周期 (ms)
 * @return  0  校准完成 (Z/XY 均在容差内)
 *         -1  Jetson 超时无响应
 *         -2  超过最大迭代次数
 *         -3  发散 (方向映射可能错误)
 *
 * @note   阻塞调用
 *   每轮迭代顺序: 先纠正 Z (旋转), 再纠正 XY (平移)
 *   旋转会改变相机视野, 因此每轮都先转再移
 *
 *   依赖 Jetson 端响应 JETSON_TASK_CALIB (0x05),
 *   返回色环校准帧 [0x66,0x05, X_H,X_L, Y_H,Y_L, Z_H,Z_L, 0x99]
 *
 * 使用示例:
 *   int ret = Position_BodyCalib(200, 20);
 *   if (ret == 0) {
 *       // 车身已校准, 可开始物料定位
 *   }
 */
int Position_BodyCalib(uint16_t v_max, uint16_t dt_ms);

/**
 * @brief  色环对准: 迭代闭环将指定颜色色环移至图像中心
 * @param  ring_color  色环颜色: JETSON_RING_TASK_RED / _GREEN / _BLUE
 * @param  v_max       底盘峰值转速 (RPM)
 * @param  dt_ms       底盘控制周期 (ms)
 * @return  0  已对准 (色环在目标中心)
 *         -1  Jetson 超时无响应
 *         -2  超过最大迭代次数
 *         -3  发散 (方向映射错误)
 *
 * @note   阻塞调用
 *   发送 JETSON_TASK_RING + 颜色参数, 等待 Jetson 返回色环坐标,
 *   与 POSITION_TARGET_X/Y 比较, 迭代移动直至重合
 *
 * 使用示例:
 *   int ret = Position_AlignToRing(JETSON_RING_TASK_RED, 300, 20);
 *   if (ret == 0) {
 *       // 红色色环已在抓取位置
 *   }
 */
int Position_AlignToRing(uint8_t ring_color, uint16_t v_max, uint16_t dt_ms);

#ifdef __cplusplus
}
#endif

#endif /* __POSITION_H__ */
