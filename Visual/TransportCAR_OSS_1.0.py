"""
=============================================================================
2025年中国大学生工程实践与创新能力大赛（工创赛）
智能物流搬运赛道 -- 西安理工大学 -- 塔吊结构 -- 视觉部分开源
时间：2025/8/14  版本号：1.0
=============================================================================
系统架构概览：
  摄像头 ──> OpenCV视觉识别 ──> 卡尔曼滤波平滑 ──> 串口发送坐标至MCU
    │                              │
    └── 三个独立卡尔曼滤波器         └── UART接收线程(unit/unit_target任务码)

主要功能模块：
  1. 卡尔曼滤波：三路独立滤波器，平滑坐标测量噪声
  2. 串口通信：后台线程接收MCU指令，主线程发送坐标数据
  3. 颜色识别：HSV阈值 + 轮廓检测，识别红/绿/蓝物料色块
  4. 色环定位：CLAHE增强 + 霍夫圆检测，定位三个色环
  5. 颜色排序：按X坐标排序确定物料左右顺序
  6. 二维码识别：OpenCV QR检测器解码二维码
  7. 状态机：根据unit任务码(1-4)切换不同竞赛场景

依赖库安装：pip install opencv-python matplotlib numpy pyserial pyzbar
=============================================================================
"""

# ============================================================================
# 导入依赖库
# ============================================================================
import cv2
import numpy as np
import serial
import threading
import serial.tools.list_ports
from pyzbar.pyzbar import decode

# ============================================================================
# 模块1：卡尔曼滤波器初始化（三路独立滤波器）
# 用途：对视觉识别的坐标进行时域平滑滤波，减少测量噪声和抖动
# 状态量：[x, y]（2维位置），观测量：[x, y]（2维测量）
# 三个滤波器分别用于 tracking 不同目标（三个色块/色环）
# ============================================================================

# --- 1.1 创建三个卡尔曼滤波器实例 ---
# 参数含义：KalmanFilter(状态维度=2, 测量维度=2)
kalman = cv2.KalmanFilter(2, 2)
kalman_2 = cv2.KalmanFilter(2, 2)
kalman_3 = cv2.KalmanFilter(2, 2)

# --- 1.2 测量矩阵 H：将状态空间映射到观测空间（单位矩阵 = 直接观测位置）---
kalman.measurementMatrix = np.array([[1, 0], [0, 1]], np.float32)
kalman_2.measurementMatrix = np.array([[1, 0], [0, 1]], np.float32)
kalman_3.measurementMatrix = np.array([[1, 0], [0, 1]], np.float32)

# --- 1.3 状态转移矩阵 F：预测下一时刻状态（单位矩阵 = 匀速/静态模型）---
kalman.transitionMatrix = np.array([[1, 0], [0, 1]], np.float32)
kalman_2.transitionMatrix = np.array([[1, 0], [0, 1]], np.float32)
kalman_3.transitionMatrix = np.array([[1, 0], [0, 1]], np.float32)

# --- 1.4 过程噪声协方差 Q：系统模型的不确定性（值越小越信任模型预测）---
kalman.processNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 1e-3
kalman_2.processNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 1e-3
kalman_3.processNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 1e-3

# --- 1.5 测量噪声协方差 R：传感器测量的不确定性（值越小越信任测量值）---
kalman.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 0.01
kalman_2.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 0.01
kalman_3.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 0.01

# --- 1.6 初始状态估计：卡尔曼滤波的起始位置猜测 ---
kalman.statePre = np.array([[6], [6]], np.float32)
kalman_2.statePre = np.array([[6], [6]], np.float32)
kalman_3.statePre = np.array([[6], [6]], np.float32)

# --- 1.7 历史值缓存变量：存储上一次的测量值和预测值 ---
last_measurement = current_measurement = np.array((2, 2), np.float32)
last_prediction = current_prediction = np.array((2, 2), np.float32)

last_measurement_2 = current_measurement_2 = np.array((2, 2), np.float32)
last_prediction_2 = current_prediction_2 = np.array((2, 2), np.float32)

last_measurement_3 = current_measurement_3 = np.array((2, 2), np.float32)
last_prediction_3 = current_prediction_3 = np.array((2, 2), np.float32)


def kalman_filter(measured_value):
    """
    卡尔曼滤波器1 —— 单目标跟踪用
    :param measured_value: 当前帧测量到的坐标 np.array([[x], [y]], dtype=np.float32)
    :return: 滤波平滑后的预测坐标 np.array([[x], [y]])
    """
    global kalman, last_measurement, current_measurement, last_prediction, current_prediction

    last_measurement = current_measurement
    last_prediction = current_prediction

    kalman.correct(measured_value)      # 修正步骤：用测量值更新滤波器状态
    current_prediction = kalman.predict()  # 预测步骤：预测下一时刻状态

    return current_prediction


def kalman_filter_2(measured_value):
    """
    卡尔曼滤波器2 —— 多目标跟踪用（第二个目标）
    :param measured_value: 当前帧测量到的坐标 np.array([[x], [y]], dtype=np.float32)
    :return: 滤波平滑后的预测坐标 np.array([[x], [y]])
    """
    global kalman_2, last_measurement_2, current_measurement_2, last_prediction_2, current_prediction_2

    last_measurement_2 = current_measurement_2
    last_prediction_2 = current_prediction_2

    kalman_2.correct(measured_value)
    current_prediction_2 = kalman_2.predict()

    return current_prediction_2


def kalman_filter_3(measured_value):
    """
    卡尔曼滤波器3 —— 多目标跟踪用（第三个目标）
    :param measured_value: 当前帧测量到的坐标 np.array([[x], [y]], dtype=np.float32)
    :return: 滤波平滑后的预测坐标 np.array([[x], [y]])
    """
    global kalman_3, last_measurement_3, current_measurement_3, last_prediction_3, current_prediction_3

    last_measurement_3 = current_measurement_3
    last_prediction_3 = current_prediction_3

    kalman_3.correct(measured_value)
    current_prediction_3 = kalman_3.predict()

    return current_prediction_3

# ============================================================================
# 模块2：串口通信（与下位机MCU的UART通信协议）
# ============================================================================
# 串口协议说明：
#   接收帧（MCU → PC, 4字节）：[0x66, unit, unit_target, 0xFF]
#     - receive[0] = 0x66 帧头, receive[3] = 0xFF 帧尾校验位，双重校验通过才更新任务码
#   发送帧（PC → MCU, 15字节）：[0x66, 命令码, Data0_H, Data0_L, Data1_H, Data1_L,
#                               Data2_H, Data2_L, Data3_H, Data3_L, Data4_H, Data4_L, ...]
#     - 帧尾常以0x77作为结束标志
#     - 坐标采用大端序(Big-Endian)拆分：高位在前 (coord >> 8) & 0xFF, 低位在后 coord & 0xFF
#   物理层：CH340 USB转TTL, 波特率115200, 超时0.05s
# ============================================================================

# --- 2.1 接收/发送缓冲区及状态变量 ---
receive = [0, 0, 0, 0]  # 接收缓冲区 [帧头0x66, unit, unit_target, 校验0xFF]
send = [0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
unit = 0           # 当前任务场景号（由MCU通过串口下发）
unit_target = 1    # 当前场景下的子任务参数（由MCU通过串口下发）

fps = 0            # 帧率计数器（调试用）


def uart_process():
    """
    串口接收线程函数 —— 后台持续读取MCU指令
    协议：4字节一帧 [0x66, unit, unit_target, 0xFF]
    帧头 0x66 + 帧尾 0xFF 双重校验通过后才更新任务码
    """
    global receive
    global unit
    global unit_target

    while True:
        input = uart.read(4)
        com_input = list(input)
        if com_input:
            print(com_input)
            try:
                receive[0] = int(com_input[0])
                receive[1] = int(com_input[1])
                receive[2] = int(com_input[2])
                receive[3] = int(com_input[3])
            except:
                receive = [0, 0, 0, 0]
            print(receive)
            if (receive[0] == 0x66 and receive[3] == 0xFF):  # 帧头+帧尾双重校验
                unit = receive[1]                             # 场景号
                unit_target = receive[2]                      # 子任务参数
            else:
                None


# --- 2.2 初始化串口并启动接收线程 ---
# Jetson Nano使用CH340 USB转TTL模块，设备路径为 /dev/ttyCH341USB0
# 如果在Windows PC上调试，请注释下面一行，启用COM口那一行
uart = serial.Serial(port="/dev/ttyCH341USB0", baudrate=115200, timeout=0.05)
# uart = serial.Serial(port="COM7", baudrate=115200, timeout=0.05)  # Windows调试用
serial_thread = threading.Thread(target=uart_process)
serial_thread.daemon = True   # 守护线程：主程序退出时自动结束
serial_thread.start()

# ============================================================================
# 模块3：视觉识别函数
# ============================================================================

# --- 3.1 HSV颜色阈值定义 ---
# 红色在HSV空间中跨越了0度边界，需要两个区间：[156~180] 和 [0~6]
# 蓝色和绿色只有一个连续区间
color_dist = {
    'red': {
        'Lower1': np.array([156, 60, 60]), 'Upper1': np.array([180, 255, 255]),
        'Lower2': np.array([0, 60, 60]),   'Upper2': np.array([6, 255, 255])
    },
    'blue': {
        'Lower': np.array([100, 100, 45]), 'Upper': np.array([124, 255, 255])
    },
    'green': {
        'Lower': np.array([38, 80, 45]),   'Upper': np.array([90, 255, 255])
    },
}


def color_blocks_position_WL(img, color, size_code):
    """
    物料颜色识别 —— 借助HSV颜色阈值筛选识别指定颜色的物料色块
    处理流程：高斯模糊 → BGR转HSV → 腐蚀 → inRange阈值筛选 → 找轮廓 → 面积筛选 → 返回中心

    :param img: 输入图像 (BGR格式)
    :param color: 目标颜色名称 ('red' / 'green' / 'blue')
    :param size_code: 最小面积阈值，小于此面积的色块被过滤掉
    :return: 色块中心坐标 (center_x, center_y) 或 None
    """
    ball_color = color
    if img is not None:
        gs_img = cv2.GaussianBlur(img, (5, 5), 0)                       # 高斯模糊：去除高频噪声
        hsv_img = cv2.cvtColor(gs_img, cv2.COLOR_BGR2HSV)               # 转为HSV空间便于颜色分割
        erode_hsv = cv2.erode(hsv_img, None, iterations=2)              # 腐蚀：消除小噪点，平整边缘
        inRange_hsv = None

        # 红色在HSV空间中跨越0度，分两段分别阈值处理后合并
        if (ball_color == 'red'):
            inRange_hsv1 = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower1'], color_dist[ball_color]['Upper1'])
            res1 = cv2.bitwise_and(erode_hsv, erode_hsv, mask=inRange_hsv1)
            inRange_hsv2 = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower2'], color_dist[ball_color]['Upper2'])
            res2 = cv2.bitwise_and(erode_hsv, erode_hsv, mask=inRange_hsv2)
            inRange_hsv = inRange_hsv1 + inRange_hsv2                    # 合并两个阈值区间
        else:
            inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])

        cv2.imshow("end2", erode_hsv)
        cv2.imshow("end", inRange_hsv)

        cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # 查找外轮廓
        c = max(cnts, key=cv2.contourArea)                              # 取面积最大的轮廓
        size = int(cv2.contourArea(c))
        print(size)

        if (size > size_code):                                          # 面积限幅：过滤小面积误检
            rect = cv2.minAreaRect(c)                                   # 最小外接矩形
            box = cv2.boxPoints(rect)
            cv2.drawContours(img0, [np.int0(box)], -1, (0, 255, 255), 2)  # 在原图上绘制识别框
            center_x, center_y = rect[0]
            return (int(center_x), int(center_y))                       # 返回色块中心坐标
        else:
            pass
    else:
        print("无画面")


def color_circle_position(img):
    """
    色环定位 —— 通过霍夫圆检测识别三个色环，实现精确定位
    处理流程：
      腐蚀 → 膨胀 → 灰度化 → CLAHE自适应直方图均衡 → 形态学梯度 → 高斯模糊 →
      对比度增强 → 高斯模糊 → 二值化 → 高斯模糊 → 霍夫圆检测 → X坐标排序

    设计意图：色环比色块更稳定，用于竞赛场地的精确定位参考

    :param img: 输入图像 (BGR格式)
    :return: 三个色环的中心坐标 (x1, y1, x2, y2, x3, y3)，按X坐标升序排列
    """
    erode_hsv = cv2.erode(img, None, iterations=2)                     # 腐蚀：去噪（粗变细）
    kernel = np.ones((7, 7), np.uint8)
    diRange_hsv = cv2.dilate(erode_hsv, kernel, 1)                     # 膨胀：填补空洞
    gray_img = cv2.cvtColor(diRange_hsv, cv2.COLOR_BGR2GRAY)          # 转单通道灰度

    # CLAHE 自适应直方图均衡：显著增强对光照变化的鲁棒性
    # clipLimit=5.0 限制对比度放大倍数，过大易产生噪点
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(8, 8))
    clahed = clahe.apply(gray_img)

    # 形态学梯度：膨胀 - 腐蚀，增强物体边缘
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    gradient = cv2.morphologyEx(gray_img, cv2.MORPH_GRADIENT, kernel)

    result = cv2.GaussianBlur(gradient, (7, 7), 3, 3)                 # 高斯模糊：平滑边缘

    eqal_img = cv2.convertScaleAbs(result, alpha=4, beta=0)           # 对比度整体增强 (乘alpha+加beta)
    cv2.imshow("video2", eqal_img)
    eqal_img = cv2.GaussianBlur(eqal_img, (7, 7), 3, 3)              # 再次高斯模糊

    retval, threshold_img = cv2.threshold(eqal_img, 70, 255, cv2.THRESH_BINARY)  # 二值化，阈值70

    threshold_img = cv2.GaussianBlur(threshold_img, (9, 9), 3, 3)    # 最后的高斯模糊

    # 霍夫圆检测 (HOUGH_GRADIENT_ALT 模式)
    # param1=100 边缘检测阈值, param2=0.95 圆心检测阈值, minRadius=15, maxRadius=50
    circles = cv2.HoughCircles(threshold_img, cv2.HOUGH_GRADIENT_ALT, 1.5, 50,
                               param1=100, param2=0.95, minRadius=15, maxRadius=50)
    cv2.imshow("video", gray_img)
    cv2.imshow("video3", threshold_img)

    try:
        if (len(circles[0]) == 3):                                      # 必须恰好检测到3个色环才有效
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                cv2.circle(img, (circle[0], circle[1]), circle[2], (0, 0, 255), 2)   # 画圆
                cv2.circle(img, (circle[0], circle[1]), 2, (255, 0, 0), 2)           # 圆心
            circle_all = [circles[0][0], circles[0][1], circles[0][2]]
            circle_list = sorted(circle_all, key=lambda x: x[0])       # 按X坐标升序排序
            return (circle_list[0][0], circle_list[0][1],
                    circle_list[1][0], circle_list[1][1],
                    circle_list[2][0], circle_list[2][1])              # 返回 X1,Y1,X2,Y2,X3,Y3
    except:
        pass


# --- 3.3 颜色排序与顺序判断 ---

def Judgeposition(qx1, qx2, qx3):
    """
    三目标X坐标排序 —— 返回按X升序的索引列表
    决赛时在粗加工区识别三种颜色物料，按左右位置排序

    :param qx1: 第一个目标的X坐标
    :param qx2: 第二个目标的X坐标
    :param qx3: 第三个目标的X坐标
    :return: 按X值升序排列的原始索引列表，如 [2,1,0] 表示 qx3 < qx2 < qx1
    """
    sorted_indices = sorted([0, 1, 2], key=lambda i: [qx1, qx2, qx3][i])
    return sorted_indices


def Judgeorder(qx1, qx2, qx3):
    """
    颜色顺序码确定 —— 根据三目标X坐标排序确定从左到右的颜色排列
    用于决赛轮次直接下发颜色顺序给下位机按序抓取

    颜色对应关系（代码中固定为 BGR）：
      索引0=Blue, 索引1=Green, 索引2=Red

    顺序码定义：
      0x01=RGB, 0x02=RBG, 0x03=GRB, 0x04=GBR, 0x05=BRG, 0x06=BGR

    :param qx1: Blue的X坐标
    :param qx2: Green的X坐标
    :param qx3: Red的X坐标
    :return: 颜色顺序码 (0x01 ~ 0x06)
    """
    sorted_indices = Judgeposition(qx1, qx2, qx3)
    if sorted_indices == [2, 1, 0]:      # Red(qx3) < Green(qx2) < Blue(qx1) → 从左到右: RGB
        return 0x01
    elif sorted_indices == [2, 0, 1]:    # RBG
        return 0x02
    elif sorted_indices == [1, 2, 0]:    # GRB
        return 0x03
    elif sorted_indices == [1, 0, 2]:    # GBR
        return 0x04
    elif sorted_indices == [0, 2, 1]:    # BRG
        return 0x05
    elif sorted_indices == [0, 1, 2]:    # BGR
        return 0x06


# --- 3.4 二维码可视化 ---

def display(img, bbox):
    """
    在图像上绘制二维码的边界框和中心点（调试可视化用）

    :param img: 输入图像
    :param bbox: 二维码边界框坐标数组，形状 (4, 2)
    """
    if bbox is None:
        print("边界框为空，无法绘制")
        return

    bbox = bbox.astype(int)
    print("Boundary Box Coordinates:", bbox)

    if len(bbox) == 1:
        bbox = bbox[0]

    n = len(bbox)
    for j in range(n):
        pt1 = tuple(bbox[j])
        pt2 = tuple(bbox[(j + 1) % n])
        cv2.line(img, pt1, pt2, (255, 0, 0), 3)                      # 蓝色边框

    center_x = int(np.mean(bbox[:, 0]))
    center_y = int(np.mean(bbox[:, 1]))
    cv2.circle(img, (center_x, center_y), 5, (0, 255, 0), -1)        # 绿色圆心

# ============================================================================
# 模块4：主程序 —— 竞赛场景状态机
# ============================================================================
# 根据串口接收的 unit 任务码切换不同场景，每个场景处理流程为：
#   摄像头采集 → 视觉识别 → 卡尔曼滤波 → 串口发送坐标/指令到MCU
#
# 发送帧格式说明：
#   send[0]  = 0x66 (同步头)
#   send[1]  = 命令码 (对应unit)
#   send[2..] = 数据载荷（坐标、差值等），大端序拆分为高/低字节
#   最后有效字节 = 0x77 (帧尾)
#
# unit场景定义：
#   unit=1: 物料盘定位 ─ 识别RGB三色块，发送三色块平均中心坐标给MCU
#   unit=2: 物料识别   ─ 根据unit_target识别指定颜色物料，发送其坐标
#   unit=3: 色环定位   ─ 识别三色环，发送中间色环坐标+Y差值做姿态校准
#   unit=4: 二维码识别 ─ 解码二维码内容发送给MCU
#   else:   空闲状态   ─ 发送默认心跳包 0xAA 0x03 0x77
# ============================================================================

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)       # 默认使用摄像头0（主摄像头）

    while True:
        success, img0 = cap.read()

        if success:
            # ================================================================
            # unit=1: 物料盘识别定位
            # 同时识别RGB三种颜色的物料色块，取三色块中心坐标的平均值
            # 作为物料盘位置发送给下位机
            # 发送帧：[0x66, 0x01, X_H, X_L, Y_H, Y_L, 0x77]
            # ================================================================
            if (unit == 1):
                try:
                    poz_x1, poz_y1 = color_blocks_position_WL(img0, 'red', 2000)
                    poz_x2, poz_y2 = color_blocks_position_WL(img0, 'green', 2000)
                    poz_x3, poz_y3 = color_blocks_position_WL(img0, 'blue', 2000)
                    poz_x0 = int((poz_x1 + poz_x2 + poz_x3) / 3)       # 三色块X平均
                    poz_y0 = int((poz_y1 + poz_y2 + poz_y3) / 3)       # 三色块Y平均
                    measured_x = poz_x0
                    measured_y = poz_y0
                    z = np.array([[measured_x], [measured_y]], dtype=np.float32)
                    x = kalman_filter(z)                                # 卡尔曼滤波平滑
                    qx0 = int(x[0][0])
                    qy0 = int(x[1][0])
                    send[1] = 0x01
                    send[2] = (qx0 & 0xff00) >> 8                      # X坐标高字节
                    send[3] = (qx0 & 0xff)                              # X坐标低字节
                    send[4] = (qy0 & 0xff00) >> 8                      # Y坐标高字节
                    send[5] = (qy0 & 0xff)                              # Y坐标低字节
                    send[6] = 0x77                                      # 帧尾

                    FH = bytearray(send)
                    write_len = uart.write(FH)
                    print(send)
                except:
                    pass

            # ================================================================
            # unit=2: 物料识别
            # 根据 unit_target 识别指定颜色的单个物料
            #   unit_target=0x01:红色, =0x02:绿色, =0x03:蓝色
            # 发送帧：[0x66, 0x02, color_code, X_H, X_L, Y_H, Y_L, 0x77]
            # ================================================================
            elif (unit == 2):
                try:
                    if (unit_target == 0x01):
                        poz_x0, poz_y0 = color_blocks_position_WL(img0, 'red', 4000)
                        send[2] = 0x01
                    elif (unit_target == 0x02):
                        poz_x0, poz_y0 = color_blocks_position_WL(img0, 'green', 4000)
                        send[2] = 0x02
                    elif (unit_target == 0x03):
                        poz_x0, poz_y0 = color_blocks_position_WL(img0, 'blue', 4000)
                        send[2] = 0x03
                    else:
                        pass
                    measured_x = poz_x0
                    measured_y = poz_y0
                    z = np.array([[measured_x], [measured_y]], dtype=np.float32)
                    x = kalman_filter(z)
                    qx0 = int(x[0][0])
                    qy0 = int(x[1][0])

                    send[1] = 0x02
                    send[3] = (qx0 & 0xff00) >> 8                       # X坐标高字节
                    send[4] = (qx0 & 0xff)                              # X坐标低字节
                    send[5] = (qy0 & 0xff00) >> 8                       # Y坐标高字节
                    send[6] = (qy0 & 0xff)                              # Y坐标低字节
                    send[7] = 0x77
                    FH = bytearray(send)
                    write_len = uart.write(FH)
                    print(send)
                except:
                    pass

            # ================================================================
            # unit=3: 色环定位
            # 利用三个色环实现精确定位：发送中间色环坐标 + 左右色环Y差值
            # 左右色环Y差值用于矫正塔吊朝向角度（Y差≈0代表正对色环）
            # 发送帧：[0x66, 0x03, X_H, X_L, Y_H, Y_L, dY_H, dY_L, 0x77]
            # ================================================================
            elif (unit == 3):
                try:
                    poz_x1, poz_y1, poz_x2, poz_y2, poz_x3, poz_y3 = color_circle_position(img0)
                    measured_x1 = poz_x1
                    measured_y1 = poz_y1

                    measured_x2 = poz_x2
                    measured_y2 = poz_y2

                    measured_x3 = poz_x3
                    measured_y3 = poz_y3

                    z1 = np.array([[measured_x1], [measured_y1]], dtype=np.float32)
                    x1 = kalman_filter(z1)
                    qx1 = int(x1[0][0])
                    qy1 = int(x1[1][0])

                    z2 = np.array([[measured_x2], [measured_y2]], dtype=np.float32)
                    x2 = kalman_filter_2(z2)
                    qx2 = int(x2[0][0])
                    qy2 = int(x2[1][0])

                    z3 = np.array([[measured_x3], [measured_y3]], dtype=np.float32)
                    x3 = kalman_filter_3(z3)
                    qx3 = int(x3[0][0])
                    qy3 = int(x3[1][0])

                    send[1] = 0x03
                    send[2] = (qx2 & 0xff00) >> 8                       # 中间色环X (按X排序的中间那个)
                    send[3] = (qx2 & 0xff)
                    send[4] = (qy2 & 0xff00) >> 8                       # 中间色环Y
                    send[5] = (qy2 & 0xff)
                    send[6] = ((qy1 - qy3) & 0xff00) >> 8              # 左右色环Y差值高位
                    send[7] = ((qy1 - qy3) & 0xff)                      # 左右色环Y差值低位
                    send[8] = 0x77
                    print(qy1 - qy3)
                    FH = bytearray(send)
                    write_len = uart.write(FH)
                    print(send)
                except:
                    pass

            # ================================================================
            # unit=4: 二维码识别
            # 使用OpenCV QR检测器读取二维码内容
            # 将解码得到的字符串逐字符以int形式发送给MCU
            # 发送帧：[0x66, 0x04, ch0, ch1, ch2, ch3, ch4, ch5, ch6, 0x77]
            # ================================================================
            elif (unit == 4):
                qrDecoder = cv2.QRCodeDetector()

                if not cap.isOpened():
                    print("无法打开摄像头")

                else:
                    try:
                        success, img0 = cap.read()
                        if not success:
                            print("无法读取摄像头图像")
                            continue

                        data, bbox, _ = qrDecoder.detectAndDecode(img0)

                        if data and bbox is not None:
                            print(f"检测到二维码: 数据 = '{data}'")

                            display(img0, bbox)                         # 绘制检测结果
                            cv2.imshow("Results", img0)

                            send[1] = 0x04
                            send[2] = int(data[0])                      # 字符逐个发送
                            send[3] = int(data[1])
                            send[4] = int(data[2])
                            send[5] = int(data[4])
                            send[6] = int(data[5])
                            send[7] = int(data[6])
                            send[8] = 0x77
                            print(send)
                            FH = bytearray(send)
                            write_len = uart.write(FH)

                        else:
                            print("未检测到二维码")
                            cv2.imshow("Results", img0)
                    except:
                        print("error")

            # ================================================================
            # 默认/空闲状态：发送心跳包告知MCU视觉系统就绪
            # ================================================================
            else:
                send[1] = 0xAA
                send[2] = 0X03
                send[3] = 0x77
                FH = bytearray(send)
                write_len = uart.write(FH)
                print(send)

            cv2.imshow("videoo", img0)                                  # 显示主摄像头画面
        else:
            break

        m_key = cv2.waitKey(1) & 0xFF

        if m_key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
