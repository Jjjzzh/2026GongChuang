"""
=============================================================================
Jetson 视觉端 — 全部模块统一库
=============================================================================
工程：2026 工创赛 智能物流搬运 汕头大学 手算attention
用途：作为 Jetson 端所有功能模块的公共文件

包含模块：
  1. STM32TaskReceiver    — 接收 STM32 下发的任务码
  2. STM32ResponseSender  — 向 STM32 回传坐标/结果
  3. KalmanFilter2D       — OpenCV 卡尔曼滤波 2D 坐标平滑
  4. ColorDetector        — HSV 阈值颜色识别（红/绿/蓝）
  5. CircleDetector       — 霍夫圆检测色环定位
  6. QRScanner            — 二维码解码

协议（与 STM32 Gongchuang 工程配对）：
  接收帧（STM32 → PC, 4 字节）：[0x66, task_code, param, 0xFF]
  发送帧（PC → STM32, 变长）：[0x66, cmd, data..., 0x77]
  硬件：CH340 USB转TTL, 波特率 115200

依赖：pip install opencv-python numpy pyserial pyzbar
=============================================================================
"""

import cv2
import numpy as np
import serial
import threading
import time
import serial.tools.list_ports

# ============================================================================
# 公共常量：任务码 & 颜色码（与 STM32 jetsondata.h 保持一致）
# ============================================================================

TASK_SCAN     = 0x01   # 扫码任务
TASK_TRAY     = 0x02   # 识别物料盘
TASK_MATERIAL = 0x03   # 识别物料（按颜色）
TASK_RING     = 0x04   # 识别色环
TASK_CALIB    = 0x05   # 车身校准 (色环XY+Z)

COLOR_RED   = 0x01
COLOR_GREEN = 0x02
COLOR_BLUE  = 0x03

FRAME_HEADER_MCU = 0x66   # 接收帧头（STM32→PC）
FRAME_TAIL_MCU   = 0xFF   # 接收帧尾
FRAME_HEADER_PC  = 0x66   # 发送帧头（PC→STM32）
FRAME_TAIL_PC    = 0x99   # 发送帧尾


# ============================================================================
# 模块1：STM32TaskReceiver — 接收 STM32 下发的任务码
# ============================================================================

class STM32TaskReceiver:
    """
    STM32 任务码接收器（后台守护线程）

    用法:
        stm32 = STM32TaskReceiver(port="COM7")
        stm32.open()

        while True:
            task = stm32.get_task()
            if task:
                code, param = task
                if code == TASK_MATERIAL:
                    ...
    """

    FRAME_SIZE = 4

    _TASK_NAMES = {
        0x01: "SCAN     ", 0x02: "TRAY     ",
        0x03: "MATERIAL ", 0x04: "RING     ",
        0x05: "CALIB    ",
    }
    _COLOR_NAMES = {
        0x01: "RED  ", 0x02: "GREEN", 0x03: "BLUE ",
    }

    def __init__(self, port=None, baudrate=115200, timeout=0.05,
                 verbose=True):
        self._port_name = port or self._find_ch340()
        self._baudrate  = baudrate
        self._timeout   = timeout
        self._verbose   = verbose
        self._uart      = None
        self._rx_thread = None
        self._running   = False

        self._receive     = [0, 0, 0, 0]
        self._task_code   = 0
        self._task_param  = 0
        self._task_updated = False
        self.frame_count  = 0
        self.error_count  = 0

    # ---------- 生命周期 ----------
    def open(self):
        if self._running:
            return
        self._uart = serial.Serial(port=self._port_name,
                                   baudrate=self._baudrate,
                                   timeout=self._timeout)
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop,
                                           name="STM32_Rx", daemon=True)
        self._rx_thread.start()
        if self._verbose:
            print(f"[STM32Rx] {self._port_name} @ {self._baudrate} → 等待任务帧...")

    def close(self):
        self._running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self._uart and self._uart.is_open:
            self._uart.close()
        if self._verbose:
            print(f"[STM32Rx] 已关闭 (帧={self.frame_count} 错={self.error_count})")

    def __enter__(self):
        self.open(); return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # ---------- 任务读取 API ----------
    def get_task(self):
        """消费型读取: 返回 (code, param) 或 None"""
        if self._task_updated:
            self._task_updated = False
            return (self._task_code, self._task_param)
        return None

    def peek_task(self):
        """只读: 返回 (code, param) 不清标志"""
        return (self._task_code, self._task_param)

    @property
    def has_task(self):
        return self._task_updated

    @property
    def task_code(self):
        return self._task_code

    @property
    def task_param(self):
        return self._task_param

    def clear_task(self):
        self._task_updated = False

    @classmethod
    def task_name(cls, code):
        return cls._TASK_NAMES.get(code, f"0x{code:02X}")

    @classmethod
    def color_name(cls, code):
        return cls._COLOR_NAMES.get(code, f"0x{code:02X}")

    # ---------- 串口工具 ----------
    @staticmethod
    def _find_ch340():
        for p in serial.tools.list_ports.comports():
            hwid = p.hwid.upper()
            if "1A86" in hwid or "7523" in hwid:
                return p.device
        ports = list(serial.tools.list_ports.comports())
        return ports[0].device if ports else "COM7"

    @staticmethod
    def list_ports():
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}  [{p.hwid}]")

    # ---------- 接收线程 ----------
    def _rx_loop(self):
        while self._running:
            try:
                raw = self._uart.read(self.FRAME_SIZE)
                if not raw or len(raw) < self.FRAME_SIZE:
                    continue
                data = list(raw)
                self._receive[0] = int(data[0])
                self._receive[1] = int(data[1])
                self._receive[2] = int(data[2])
                self._receive[3] = int(data[3])

                if (self._receive[0] == FRAME_HEADER_MCU and
                    self._receive[3] == FRAME_TAIL_MCU):
                    self._task_code    = self._receive[1]
                    self._task_param   = self._receive[2]
                    self._task_updated = True
                    self.frame_count  += 1
                    if self._verbose:
                        self._log_frame()
                else:
                    self.error_count += 1
                    if self._verbose:
                        print(f"[STM32Rx] 校验失败 #{self.error_count} | "
                              f"raw={[hex(b) for b in data]}")
            except serial.SerialException as e:
                if self._verbose:
                    print(f"[STM32Rx] 串口异常: {e}")
                time.sleep(0.5)
            except Exception as e:
                if self._verbose:
                    print(f"[STM32Rx] 异常: {e}")

    def _log_frame(self):
        name = self._TASK_NAMES.get(self._task_code,
                                     f"0x{self._task_code:02X}   ")
        if self._task_code in (TASK_MATERIAL, TASK_RING):
            c = self._COLOR_NAMES.get(self._task_param,
                                       f"0x{self._task_param:02X}")
            print(f"[STM32Rx] #{self.frame_count} | {name}| {c}")
        else:
            print(f"[STM32Rx] #{self.frame_count} | {name}| "
                  f"param=0x{self._task_param:02X}")


# ============================================================================
# 模块2：STM32ResponseSender — 向 STM32 回传数据
# ============================================================================

class STM32ResponseSender:
    """STM32 数据发送器 —— 将视觉识别结果发送回 MCU"""

    def __init__(self, port="/dev/ttyCH341USB0", baudrate=115200):
        self.uart = serial.Serial(port=port, baudrate=baudrate, timeout=0.05)

    def send_qrcode(self, ch0, ch1, ch2, ch3, ch4, ch5):
        """二维码：[0x66, 0x01, ch0,ch1,ch2,ch3,ch4,ch5, 0x99]"""
        self.uart.write(bytearray([0x66, 0x01, ch0, ch1, ch2, ch3, ch4, ch5, 0x99]))

    def send_tray(self, x, y):
        """物料盘中心坐标：[0x66, 0x02, X_H,X_L, Y_H,Y_L, 0x99]"""
        self.uart.write(bytearray([0x66, 0x02,
                                    (x >> 8) & 0xFF, x & 0xFF,
                                    (y >> 8) & 0xFF, y & 0xFF,
                                    0x99]))

    def send_grab(self):
        """抓取指令：[0x66, 0x03, 0x01, 0x99]"""
        self.uart.write(bytearray([0x66, 0x03, 0x01, 0x99]))

    def send_ring(self, x, y):
        """目标色环坐标：[0x66, 0x04, X_H,X_L, Y_H,Y_L, 0x99]"""
        self.uart.write(bytearray([0x66, 0x04,
                                    (x >> 8) & 0xFF, x & 0xFF,
                                    (y >> 8) & 0xFF, y & 0xFF,
                                    0x99]))

    def send_ring_calib(self, x, y, z):
        """色环校准：[0x66, 0x05, X_H,X_L, Y_H,Y_L, Z_H,Z_L, 0x99]
        x=中间色环X  y=中间色环Y  z=右Y-左Y"""
        self.uart.write(bytearray([0x66, 0x05,
                                    (x >> 8) & 0xFF, x & 0xFF,
                                    (y >> 8) & 0xFF, y & 0xFF,
                                    (z >> 8) & 0xFF, z & 0xFF,
                                    0x99]))

    def close(self):
        self.uart.close()


# ============================================================================
# 模块3：KalmanFilter2D — 2D 坐标卡尔曼滤波平滑
# ============================================================================

class KalmanFilter2D:
    """
    OpenCV KalmanFilter 封装 —— 对视觉识别的 (x, y) 坐标进行时域平滑

    状态量：[x, y]  观测量：[x, y]
    模型：匀速/静态假设 (transitionMatrix = I)

    用法:
        kf = KalmanFilter2D()
        smoothed = kf.update(measured_x, measured_y)
    """

    def __init__(self, process_noise=1e-3, measure_noise=1e-2,
                 init_x=6.0, init_y=6.0):
        self._kf = cv2.KalmanFilter(2, 2)
        self._kf.measurementMatrix = np.eye(2, dtype=np.float32)
        self._kf.transitionMatrix  = np.eye(2, dtype=np.float32)
        self._kf.processNoiseCov   = np.eye(2, dtype=np.float32) * process_noise
        self._kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * measure_noise
        self._kf.statePre = np.array([[init_x], [init_y]], dtype=np.float32)

        self._last_measurement = np.zeros((2, 2), dtype=np.float32)
        self._last_prediction  = np.zeros((2, 2), dtype=np.float32)
        self._current_measurement = np.zeros((2, 2), dtype=np.float32)
        self._current_prediction  = np.zeros((2, 2), dtype=np.float32)

    def update(self, x, y):
        """
        输入当前帧测量坐标 (x, y)，返回平滑后的 (smooth_x, smooth_y)
        """
        self._last_measurement = self._current_measurement
        self._last_prediction  = self._current_prediction

        measured = np.array([[x], [y]], dtype=np.float32)
        self._kf.correct(measured)
        self._current_prediction = self._kf.predict()

        return (int(self._current_prediction[0][0]),
                int(self._current_prediction[1][0]))

    @property
    def raw_xy(self):
        return (self._current_prediction[0][0],
                self._current_prediction[1][0])

# ============================================================================
# 模块4：ColorDetector — 摄像头 + 颜色识别
# ============================================================================

class ColorDetector:
    """
    HSV 颜色识别器 —— 管理摄像头后台抓帧 + 红绿蓝三色定位

    用法:
        det = ColorDetector(camera_index=0)
        pos = det.get_color_positions()
        # pos → {"Red": (cx,cy) 或 None, "Green": ..., "Blue": ...}
        det.release()
    """

    HSV_RANGES = {
        "Red":   [((0, 100, 50),   (10, 255, 255)),
                  ((170, 100, 50), (180, 255, 255))],
        "Green": [((70, 80, 50),   (100, 255, 255))],
        "Blue":  [((100, 80, 50),  (130, 255, 255))],
    }
    MIN_AREA = 200
    KERNEL = np.ones((3, 3), np.uint8)

    def __init__(self, camera_index=0, width=1280, height=720):
        self._index = camera_index
        self._width = width
        self._height = height

        self._cap   = None
        self._frame = None
        self._fps   = 0.0
        self._lock  = threading.Lock()
        self._running = False
        self._thread  = None

        self._init_camera()

    # ---------- 摄像头初始化 ----------
    def _init_camera(self):
        cap = cv2.VideoCapture(self._index, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        assert cap.isOpened(), f"摄像头{self._index}打开失败"

        rw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        rh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rf = cap.get(cv2.CAP_PROP_FPS)
        print(f"摄像头已初始化: {rw}x{rh} @ {rf:.0f}fps")

        self._cap = cap
        ret, frame = cap.read()
        if ret:
            self._frame = frame

        self._running = True
        self._thread = threading.Thread(target=self._grab_loop,
                                         name="CamGrab", daemon=True)
        self._thread.start()

    # ---------- 后台抓帧线程 ----------
    def _grab_loop(self):
        count = 0
        t0 = time.perf_counter()
        while self._running:
            ret, frame = self._cap.read()
            if ret:
                count += 1
                with self._lock:
                    self._frame = frame
                    now = time.perf_counter()
                    if now - t0 >= 1.0:
                        self._fps = count / (now - t0)
                        count = 0
                        t0 = now
            else:
                time.sleep(0.001)

    # ---------- 颜色识别 ----------
    @classmethod
    def _make_mask(cls, hsv, ranges):
        mask = None
        for (h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi) in ranges:
            m = cv2.inRange(hsv, (h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi))
            mask = m if mask is None else mask | m
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cls.KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cls.KERNEL)
        return mask

    @staticmethod
    def _largest_centroid(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours
                 if cv2.contourArea(c) > ColorDetector.MIN_AREA]
        if not valid:
            return None
        cnt = max(valid, key=cv2.contourArea)
        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            return None
        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

    def get_color_positions(self):
        """
        读取后台最新帧，识别红绿蓝三色坐标。
        返回: {"Red": (cx,cy) 或 None, "Green": ..., "Blue": ...}
        """
        with self._lock:
            if self._frame is None:
                return {"Red": None, "Green": None, "Blue": None}
            frame = self._frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        result = {}
        for name in ("Red", "Green", "Blue"):
            mask = self._make_mask(hsv, self.HSV_RANGES[name])
            result[name] = self._largest_centroid(mask)

        return result

    # ---------- 生命周期 ----------
    def release(self):
        """停止后台线程，释放摄像头"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._cap and self._cap.isOpened():
            self._cap.release()
        print("摄像头已释放")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()

    # ---------- 属性 ----------
    @property
    def frame(self):
        """线程安全地获取当前帧（副本）"""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    @property
    def fps(self):
        with self._lock:
            return self._fps


# ============================================================================
# 模块5：CircleDetector — 色环定位
# ============================================================================
class CircleDetector:
    """
    霍夫圆检测 —— 定位图像中的三个色环

    处理流程：
      腐蚀 → 膨胀 → 灰度化 → CLAHE增强 → 形态学梯度 → 高斯模糊 →
      对比度增强 → 高斯模糊 → 二值化 → 高斯模糊 → 霍夫圆检测

    用法:
        detector = CircleDetector()
        circles = detector.find(img)
        if circles:  # 恰好 3 个
            x1,y1, x2,y2, x3,y3 = circles  # 按 X 升序
    """

    def __init__(self):
        self._debug_img = None  # 调试用二值图

    def find(self, img):
        """
        :param img: BGR 图像
        :return: (x1,y1, x2,y2, x3,y3) 按 X 升序, 或 None
        """
        if img is None:
            return None

        eroded  = cv2.erode(img, None, iterations=2)
        dilated = cv2.dilate(eroded, np.ones((7, 7), np.uint8), 1)
        gray    = cv2.cvtColor(dilated, cv2.COLOR_BGR2GRAY)

        # 形态学梯度 —— 增强边缘
        kernel   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        gradient = cv2.morphologyEx(gray, cv2.MORPH_GRADIENT, kernel)

        result   = cv2.GaussianBlur(gradient, (7, 7), 3)
        enhanced = cv2.convertScaleAbs(result, alpha=4, beta=0)
        enhanced = cv2.GaussianBlur(enhanced, (7, 7), 3)

        _, binary = cv2.threshold(enhanced, 70, 255, cv2.THRESH_BINARY)
        binary = cv2.GaussianBlur(binary, (9, 9), 3)
        self._debug_img = binary

        circles = cv2.HoughCircles(
            binary, cv2.HOUGH_GRADIENT_ALT, 1.5, 50,
            param1=100, param2=0.95, minRadius=15, maxRadius=50
        )

        if circles is None or len(circles[0]) != 3:
            return None

        circles = np.uint16(np.around(circles))
        pts = [(int(c[0]), int(c[1])) for c in circles[0, :]]
        pts.sort(key=lambda p: p[0])  # 按 X 升序

        return (pts[0][0], pts[0][1],
                pts[1][0], pts[1][1],
                pts[2][0], pts[2][1])

    @property
    def debug_binary(self):
        return self._debug_img


# ============================================================================
# 模块6：QRScanner — 二维码识别
# ============================================================================

def scan_qr(camera_index=0, width=1920, height=1080):
    """
    打开摄像头，循环检测二维码。
    识别到 → 打印并返回内容。
    按 q 退出 → 返回 None。
    """
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    assert cap.isOpened(), "摄像头打开失败"

    qr = cv2.QRCodeDetector()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        small = cv2.resize(frame, None, fx=0.5, fy=0.5)
        data, _, _ = qr.detectAndDecode(small)

        if data:
            print(f"[QR] {data}")
            cap.release()
            cv2.destroyAllWindows()
            return data

        cv2.imshow("QR Scanner — q 退出", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None

# ============================================================================
# 主程序 —— 双摄像头 + 任务状态机
# ============================================================================
# 摄像头分配：
#   摄像头0 (cam0) → 识别用：后台线程持续抓帧，供颜色/色环检测
#   摄像头2 (cap2) → 扫码用：TASK_SCAN 时按需打开，完成后释放
#
# 任务码（与 STM32 jetsondata.h 保持一致）：
#   TASK_SCAN     = 0x01  扫码（→ 摄像头2）
#   TASK_TRAY     = 0x02  物料盘定位（三色块平均中心）
#   TASK_MATERIAL = 0x03  单物料识别（param 指定颜色）
#   TASK_RING     = 0x04  色环定位（三路卡尔曼各管一个）
#   TASK_CALIB    = 0x05  车身校准（色环XY + 左右Y差）
#
# 卡尔曼滤波器分配：
#   kf1 → 单目标 / 左色环
#   kf2 → 中间色环
#   kf3 → 右色环
# ============================================================================

if __name__ == "__main__":
    # ===================================================================
    # 1. 串口通信初始化
    # ===================================================================
    stm32 = STM32TaskReceiver(port="/dev/ttyCH341USB0")
    sender = STM32ResponseSender(port="/dev/ttyCH341USB0")
    stm32.open()

    # ===================================================================
    # 2. 摄像头0：识别用（后台线程持续抓帧）
    # ===================================================================
    cam0 = ColorDetector(0)
    print("[Vision] 摄像头0 就绪 → 识别专用")

    # ===================================================================
    # 3. 卡尔曼滤波器（三路独立，多目标各用各的）
    # ===================================================================
    kf1 = KalmanFilter2D()
    kf2 = KalmanFilter2D()
    kf3 = KalmanFilter2D()

    # ===================================================================
    # 4. 检测器
    # ===================================================================
    circle_det = CircleDetector()
    qr_decoder = cv2.QRCodeDetector()

    # ===================================================================
    # 5. 状态变量
    # ===================================================================
    current_code = 0
    param = 0
    COLOR_NAME_MAP = {COLOR_RED: "Red", COLOR_GREEN: "Green", COLOR_BLUE: "Blue"}

    print("=" * 55)
    print("[Vision] 双摄像头模式启动")
    print("  摄像头0 → 识别（后台线程，持续抓帧）")
    print("  摄像头2 → 扫码（TASK_SCAN 时按需打开）")
    print("[Vision] 等待 STM32 任务帧...")
    print("=" * 55)

    while True:
        # ----- 读取 STM32 任务码 -----
        task = stm32.get_task()
        if task:
            current_code, param = task
            print(f"\n[Vision] ▶ {STM32TaskReceiver.task_name(current_code)}  "
                  f"param=0x{param:02X}")

        # ================================================================
        # TASK_SCAN (0x01)：二维码识别 → 摄像头2（按需打开）
        # 子循环内独立处理，不干扰摄像头0的抓帧线程
        # ================================================================
        if current_code == TASK_SCAN:
            print("[QR] 打开摄像头2...")
            cap2 = cv2.VideoCapture(2, cv2.CAP_V4L2)
            cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

            if not cap2.isOpened():
                print("[QR] ✗ 摄像头2 打开失败！")
                current_code = 0
                continue

            qr_found = False
            # 扫码子循环（最多约10秒 / 300帧，避免无限卡死）
            for _ in range(300):
                ret, frame2 = cap2.read()
                if not ret:
                    continue

                small = cv2.resize(frame2, None, fx=0.5, fy=0.5)
                data, _, _ = qr_decoder.detectAndDecode(small)

                if data:
                    print(f"[QR] ✓ 解码成功: '{data}'")
                    ch = [ord(c) if isinstance(c, str) else int(c)
                          for c in data[:6]]
                    while len(ch) < 6:
                        ch.append(0)
                    sender.send_qrcode(*ch)
                    qr_found = True
                    break

                cv2.imshow("Camera2 - QR Scan", frame2)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if not qr_found:
                print("[QR] 超时 / 未检测到二维码")

            cap2.release()
            cv2.destroyWindow("Camera2 - QR Scan")
            current_code = 0
            continue  # 回到主循环，继续识别任务

        # ================================================================
        # 以下任务全部使用摄像头0（后台线程已持续抓帧）
        # ================================================================

        # ----- 获取颜色识别结果 -----
        pos = cam0.get_color_positions()

        # ================================================================
        # TASK_TRAY (0x02)：物料盘定位
        # 识别 RGB 三色块 → 取平均中心 → 单路卡尔曼 → 发送
        # ================================================================
        if current_code == TASK_TRAY:
            r = pos.get("Red")
            g = pos.get("Green")
            b = pos.get("Blue")
            if r and g and b:
                cx = (r[0] + g[0] + b[0]) // 3
                cy = (r[1] + g[1] + b[1]) // 3
                sx, sy = kf1.update(cx, cy)
                sender.send_tray(sx, sy)
                print(f"[Tray] 三色平均中心 → ({sx}, {sy})")
                current_code = 0

        # ================================================================
        # TASK_MATERIAL (0x03)：单物料识别
        # param 指定颜色 → 识别该色块 → 单路卡尔曼 → 发送
        # ================================================================
        elif current_code == TASK_MATERIAL:
            color_key = COLOR_NAME_MAP.get(param)
            if color_key:
                pt = pos.get(color_key)
                if pt:
                    sx, sy = kf1.update(pt[0], pt[1])
                    # 物料坐标帧：[0x66, 0x03, color_code, X_H, X_L, Y_H, Y_L, 0x99]
                    sender.uart.write(bytearray([
                        0x66, 0x03, param,
                        (sx >> 8) & 0xFF, sx & 0xFF,
                        (sy >> 8) & 0xFF, sy & 0xFF,
                        0x99
                    ]))
                    print(f"[Material] {color_key} → ({sx}, {sy})")
                    current_code = 0

        # ================================================================
        # TASK_RING (0x04)：色环定位
        # 霍夫圆检测 → 三路卡尔曼各管一个色环 → 发送中间色环坐标
        # ================================================================
        elif current_code == TASK_RING:
            frame = cam0.frame
            if frame is not None:
                circles = circle_det.find(frame)
                if circles:
                    x1, y1, x2, y2, x3, y3 = circles  # 已按 X 升序
                    sx1, sy1 = kf1.update(x1, y1)      # 左色环
                    sx2, sy2 = kf2.update(x2, y2)      # 中间色环
                    sx3, sy3 = kf3.update(x3, y3)      # 右色环
                    sender.send_ring(sx2, sy2)
                    print(f"[Ring] 中间色环 → ({sx2}, {sy2})")
                    current_code = 0

        # ================================================================
        # TASK_CALIB (0x05)：车身校准
        # 色环检测 → 中间色环XY + 左右Y差值（Y差≈0 = 正对色环）
        # ================================================================
        elif current_code == TASK_CALIB:
            frame = cam0.frame
            if frame is not None:
                circles = circle_det.find(frame)
                if circles:
                    x1, y1, x2, y2, x3, y3 = circles
                    sx1, sy1 = kf1.update(x1, y1)
                    sx2, sy2 = kf2.update(x2, y2)
                    sx3, sy3 = kf3.update(x3, y3)
                    z = sy1 - sy3  # 左Y - 右Y：正值=车身偏右
                    sender.send_ring_calib(sx2, sy2, z)
                    print(f"[Calib] 中间:({sx2},{sy2})  Y差:{z}")
                    current_code = 0

        # ----- 显示摄像头0画面 + FPS -----
        frame = cam0.frame
        if frame is not None:
            cv2.imshow("Camera0 - Recognition", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # ===================================================================
    # 清理
    # ===================================================================
    cam0.release()
    stm32.close()
    sender.close()
    cv2.destroyAllWindows()
    print("[Vision] 已退出")