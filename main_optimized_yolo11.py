"""
钢球检测 - YOLO11 优化版
基于最新的 YOLO11 模型
优化内容：
1. 只输出1个框
2. 计算中心点
3. 自适应平滑（根据速度调整）
4. 坐标标定接口
5. 串口上报
"""

from maix import app, camera, display, image, nn, time, uart
import struct, os

# ========== 配置参数 ==========

# 模型路径（固定为部署后的位置）
MODEL_PATH = "/root/maixcam1/best.mud"

# 检测参数
CONF_THRESHOLD = 0.25   # 置信度阈值（降低，更容易检测快速移动）
IOU_THRESHOLD = 0.7     # NMS 阈值

# 自适应坐标平滑（减少跳动 + 快速响应）
USE_SMOOTHING = True
SMOOTHING_ALPHA_SLOW = 0.25  # 慢速时更平滑
SMOOTHING_ALPHA_FAST = 0.85  # 快速运动时响应更快
VELOCITY_THRESHOLD = 5       # 速度阈值降低

# 运动预测（应对快速运动时的短暂丢失）
USE_PREDICTION = True        # 启用预测！⭐
MAX_LOST_FRAMES = 3          # 最多预测3帧（降低，避免飘太远）

# 坐标标定（像素 → 厘米）
CALIBRATION_K = 0.0840  # 标定系数 (cm/px)
CALIBRATION_B = -13.36  # 标定偏移 (cm)

# 显示选项
SHOW_CENTER_CROSS = True
SHOW_BBOX = False
SHOW_COORD = True
SHOW_FPS = True

# 串口上报
REPORT_ENABLED = True
UART_PORT = "/dev/ttyS0"  # MaixCAM的串口设备
UART_BAUDRATE = 115200

# 数据包协议（增加校验）
PACKET_HEADER = 0xAA  # 包头
PACKET_TAIL = 0x55    # 包尾

# ========== 辅助函数 ==========

# 速度计算全局变量
last_error = 0.0
last_time = 0.0

def encode_ball_data(error_cm):
    """编码钢球位置+速度（发送给STM32）
    数据包格式：[0xAA][float position 4字节][float velocity 4字节][校验和 1字节]
    总长度：10字节
    position正值：球在右侧，负值：球在左侧
    velocity正值：向右运动，负值：向左运动
    """
    global last_error, last_time

    # 计算速度（cm/s）
    current_time = time.time()  # 秒
    if last_time > 0:
        dt = current_time - last_time
        if dt > 0:
            velocity = (error_cm - last_error) / dt
        else:
            velocity = 0.0
    else:
        velocity = 0.0

    # 更新历史
    last_error = error_cm
    last_time = current_time

    # 打包数据（小端格式）
    data = struct.pack("<Bff", PACKET_HEADER, error_cm, velocity)
    # B = 包头(1字节)
    # f = position(4字节)
    # f = velocity(4字节)
    # 共9字节

    # 计算校验和（所有字节的和 & 0xFF）
    checksum = sum(data) & 0xFF

    # 组装数据包：数据(9字节) + 校验和(1字节) = 10字节
    packet = data + struct.pack("<B", checksum)

    return packet

def pixel_to_cm(pixel_x):
    """像素 → 厘米"""
    if CALIBRATION_K is None or CALIBRATION_B is None:
        return None
    return CALIBRATION_K * pixel_x + CALIBRATION_B

# ========== 主程序 ==========

# 加载 YOLO11 模型
detector = nn.YOLO11(model=MODEL_PATH, dual_buff=True)

print("=" * 50)
print("钢球检测系统 - YOLO11 (自适应平滑)")
print(f"输入分辨率: {detector.input_width()}x{detector.input_height()}")
print(f"类别: {detector.labels}")
print(f"检测阈值: conf={CONF_THRESHOLD}, iou={IOU_THRESHOLD}")
print(f"平滑系数: 慢速={SMOOTHING_ALPHA_SLOW}, 快速={SMOOTHING_ALPHA_FAST}")
if CALIBRATION_K is not None:
    print(f"坐标标定: k={CALIBRATION_K:.4f}, b={CALIBRATION_B:.4f}")
else:
    print("坐标标定: 未标定")
print("=" * 50)

# 初始化摄像头
cam = camera.Camera(
    detector.input_width(),
    detector.input_height(),
    detector.input_format(),
)

disp = display.Display()

# 初始化原始串口（用于发送到STM32）
if REPORT_ENABLED:
    uart_stm32 = uart.UART(UART_PORT, UART_BAUDRATE)
    print(f"UART初始化: {UART_PORT} @ {UART_BAUDRATE}")

# 状态变量
smoothed_x = None
smoothed_y = None
lost_frame_count = 0
last_velocity_x = 0
last_velocity_y = 0
predicted_x = None
predicted_y = None

# 帧率统计
fps_counter = 0
fps_start_time = time.time_ms()
current_fps = 0

while not app.need_exit():
    img = cam.read()
    if img is None:
        continue

    # YOLO11 检测
    objs = detector.detect(img, conf_th=CONF_THRESHOLD, iou_th=IOU_THRESHOLD)

    # 只取置信度最高的1个
    if len(objs) > 0:
        best_obj = max(objs, key=lambda x: x.score)

        # 计算中心点
        raw_center_x = int(best_obj.x + best_obj.w / 2)
        raw_center_y = int(best_obj.y + best_obj.h / 2)

        # 重置丢失计数
        lost_frame_count = 0

        # 计算速度并选择平滑系数
        if smoothed_x is not None:
            last_velocity_x = raw_center_x - smoothed_x
            last_velocity_y = raw_center_y - smoothed_y

            # 计算速度大小（曼哈顿距离）
            velocity_magnitude = abs(last_velocity_x) + abs(last_velocity_y)

            # 根据速度动态选择平滑系数
            if velocity_magnitude > VELOCITY_THRESHOLD:
                # 快速运动：提高响应速度
                current_alpha = SMOOTHING_ALPHA_FAST
            else:
                # 慢速运动：增加平滑度
                current_alpha = SMOOTHING_ALPHA_SLOW
        else:
            # 首次检测
            current_alpha = SMOOTHING_ALPHA_SLOW
            last_velocity_x = 0
            last_velocity_y = 0

        # 坐标平滑
        if USE_SMOOTHING:
            if smoothed_x is None:
                smoothed_x = float(raw_center_x)
                smoothed_y = float(raw_center_y)
            else:
                # 使用自适应平滑系数
                smoothed_x = current_alpha * raw_center_x + (1 - current_alpha) * smoothed_x
                smoothed_y = current_alpha * raw_center_y + (1 - current_alpha) * smoothed_y

            center_x = int(smoothed_x)
            center_y = int(smoothed_y)
        else:
            center_x = raw_center_x
            center_y = raw_center_y

        predicted_x = center_x
        predicted_y = center_y

        # 串口上报：发送位置+速度到STM32
        if REPORT_ENABLED:
            x_cm = pixel_to_cm(center_x)
            if x_cm is not None:
                error = 0.0 - x_cm  # 目标中心为0，计算偏差
                packet = encode_ball_data(error)  # 发送10字节数据包（位置+速度）
                uart_stm32.write(packet)

        # 终端打印
        if fps_counter % 10 == 0:
            cm_x = pixel_to_cm(center_x)
            if cm_x is not None:
                error = 0.0 - cm_x
                print(f"钢球: X={cm_x:.2f}cm, 偏差={error:.2f}cm, 置信度={best_obj.score:.2f}")
            else:
                print(f"钢球: X={center_x}px, Y={center_y}px, 置信度={best_obj.score:.2f}")

    elif USE_PREDICTION and lost_frame_count < MAX_LOST_FRAMES and predicted_x is not None:
        # 预测模式
        lost_frame_count += 1
        predicted_x = int(predicted_x + last_velocity_x)
        predicted_y = int(predicted_y + last_velocity_y)

        # 限制预测范围在画面内
        predicted_x = max(0, min(predicted_x, detector.input_width() - 1))
        predicted_y = max(0, min(predicted_y, detector.input_height() - 1))

        center_x = predicted_x
        center_y = predicted_y

        if fps_counter % 5 == 0:
            print(f"预测: X={center_x}px (丢失{lost_frame_count}帧)")
    else:
        # 真正丢失
        if smoothed_x is not None and fps_counter % 20 == 0:
            print("警告: 钢球丢失")
        center_x = None
        center_y = None

    # 绘制检测结果
    if center_x is not None:
        if SHOW_BBOX and len(objs) > 0:
            img.draw_rect(int(best_obj.x), int(best_obj.y),
                         int(best_obj.w), int(best_obj.h),
                         color=image.COLOR_RED, thickness=2)

        if SHOW_CENTER_CROSS:
            cross_size = 15
            img.draw_line(center_x - cross_size, center_y,
                         center_x + cross_size, center_y,
                         color=image.COLOR_RED, thickness=2)
            img.draw_line(center_x, center_y - cross_size,
                         center_x, center_y + cross_size,
                         color=image.COLOR_RED, thickness=2)
            img.draw_circle(center_x, center_y, 3,
                           color=image.COLOR_GREEN, thickness=-1)

        if SHOW_COORD:
            cm_x = pixel_to_cm(center_x)
            if cm_x is not None:
                text = f"X:{cm_x:.1f}cm"
            else:
                text = f"X:{center_x}px"
            img.draw_string(center_x + 20, center_y - 10, text,
                           color=image.COLOR_YELLOW, scale=1.5)

    # 显示帧率
    if SHOW_FPS:
        fps_counter += 1
        elapsed = time.time_ms() - fps_start_time
        if elapsed >= 1000:
            current_fps = fps_counter * 1000.0 / elapsed
            fps_counter = 0
            fps_start_time = time.time_ms()

        img.draw_string(5, 5, f"FPS:{current_fps:.1f}",
                       color=image.COLOR_GREEN, scale=2)

    disp.show(img)

print("程序退出")
