import cv2
import time
import onnxruntime as ort
import numpy as np
from PIL import Image
from collections import deque, Counter
import configparser
import queue
import re
import os
import glob
import sys
import threading

try:
    import serial
except Exception:
    serial = None

# ===== 配置 =====
model_path = "/home/orangepi/project/model.onnx"
class_names = ['hazardous', 'kitchen', 'other', 'recyclable']

# ===== 识别区域配置（可直接修改） =====
# 使用整幅画面的比例来定义“居中识别区域”的宽和高
# 默认 0.50 / 0.50 与旧版逻辑一致：取画面中间 50% x 50% 的区域
ROI_WIDTH_RATIO = 0.50
ROI_HEIGHT_RATIO = 0.50

# ===== 摄像头配置 =====
# 自动扫描 /dev/video*，优先选择 USB 摄像头。
# 注意：这里优先使用 /dev/videoX 设备路径打开，而不是用数字 index 打开，
# 因为部分香橙派 / OpenCV / V4L2 环境下按 index 打开会失败。
AUTO_DETECT_USB_CAMERA = True
FALLBACK_CAMERA_PATH = "/dev/video1"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_TEST_READ_COUNT = 3

# ===== 预留：四类垃圾对应的垃圾桶编号 =====
BIN_HAZARDOUS = 2
BIN_KITCHEN = 1
BIN_OTHER = 4
BIN_RECYCLABLE = 3

class_to_bin = {
    "hazardous": BIN_HAZARDOUS,
    "kitchen": BIN_KITCHEN,
    "other": BIN_OTHER,
    "recyclable": BIN_RECYCLABLE,
}

# ===== 点位布局 / 颜色定义 =====
# 约定：
# 1、2号桶在待机点左侧；3、4号桶在待机点右侧
# HOME 只做基准定义，不作为香橙派下发的投放目标
# color_ascii / direction_ascii 会直接下发给 STM32
# ratio_rgb / ratio_tolerance 是“颜色传感器格式”：
# 1. 三通道之和建议为 255
# 2. 这组值会原样作为 RGB/TOL 下发给 STM32
# 3. STM32 内部会把当前颜色和目标颜色都转成比例后再匹配
POINT_LAYOUT = {
    "HOME": {
        "point_name": "待机点",
        "direction_ascii": "CENTER",
        "color_ascii": "PURPLE",
        "ratio_rgb": (93, 86, 76),
        "ratio_tolerance": (11, 11, 11),
    },
    "BIN_1": {
        "point_name": "1号桶",
        "direction_ascii": "LEFT",
        "color_ascii": "RED",
        "ratio_rgb": (155, 50, 50),
        "ratio_tolerance": (28, 18, 18),
    },
    "BIN_2": {
        "point_name": "2号桶",
        "direction_ascii": "LEFT",
        "color_ascii": "GREEN",
        "ratio_rgb": (73, 125, 57),
        "ratio_tolerance": (19, 16, 22),
    },
    "BIN_3": {
        "point_name": "3号桶",
        "direction_ascii": "RIGHT",
        "color_ascii": "BLUE",
        "ratio_rgb": (51,106,98),
        "ratio_tolerance": (12,12,11),
    },
    "BIN_4": {
        "point_name": "4号桶",
        "direction_ascii": "RIGHT",
        "color_ascii": "YELLOW",
        "ratio_rgb": (102, 97, 56),
        "ratio_tolerance": (11, 11, 11),
    },
}

BIN_METADATA = {
    1: POINT_LAYOUT["BIN_1"],
    2: POINT_LAYOUT["BIN_2"],
    3: POINT_LAYOUT["BIN_3"],
    4: POINT_LAYOUT["BIN_4"],
}

# ===== 串口通讯配置 =====
ENABLE_SERIAL = True
SERIAL_PORT = "/dev/ttyS5"
SERIAL_BAUD = 9600
SERIAL_TIMEOUT = 0
SERIAL_CMD_EOL = "\n"

# ===== STM32 运行时配置文件 =====
# 这个 INI 文件可以写中文注释；运行中修改保存后，脚本会自动重新下发给 STM32。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
STM32_RUNTIME_CONFIG_PATH = os.path.join(SCRIPT_DIR, "stm32_runtime_config.ini")
STM32_CONFIG_RELOAD_INTERVAL_SEC = 1.0
STM32_CONFIG_SECTION = "stm32_runtime_config"
STM32_COMMAND_SPACING_SEC = 0.20
STM32_COMMAND_ACK_TIMEOUT_SEC = 1.50

STM32_RUNTIME_CONFIG_DEFAULTS = {
    "MT": 20000,  # 去目标桶超时 ms
    "PD": 1000,   # 到桶后开门前等待 ms
    "GT": 3000,   # 门动作超时 ms
    "DW": 1500,   # 开门后等待投放 ms
    "PC": 1000,   # 关门后等待 ms
    "RH": 20000,  # 回 home 超时 ms
    "EB": 200,    # 错误状态 LED 闪烁间隔 ms
    "SP": 80,     # 平台移动速度，STM32 端限制 0~90
    "OA": 20,     # 开门角度，MG90S 保守起始值
    "CA": 80,     # 关门角度，MG90S 保守起始值
    "OS": 600,    # 开门动作等待 ms
    "CS": 600,    # 关门动作等待 ms
}

STM32_RUNTIME_CONFIG_LIMITS = {
    "MT": (0, 120000),
    "PD": (0, 120000),
    "GT": (0, 120000),
    "DW": (0, 120000),
    "PC": (0, 120000),
    "RH": (0, 120000),
    "EB": (0, 120000),
    "SP": (0, 90),
    "OA": (0, 180),
    "CA": (0, 180),
    "OS": (0, 10000),
    "CS": (0, 10000),
}

# ===== 与 STM32 的协议指令（纯 ASCII） =====
CMD_OPI_TARGET_PREFIX = "[$TARGET_BIN:"
CMD_OPI_TARGET_SUFFIX = "]"
CMD_OPI_HOME_PREFIX = "[$HOME:RGB:"
CMD_OPI_HOME_SUFFIX = "]"
CMD_OPI_PING = "[$PING]"
CMD_OPI_RESET = "[$RESET]"
CMD_OPI_HARD_RESET = "[$HARD_RESET]"
CMD_STM32_ACK = "[$ACK]"
CMD_STM32_BUSY = "[$BUSY]"
CMD_STM32_ERR = "[$ERR]"
CMD_STM32_DONE_HOME = "[$DONE_HOME]"
CMD_STM32_DONE_HOME_ASCII = "[$DONE_HOME]"

# ===== ONNX Runtime =====
session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
input_name = session.get_inputs()[0].name
print("[INIT] ONNX 模型加载成功", flush=True)


# ===== 预处理 =====
def preprocess(pil_img):
    pil_img = pil_img.resize((224, 224))
    img = np.array(pil_img).astype(np.float32) / 255.0
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    img = (img - mean) / std
    img = np.transpose(img, (2, 0, 1))   # HWC -> CHW
    img = np.expand_dims(img, axis=0)     # -> NCHW
    return img.astype(np.float32)


def softmax(x):
    e = np.exp(x - np.max(x, axis=1, keepdims=True))
    return e / np.sum(e, axis=1, keepdims=True)


def color_distance(rgb_a, rgb_b):
    return (
        abs(rgb_a[0] - rgb_b[0]) +
        abs(rgb_a[1] - rgb_b[1]) +
        abs(rgb_a[2] - rgb_b[2])
    )


def make_ratio_rgb(rgb):
    total = int(rgb[0]) + int(rgb[1]) + int(rgb[2])
    if total <= 0:
        return (0, 0, 0)

    r = int(round((int(rgb[0]) * 255.0) / total))
    g = int(round((int(rgb[1]) * 255.0) / total))
    b = 255 - r - g
    return (
        max(0, min(255, r)),
        max(0, min(255, g)),
        max(0, min(255, b)),
    )


def is_rgb_within_tolerance(rgb, target_rgb, tolerance_rgb):
    return (
        abs(rgb[0] - target_rgb[0]) <= tolerance_rgb[0] and
        abs(rgb[1] - target_rgb[1]) <= tolerance_rgb[1] and
        abs(rgb[2] - target_rgb[2]) <= tolerance_rgb[2]
    )


def classify_tcs_rgb888(rgb):
    """按与 STM32 一致的比例色格式，对 TCS 颜色做近似分类。"""
    best_key = None
    best_profile = None
    best_distance = None
    ratio_rgb = make_ratio_rgb(rgb)

    for key, profile in POINT_LAYOUT.items():
        target = profile["ratio_rgb"]
        tolerance = profile["ratio_tolerance"]

        if not is_rgb_within_tolerance(ratio_rgb, target, tolerance):
            continue

        dist = color_distance(ratio_rgb, target)
        if best_distance is None or dist < best_distance:
            best_key = key
            best_profile = profile
            best_distance = dist

    if best_profile is None:
        return None

    return {
        "key": best_key,
        "label": best_profile["point_name"],
        "color_name": best_profile["color_ascii"],
        "distance": best_distance,
        "target_rgb": best_profile["ratio_rgb"],
        "tolerance": best_profile["ratio_tolerance"],
        "input_ratio_rgb": ratio_rgb,
    }


def get_bin_metadata(bin_id):
    """获取目标桶的方向、颜色等元数据。"""
    return BIN_METADATA.get(bin_id)


def log_event(msg):
    print(f"[EVENT] {msg}", flush=True)


def log_infer(msg):
    print(f"[INFER] {msg}", flush=True)


def apply_current_scene_as_background(gray_frame):
    """把当前空场景直接设为背景。"""
    return gray_frame.copy()


def allow_background_update(state_name, candidate_present_flag, motion_pixels_value):
    """控制是否允许缓慢更新背景。"""
    return (
        enable_auto_background_update
        and state_name == "WAIT_OBJECT"
        and (not candidate_present_flag)
        and motion_pixels_value < candidate_motion_freeze_threshold
    )


def abnormal_contour_filter_enabled():
    return enable_abnormal_contour_filter


def open_serial_port():
    """打开串口；若不可用则返回 None。"""
    if not ENABLE_SERIAL:
        print("[INIT] 串口通讯已关闭", flush=True)
        return None
    if serial is None:
        print("[WARN] 未安装 pyserial，串口通讯不可用", flush=True)
        return None

    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
        print(f"[INIT] 串口已打开: {SERIAL_PORT} @ {SERIAL_BAUD}", flush=True)
        return ser
    except Exception as e:
        print(f"[WARN] 串口打开失败: {e}", flush=True)
        return None


def build_target_command(bin_id):
    """构造发给 STM32 的目标桶指令，附带方向、RGB 和容差。"""
    metadata = get_bin_metadata(bin_id)
    if metadata is None:
        return f"{CMD_OPI_TARGET_PREFIX}{bin_id}{CMD_OPI_TARGET_SUFFIX}"

    direction_ascii = metadata["direction_ascii"]
    r, g, b = metadata["ratio_rgb"]
    tol_r, tol_g, tol_b = metadata["ratio_tolerance"]
    return (
        f"{CMD_OPI_TARGET_PREFIX}{bin_id},"
        f"DIR:{direction_ascii},"
        f"RGB:{r},{g},{b},"
        f"TOL:{tol_r},{tol_g},{tol_b}"
        f"{CMD_OPI_TARGET_SUFFIX}"
    )


def build_home_command():
    """构造发给 STM32 的 HOME 颜色配置指令。"""
    home_meta = POINT_LAYOUT["HOME"]
    r, g, b = home_meta["ratio_rgb"]
    tol_r, tol_g, tol_b = home_meta["ratio_tolerance"]
    return (
        f"{CMD_OPI_HOME_PREFIX}{r},{g},{b},"
        f"TOL:{tol_r},{tol_g},{tol_b}"
        f"{CMD_OPI_HOME_SUFFIX}"
    )


def load_stm32_runtime_config():
    """读取可带注释的 STM32 运行时配置 INI。"""
    cfg = dict(STM32_RUNTIME_CONFIG_DEFAULTS)

    if not os.path.exists(STM32_RUNTIME_CONFIG_PATH):
        print(f"[WARN] 未找到 STM32 配置文件，使用默认值: {STM32_RUNTIME_CONFIG_PATH}", flush=True)
        return cfg

    parser = configparser.ConfigParser(inline_comment_prefixes=("#", ";"))
    parser.optionxform = str
    try:
        parser.read(STM32_RUNTIME_CONFIG_PATH, encoding="utf-8")
    except Exception as e:
        print(f"[WARN] STM32 配置文件读取失败，使用默认值: {e}", flush=True)
        return cfg

    if not parser.has_section(STM32_CONFIG_SECTION):
        print(f"[WARN] STM32 配置文件缺少 [{STM32_CONFIG_SECTION}]，使用默认值", flush=True)
        return cfg

    for key, default_value in STM32_RUNTIME_CONFIG_DEFAULTS.items():
        raw_value = parser.get(STM32_CONFIG_SECTION, key, fallback=str(default_value)).strip()
        try:
            value = int(raw_value)
        except ValueError:
            print(f"[WARN] STM32 配置 {key}={raw_value!r} 不是整数，使用默认值 {default_value}", flush=True)
            value = default_value

        min_value, max_value = STM32_RUNTIME_CONFIG_LIMITS[key]
        if value < min_value or value > max_value:
            print(
                f"[WARN] STM32 配置 {key}={value} 超出范围 {min_value}~{max_value}，使用默认值 {default_value}",
                flush=True,
            )
            value = default_value

        cfg[key] = value

    return cfg


def build_stm32_config_command(cfg):
    """构造 STM32 运行时配置命令。"""
    ordered_keys = ["MT", "PD", "GT", "DW", "PC", "RH", "EB", "SP", "OA", "CA", "OS", "CS"]
    body = ",".join(f"{key}={int(cfg[key])}" for key in ordered_keys)
    return f"[$CFG:{body}]"


def send_serial_message(ser, message):
    """发送一条串口消息。"""
    if ser is None:
        print(f"[WARN] 串口未就绪，未发送: {message}", flush=True)
        return False
    try:
        payload = (message + SERIAL_CMD_EOL).encode("utf-8")
        ser.write(payload)
        ser.flush()
        print(f"[TX] {message}", flush=True)
        return True
    except Exception as e:
        print(f"[WARN] 串口发送失败: {e}", flush=True)
        return False


stm32_config_last_mtime = None
stm32_config_last_check_time = 0.0
stm32_config_current = dict(STM32_RUNTIME_CONFIG_DEFAULTS)
stm32_command_queue = deque()
stm32_waiting_ack = None
stm32_waiting_ack_since = 0.0
stm32_last_command_sent_at = 0.0


def queue_stm32_command(message, expect_ack=True):
    stm32_command_queue.append({
        "message": message,
        "expect_ack": expect_ack,
    })


def process_stm32_command_queue(ser):
    global stm32_waiting_ack, stm32_waiting_ack_since, stm32_last_command_sent_at

    if ser is None:
        return False

    now = time.time()

    if stm32_waiting_ack is not None:
        if (now - stm32_waiting_ack_since) >= STM32_COMMAND_ACK_TIMEOUT_SEC:
            log_event(f"等待 ACK 超时，继续后续命令: {stm32_waiting_ack}")
            stm32_waiting_ack = None
        else:
            return False

    if not stm32_command_queue:
        return False

    if (now - stm32_last_command_sent_at) < STM32_COMMAND_SPACING_SEC:
        return False

    item = stm32_command_queue.popleft()
    if send_serial_message(ser, item["message"]):
        stm32_last_command_sent_at = now
        if item["expect_ack"]:
            stm32_waiting_ack = item["message"]
            stm32_waiting_ack_since = now
        return True

    return False


def send_stm32_runtime_config_if_needed(ser, force=False):
    """配置文件变化时自动下发 STM32 运行时配置。"""
    global stm32_config_last_mtime, stm32_config_last_check_time, stm32_config_current

    now = time.time()
    if (not force) and ((now - stm32_config_last_check_time) < STM32_CONFIG_RELOAD_INTERVAL_SEC):
        return False

    stm32_config_last_check_time = now

    try:
        mtime = os.path.getmtime(STM32_RUNTIME_CONFIG_PATH)
    except OSError:
        mtime = None

    if (not force) and (mtime == stm32_config_last_mtime):
        return False

    stm32_config_current = load_stm32_runtime_config()
    stm32_config_last_mtime = mtime
    queue_stm32_command(CMD_OPI_PING)
    queue_stm32_command(build_home_command())
    queue_stm32_command(build_stm32_config_command(stm32_config_current))
    log_event("已排队 STM32 启动握手与 HOME/运行时配置")
    return True


console_command_queue = queue.Queue()


def console_input_worker():
    """后台读取用户输入，避免阻塞摄像头主循环。"""
    while True:
        try:
            line = sys.stdin.readline()
        except Exception as e:
            print(f"[WARN] 用户输入读取失败: {e}", flush=True)
            return

        if line == "":
            return

        command = line.strip()
        if command:
            console_command_queue.put(command)


def start_console_input_worker():
    worker = threading.Thread(target=console_input_worker, daemon=True)
    worker.start()
    print(
        "[INIT] 运行时命令: help, pause, resume, reset, hard_reset, ping, get_cfg, cfg, "
        "color_debug, stop_color, target <1-4>, send <raw>, quit",
        flush=True,
    )


def poll_serial_messages(ser, rx_buffer):
    """非阻塞读取串口消息，返回 (messages, new_buffer)。"""
    if ser is None:
        return [], rx_buffer

    messages = []
    try:
        waiting = ser.in_waiting
        if waiting <= 0:
            return messages, rx_buffer

        raw = ser.read(waiting)
        if not raw:
            return messages, rx_buffer

        rx_buffer += raw.decode("utf-8", errors="ignore")
        while "\n" in rx_buffer:
            line, rx_buffer = rx_buffer.split("\n", 1)
            line = line.strip()
            if line:
                messages.append(line)
    except Exception as e:
        print(f"[WARN] 串口读取失败: {e}", flush=True)

    return messages, rx_buffer


def is_done_home_message(msg):
    return msg == CMD_STM32_DONE_HOME


def log_stm32_debug(msg):
    print(f"[STM32_DEBUG] {msg}", flush=True)


color_debug_session_active = False
color_debug_session_started_at = 0.0
color_debug_samples = []


def start_color_debug_session():
    global color_debug_session_active, color_debug_session_started_at, color_debug_samples
    color_debug_session_active = True
    color_debug_session_started_at = time.time()
    color_debug_samples = []
    log_event("已开始采集 STM32 颜色调试样本")


def stop_color_debug_session():
    global color_debug_session_active
    was_active = color_debug_session_active
    color_debug_session_active = False
    if was_active:
        summarize_color_debug_session()


def record_color_debug_sample(sample):
    if color_debug_session_active:
        color_debug_samples.append(sample)


def _calc_avg(values):
    if not values:
        return 0
    return int(round(sum(values) / len(values)))


def summarize_color_debug_session():
    sample_count = len(color_debug_samples)
    duration_sec = max(0.0, time.time() - color_debug_session_started_at)

    if sample_count == 0:
        print("[COLOR_DEBUG] 本次没有采集到任何颜色样本", flush=True)
        return

    c_values = [s["c"] for s in color_debug_samples]
    r_values = [s["r"] for s in color_debug_samples]
    g_values = [s["g"] for s in color_debug_samples]
    b_values = [s["b"] for s in color_debug_samples]
    rr_values = [s["calc_ratio_r"] for s in color_debug_samples]
    rg_values = [s["calc_ratio_g"] for s in color_debug_samples]
    rb_values = [s["calc_ratio_b"] for s in color_debug_samples]
    stm32_rr_values = [s["ratio_r"] for s in color_debug_samples]
    stm32_rg_values = [s["ratio_g"] for s in color_debug_samples]
    stm32_rb_values = [s["ratio_b"] for s in color_debug_samples]
    home_hits = sum(1 for s in color_debug_samples if s["home"] != 0)
    bin_hits = sum(1 for s in color_debug_samples if s["bin"] != 0)

    avg_rgb = (_calc_avg(r_values), _calc_avg(g_values), _calc_avg(b_values))
    avg_ratio = (_calc_avg(rr_values), _calc_avg(rg_values), _calc_avg(rb_values))
    max_dev_ratio = (
        max(abs(v - avg_ratio[0]) for v in rr_values),
        max(abs(v - avg_ratio[1]) for v in rg_values),
        max(abs(v - avg_ratio[2]) for v in rb_values),
    )
    recommended_tol = tuple(max(8, dev + 10) for dev in max_dev_ratio)

    matched = classify_tcs_rgb888(avg_rgb)

    print(
        f"[COLOR_DEBUG] 样本总结: count={sample_count} duration={duration_sec:.2f}s "
        f"C(avg/min/max)={_calc_avg(c_values)}/{min(c_values)}/{max(c_values)} "
        f"RGB(avg)={avg_rgb} RGB(min)=({min(r_values)},{min(g_values)},{min(b_values)}) "
        f"RGB(max)=({max(r_values)},{max(g_values)},{max(b_values)})",
        flush=True,
    )
    print(
        f"[COLOR_DEBUG] ratio(avg/min/max)="
        f"({avg_ratio[0]},{avg_ratio[1]},{avg_ratio[2]})/"
        f"({min(rr_values)},{min(rg_values)},{min(rb_values)})/"
        f"({max(rr_values)},{max(rg_values)},{max(rb_values)}) "
        f"home_hits={home_hits}/{sample_count} bin_hits={bin_hits}/{sample_count}",
        flush=True,
    )
    print(
        f"[COLOR_DEBUG] STM32原始ratio(avg/min/max)="
        f"({ _calc_avg(stm32_rr_values)},{ _calc_avg(stm32_rg_values)},{ _calc_avg(stm32_rb_values)})/"
        f"({min(stm32_rr_values)},{min(stm32_rg_values)},{min(stm32_rb_values)})/"
        f"({max(stm32_rr_values)},{max(stm32_rg_values)},{max(stm32_rb_values)}) "
        f"(仅供对照，建议定义以本地重算ratio为准)",
        flush=True,
    )

    if matched is None:
        print("[COLOR_DEBUG] 与当前预设颜色都不够接近", flush=True)
    else:
        print(
            f"[COLOR_DEBUG] 最接近当前预设: {matched['label']}({matched['color_name']}) "
            f"target_ratio_rgb={matched['target_rgb']} ratio_tolerance={matched['tolerance']}",
            flush=True,
        )

    print(
        f"[COLOR_DEBUG] 建议定义: ratio_rgb={avg_ratio} ratio_tolerance={recommended_tol} "
        f"(推荐直接作为 STM32 的 RGB/TOL 下发值)",
        flush=True,
    )
    print(
        f"[COLOR_DEBUG] 可直接写入: ratio_rgb = {avg_ratio} | ratio_tolerance = {recommended_tol}",
        flush=True,
    )


def parse_sensors_debug_body(body):
    m = re.match(
        r"SENS C=(\d+) RGB=(\d+),(\d+),(\d+) ratio=(\d+),(\d+),(\d+) "
        r"tgt=(\d+) rgb=(\d+),(\d+),(\d+) tol=(\d+),(\d+),(\d+) bin=(\d+) home=(\d+)",
        body
    )
    if not m:
        return None

    calc_ratio = make_ratio_rgb((int(m.group(2)), int(m.group(3)), int(m.group(4))))

    sample = {
        "c": int(m.group(1)),
        "r": int(m.group(2)),
        "g": int(m.group(3)),
        "b": int(m.group(4)),
        "ratio_r": int(m.group(5)),
        "ratio_g": int(m.group(6)),
        "ratio_b": int(m.group(7)),
        "calc_ratio_r": calc_ratio[0],
        "calc_ratio_g": calc_ratio[1],
        "calc_ratio_b": calc_ratio[2],
        "target_bin": int(m.group(8)),
        "target_r": int(m.group(9)),
        "target_g": int(m.group(10)),
        "target_b": int(m.group(11)),
        "tol_r": int(m.group(12)),
        "tol_g": int(m.group(13)),
        "tol_b": int(m.group(14)),
        "bin": int(m.group(15)),
        "home": int(m.group(16)),
    }

    record_color_debug_sample(sample)
    return sample


def parse_stm32_debug_message(msg):
    """把 STM32 的 [#DBG:...] 调试指令翻译成更直观的中文日志。"""
    if not msg.startswith("[#DBG:") or not msg.endswith("]"):
        return None

    body = msg[6:-1].strip()

    debug_map = {
        "COMM init": "通讯模块初始化完成",
        "GATE open_cmd": "收到开闸命令",
        "GATE opened": "闸门已打开",
        "DROP wait_done": "投放等待完成",
        "GATE close_cmd": "收到关闸命令",
        "MOVE return_home_cmd": "开始执行回待机点",
        "HOME reached": "已回到待机点",
        "TASK done_home": "本轮任务完成，已投放并回到待机点",
        "TIMEOUT move_to_bin": "移动到目标桶超时",
    }

    if body in debug_map:
        return debug_map[body]

    m = re.match(r"RX=(.+)", body)
    if m:
        return f"收到串口数据: {m.group(1)}"

    m = re.match(r"CMD target_bin=(\d+)", body)
    if m:
        return f"解析到目标桶指令，target_bin={m.group(1)}"

    m = re.match(r"CMD home rgb=(\d+),(\d+),(\d+) tol=(\d+),(\d+),(\d+)", body)
    if m:
        return (
            f"已更新 HOME 颜色定义: ratio_rgb=({m.group(1)},{m.group(2)},{m.group(3)}) "
            f"ratio_tol=({m.group(4)},{m.group(5)},{m.group(6)})"
        )

    m = re.match(r"STATE=(.+)->(.+)", body)
    if m:
        return f"状态切换: {m.group(1)} -> {m.group(2)}"

    m = re.match(r"MOVE reached bin=(\d+)", body)
    if m:
        return f"已到达目标桶，bin={m.group(1)}"

    m = re.match(r"TIMEOUT ([a-z_]+) elapsed=(\d+) limit=(\d+)", body)
    if m:
        return f"{m.group(1)} 超时，实际经过={m.group(2)}ms，配置上限={m.group(3)}ms"

    m = re.match(r"ERROR enter from=(.+)", body)
    if m:
        return f"进入错误状态，来源状态={m.group(1)}"

    sample = parse_sensors_debug_body(body)
    if sample is not None:
        rgb888 = (sample["r"], sample["g"], sample["b"])
        c_val = sample["c"]
        r_raw = sample["r"]
        g_raw = sample["g"]
        b_raw = sample["b"]
        calc_ratio = (sample["calc_ratio_r"], sample["calc_ratio_g"], sample["calc_ratio_b"])

        matched = classify_tcs_rgb888(rgb888)
        if matched is None:
            return (
                f"TCS颜色读取: C={c_val} R={r_raw} G={g_raw} B={b_raw} "
                f"RGB888={rgb888} ratio={calc_ratio} "
                f"bin={sample['bin']} home={sample['home']} -> 判定=未知颜色"
            )

        return (
            f"TCS颜色读取: C={c_val} R={r_raw} G={g_raw} B={b_raw} "
            f"RGB888={rgb888} ratio={calc_ratio} "
            f"bin={sample['bin']} home={sample['home']} -> "
            f"判定={matched['label']}({matched['color_name']}) "
            f"目标ratio_rgb={matched['target_rgb']} ratio_tol={matched['tolerance']} 距离={matched['distance']}"
        )

    return f"未定义调试信息: {body}"


def handle_stm32_message(msg):
    """处理来自 STM32 的消息；返回本轮识别重置原因，None 表示不重置。"""
    global stm32_waiting_ack

    print(f"[RX] {msg}", flush=True)

    dbg_msg = parse_stm32_debug_message(msg)
    if dbg_msg is not None:
        log_stm32_debug(dbg_msg)
        return False

    if is_done_home_message(msg):
        log_event("收到 STM32 完成信号: DONE_HOME")
        return "收到 STM32 完成信号，恢复识别"

    if msg == CMD_STM32_ACK:
        stm32_waiting_ack = None
        log_event("STM32 已返回 ACK")
        return False

    if msg == CMD_STM32_BUSY:
        stm32_waiting_ack = None
        log_event("STM32 忙碌中，继续等待")
        return False

    if msg == CMD_STM32_ERR:
        stm32_waiting_ack = None
        log_event("STM32 返回异常，立即发送 [$RESET] 并恢复识别")
        send_serial_message(ser, CMD_OPI_RESET)
        return "收到 STM32 ERR，已发送 RESET，恢复识别"

    log_event(f"收到未定义的 STM32 指令: {msg}")
    return None


def list_video_candidates():
    """列出 /dev/video* 候选，并尽量判断是否为 USB 设备。"""
    candidates = []

    for path in sorted(glob.glob("/dev/video*")):
        name = os.path.basename(path)
        if not name.startswith("video"):
            continue

        try:
            index = int(name.replace("video", ""))
        except ValueError:
            continue

        sys_device_link = f"/sys/class/video4linux/{name}/device"
        resolved = ""
        is_usb = False

        try:
            if os.path.exists(sys_device_link):
                resolved = os.path.realpath(sys_device_link)
                is_usb = "usb" in resolved.lower()
        except Exception:
            resolved = ""

        candidates.append({
            "index": index,
            "path": path,
            "is_usb": is_usb,
            "resolved": resolved,
        })

    # 优先 USB 摄像头，再按 /dev/video 编号升序
    candidates.sort(key=lambda item: (not item["is_usb"], item["index"]))
    return candidates


def find_processes_using_device(device_path):
    """
    检查是否有进程占用某个 /dev/videoX。
    不依赖 lsof/fuser，直接遍历 /proc。
    """
    users = []
    target = os.path.realpath(device_path)

    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue

        fd_dir = f"/proc/{pid}/fd"
        comm_path = f"/proc/{pid}/comm"

        try:
            if not os.path.isdir(fd_dir):
                continue

            proc_name = "unknown"
            try:
                with open(comm_path, "r", encoding="utf-8", errors="ignore") as f:
                    proc_name = f.read().strip()
            except Exception:
                pass

            for fd in os.listdir(fd_dir):
                fd_path = os.path.join(fd_dir, fd)
                try:
                    link = os.path.realpath(fd_path)
                except Exception:
                    continue

                if link == target:
                    users.append((pid, proc_name))
                    break

        except Exception:
            continue

    return users


def try_read_camera_frames(cap):
    """试读几帧，确认这个设备真的能出图。"""
    for _ in range(CAMERA_TEST_READ_COUNT):
        ret, frame = cap.read()
        if ret and frame is not None and frame.size > 0:
            return True
        time.sleep(0.1)
    return False


def try_open_camera_candidate(item):
    """
    尝试打开一个摄像头候选。
    注意：不同 OpenCV 构建对“路径打开”和“编号打开”的支持不一样，
    所以这里 4 种方式都试一遍：
    1. 路径 + V4L2
    2. 路径 + 默认后端
    3. 编号 + V4L2
    4. 编号 + 默认后端
    """
    device_path = item["path"]
    index = item["index"]

    users = find_processes_using_device(device_path)
    if users:
        print(
            f"[WARN] {device_path} 可能正在被占用: "
            + ", ".join([f"pid={pid}({name})" for pid, name in users]),
            flush=True
        )

    open_methods = [
        (f"PATH_V4L2:{device_path}", lambda: cv2.VideoCapture(device_path, cv2.CAP_V4L2)),
        (f"PATH_DEFAULT:{device_path}", lambda: cv2.VideoCapture(device_path)),
        (f"INDEX_V4L2:{index}", lambda: cv2.VideoCapture(index, cv2.CAP_V4L2)),
        (f"INDEX_DEFAULT:{index}", lambda: cv2.VideoCapture(index)),
    ]

    for method_name, opener in open_methods:
        try:
            cap = opener()
        except Exception as e:
            print(f"[WARN] 摄像头打开异常: {method_name}, err={e}", flush=True)
            continue

        if not cap.isOpened():
            try:
                cap.release()
            except Exception:
                pass
            print(f"[WARN] 摄像头打开失败: {method_name}", flush=True)
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if try_read_camera_frames(cap):
            print(f"[INIT] 摄像头打开方式: {method_name}", flush=True)
            return cap

        try:
            cap.release()
        except Exception:
            pass

        print(f"[WARN] 摄像头已打开但读帧失败: {method_name}", flush=True)

    return None


def open_preferred_camera():
    """
    自动打开摄像头：
    1. 扫描 /dev/video*
    2. 优先尝试 USB 摄像头
    3. 同时尝试按路径和按编号打开
    4. 实际试读帧确认可用
    5. 全部失败后回退到 FALLBACK_CAMERA_PATH
    """
    tested_paths = set()

    if AUTO_DETECT_USB_CAMERA:
        candidates = list_video_candidates()

        if candidates:
            print("[INIT] 检测到以下视频设备候选：", flush=True)
            for item in candidates:
                print(
                    f"[INIT]   path={item['path']} index={item['index']} "
                    f"is_usb={item['is_usb']} resolved={item['resolved']}",
                    flush=True
                )
        else:
            print("[WARN] 未扫描到 /dev/video* 候选设备", flush=True)

        for item in candidates:
            tested_paths.add(item["path"])
            cap = try_open_camera_candidate(item)
            if cap is not None:
                print(
                    f"[INIT] 已自动选择摄像头: {item['path']} "
                    f"(index={item['index']}, is_usb={item['is_usb']})",
                    flush=True
                )
                return cap, item["index"], item["path"]

    print(f"[WARN] 自动选择失败，回退到 FALLBACK_CAMERA_PATH={FALLBACK_CAMERA_PATH}", flush=True)

    fallback_name = os.path.basename(FALLBACK_CAMERA_PATH)
    try:
        fallback_index = int(fallback_name.replace("video", "")) if fallback_name.startswith("video") else -1
    except Exception:
        fallback_index = -1

    fallback_item = {
        "index": fallback_index,
        "path": FALLBACK_CAMERA_PATH,
        "is_usb": "unknown",
        "resolved": "",
    }

    cap = try_open_camera_candidate(fallback_item)
    if cap is None:
        raise RuntimeError(
            "自动检测和回退摄像头均失败。可能原因："
            "1) 摄像头被其他进程占用；"
            "2) 当前 /dev/videoX 不是视频流节点；"
            "3) OpenCV/V4L2 后端不兼容；"
            "4) USB 摄像头未正常输出。"
            "请先执行：systemctl status trash-ai.service；"
            "如服务正在运行，先 sudo systemctl stop trash-ai.service，"
            "再执行 v4l2-ctl --list-devices 查看节点。"
        )

    return cap, fallback_index, FALLBACK_CAMERA_PATH


# ===== 摄像头 =====
cap, camera_id, camera_path = open_preferred_camera()
print(f"[INIT] 摄像头 {camera_path} 打开成功", flush=True)
print("[INIT] 纯终端运行：可编辑代码里的开关和参数；调试图会保存为 debug_latest.jpg", flush=True)
print("[INIT] 点位布局: BIN1/BIN2 在待机点左侧, BIN3/BIN4 在待机点右侧；下发指令包含 DIR + RGB + TOL", flush=True)
for _bin_id, _meta in BIN_METADATA.items():
    print(
        f"[INIT] BIN{_bin_id}: DIR={_meta['direction_ascii']} COLOR={_meta['color_ascii']} "
        f"ratio_rgb={_meta['ratio_rgb']} ratio_tol={_meta['ratio_tolerance']}",
        flush=True
    )
print(
    f"[INIT] HOME: DIR={POINT_LAYOUT['HOME']['direction_ascii']} "
    f"COLOR={POINT_LAYOUT['HOME']['color_ascii']} "
    f"ratio_rgb={POINT_LAYOUT['HOME']['ratio_rgb']} "
    f"ratio_tol={POINT_LAYOUT['HOME']['ratio_tolerance']}",
    flush=True
)
print(f"[INIT] HOME 指令示例: {build_home_command()}", flush=True)
print("[INIT] 指令示例: [$TARGET_BIN:3,DIR:RIGHT,RGB:65,92,98,TOL:12,10,10]", flush=True)
print(f"[INIT] STM32 运行时配置文件: {STM32_RUNTIME_CONFIG_PATH}", flush=True)
print(
    f"[INIT] ROI 配置: WIDTH_RATIO={ROI_WIDTH_RATIO:.2f}, HEIGHT_RATIO={ROI_HEIGHT_RATIO:.2f}",
    flush=True
)

# ===== 参数 =====
history = deque(maxlen=5)

# 背景初始化/预热参数
camera_warmup_frames = 20
background_init_frames = 15

# 背景策略开关
# False：初始化后的背景保持固定，场景中的静态无关物体也会被当作背景
# True：在 WAIT_OBJECT 且确实空闲时缓慢更新背景
enable_auto_background_update = False

# 异常轮廓过滤总开关
# False 时，不启用 contour_ratio / box_ratio / full_frame 这些异常过滤
enable_abnormal_contour_filter = True

# 检测灵敏度参数
motion_binary_threshold = 18
bg_binary_threshold = 15
blur_kernel_size = 11
morph_kernel_size = 3
dilate_iterations = 1
close_iterations = 1

# 候选前景参数（比正式目标更宽松）
candidate_fg_pixels_threshold = 120
candidate_motion_freeze_threshold = 80

# 运动/目标参数
motion_threshold = 700
stable_motion_threshold = 550
big_motion_threshold = 12000
cooldown_after_big_motion = 0.20

min_contour_area = 500
object_hold_time = 0.30
stable_hold_time = 0.55
empty_hold_time = 1.0
infer_interval = 0.2

# 目标面积过滤参数（相对于 ROI）
min_box_ratio = 0.004
max_box_ratio = 0.78
full_frame_reject_ratio = 0.92
min_contour_ratio = 0.0015
max_contour_ratio = 0.75

# 额外过滤：贴边、中心区域、框稳定性
edge_margin_px = 6
center_x_min_ratio = 0.12
center_x_max_ratio = 0.88
center_y_min_ratio = 0.12
center_y_max_ratio = 0.88

box_center_move_threshold = 40.0
box_area_change_ratio_threshold = 0.60

# 分类参数
conf_threshold = 0.75
min_votes = 3

prev_gray = None
background_gray = None
background_acc = None
background_init_count = 0
warmup_count = 0
last_infer_time = 0
last_reject_log_time = 0
reject_log_interval = 1.0
last_big_motion_time = 0.0

# ===== 状态变量 =====
state = "WAIT_OBJECT"  # WAIT_OBJECT -> WAIT_STABLE -> CLASSIFYING -> WAIT_STM32_DONE
object_present_since = None
stable_since = None
empty_since = None

final_pred = "None"
final_conf = 0.0
final_bin = None
motion_pixels = 0
fg_pixels = 0
candidate_present = False
candidate_box = None
tracked_box = None
working_box = None
last_valid_box = None
box_is_stable = False

# 调试输出
last_print_time = 0
print_interval = 1.0

last_save_time = 0
save_interval = 2.0

# 串口运行时变量
ser = open_serial_port()
serial_rx_buffer = ""
target_sent = False
target_sent_time = 0.0
recognition_enabled = True
user_exit_requested = False
send_stm32_runtime_config_if_needed(ser, force=True)
start_console_input_worker()


def set_state(new_state, reason=""):
    global state
    if state != new_state:
        old_state = state
        state = new_state
        if reason:
            log_event(f"状态切换: {old_state} -> {new_state} | {reason}")
        else:
            log_event(f"状态切换: {old_state} -> {new_state}")


def reset_cycle(reason=""):
    global state, object_present_since, stable_since, empty_since
    global final_pred, final_conf, final_bin, last_infer_time
    global tracked_box, candidate_box, working_box, last_valid_box, box_is_stable, candidate_present, fg_pixels
    global target_sent, target_sent_time
    history.clear()
    state = "WAIT_OBJECT"
    object_present_since = None
    stable_since = None
    empty_since = None
    final_pred = "None"
    final_conf = 0.0
    final_bin = None
    last_infer_time = 0
    tracked_box = None
    candidate_box = None
    working_box = None
    last_valid_box = None
    box_is_stable = False
    candidate_present = False
    fg_pixels = 0
    target_sent = False
    target_sent_time = 0.0
    if reason:
        log_event(f"本轮重置: {reason}")
    else:
        log_event("本轮重置")


def print_runtime_help():
    print(
        "[CMD] 可用命令:\n"
        "  pause / stop          暂停摄像头识别，不再触发分类和投放\n"
        "  resume / start        恢复摄像头识别\n"
        "  reset                 向 STM32 发送 [$RESET]，并重置本轮识别\n"
        "  hard_reset            向 STM32 发送 [$HARD_RESET]，直接回初始 IDLE\n"
        "  ping                  向 STM32 发送 [$PING]\n"
        "  get_cfg               查询 STM32 当前运行时配置\n"
        "  cfg                   立即重读 stm32_runtime_config.ini 并下发\n"
        "  home_cfg              立即下发 HOME 颜色定义\n"
        "  color_debug           开启 STM32 颜色调试输出\n"
        "  stop_color            关闭 STM32 颜色调试输出\n"
        "  target <1-4>          手动发送目标桶指令，例如 target 3\n"
        "  send <raw>            发送原始串口消息，例如 send [$GET_CFG]\n"
        "  quit / exit           退出 Python 脚本",
        flush=True,
    )


def handle_user_command(command):
    global recognition_enabled, user_exit_requested
    global final_pred, final_conf, final_bin, target_sent, target_sent_time

    parts = command.split(maxsplit=1)
    verb = parts[0].lower()
    arg = parts[1].strip() if len(parts) > 1 else ""

    print(f"[USER] {command}", flush=True)

    if verb in ("help", "?"):
        print_runtime_help()
    elif verb in ("pause", "stop"):
        recognition_enabled = False
        if state != "WAIT_STM32_DONE":
            reset_cycle("用户暂停摄像头识别")
        log_event("摄像头识别已暂停")
    elif verb in ("resume", "start"):
        recognition_enabled = True
        if state != "WAIT_STM32_DONE":
            reset_cycle("用户恢复摄像头识别")
        log_event("摄像头识别已恢复")
    elif verb == "reset":
        send_serial_message(ser, CMD_OPI_RESET)
        reset_cycle("用户手动 RESET")
    elif verb in ("hard_reset", "hardreset"):
        send_serial_message(ser, CMD_OPI_HARD_RESET)
        reset_cycle("用户手动 HARD_RESET")
    elif verb == "ping":
        send_serial_message(ser, CMD_OPI_PING)
    elif verb == "get_cfg":
        send_serial_message(ser, "[$GET_CFG]")
    elif verb == "cfg":
        send_stm32_runtime_config_if_needed(ser, force=True)
    elif verb == "home_cfg":
        queue_stm32_command(build_home_command())
        log_event("已排队下发 HOME 颜色定义")
    elif verb == "color_debug":
        start_color_debug_session()
        send_serial_message(ser, "[$ColorDebug]")
    elif verb == "stop_color":
        send_serial_message(ser, "[$StopColor]")
        stop_color_debug_session()
    elif verb == "target":
        try:
            bin_id = int(arg)
        except ValueError:
            print("[CMD] target 命令需要桶号 1~4，例如 target 3", flush=True)
            return

        if bin_id not in BIN_METADATA:
            print("[CMD] 桶号必须是 1~4", flush=True)
            return

        final_pred = "MANUAL"
        final_conf = 1.0
        final_bin = bin_id
        target_sent = send_serial_message(ser, build_target_command(bin_id))
        target_sent_time = time.time()
        set_state("WAIT_STM32_DONE", f"用户手动发送目标桶 {bin_id}")
    elif verb == "send":
        if not arg:
            print("[CMD] send 命令需要原始消息，例如 send [$GET_CFG]", flush=True)
            return
        send_serial_message(ser, arg)
    elif verb in ("quit", "exit"):
        user_exit_requested = True
        log_event("用户请求退出")
    else:
        print(f"[CMD] 未知命令: {verb}，输入 help 查看可用命令", flush=True)


def process_user_commands():
    while True:
        try:
            command = console_command_queue.get_nowait()
        except queue.Empty:
            return

        handle_user_command(command)


while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] 读取摄像头画面失败", flush=True)
        break

    h, w, _ = frame.shape

    # ===== 可配置识别区域 ROI（默认居中） =====
    roi_width = max(1, min(w, int(w * ROI_WIDTH_RATIO)))
    roi_height = max(1, min(h, int(h * ROI_HEIGHT_RATIO)))
    x1 = max(0, (w - roi_width) // 2)
    y1 = max(0, (h - roi_height) // 2)
    x2 = min(w, x1 + roi_width)
    y2 = min(h, y1 + roi_height)
    roi = frame[y1:y2, x1:x2]
    roi_h, roi_w = roi.shape[:2]
    roi_area = roi_h * roi_w

    center_x_min = roi_w * center_x_min_ratio
    center_x_max = roi_w * center_x_max_ratio
    center_y_min = roi_h * center_y_min_ratio
    center_y_max = roi_h * center_y_max_ratio

    # ===== 灰度图 =====
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (blur_kernel_size, blur_kernel_size), 0)

    # ===== 轮询 STM32 串口消息 =====
    serial_messages, serial_rx_buffer = poll_serial_messages(ser, serial_rx_buffer)
    for stm32_msg in serial_messages:
        reset_reason = handle_stm32_message(stm32_msg)
        if reset_reason:
            reset_cycle(reset_reason)

    process_stm32_command_queue(ser)
    send_stm32_runtime_config_if_needed(ser)
    process_user_commands()
    if user_exit_requested:
        break

    # ===== 摄像头预热 =====
    if warmup_count < camera_warmup_frames:
        warmup_count += 1
        if warmup_count == 1:
            print(f"[INIT] 摄像头预热开始，需要丢弃前 {camera_warmup_frames} 帧", flush=True)
        elif warmup_count == camera_warmup_frames:
            print("[INIT] 摄像头预热完成，开始初始化背景", flush=True)
        continue

    # ===== 初始背景用多帧平均 =====
    if background_gray is None:
        if background_acc is None:
            background_acc = gray.astype(np.float32)
            background_init_count = 1
            print(f"[INIT] 正在采集背景帧 1/{background_init_frames}", flush=True)
        else:
            background_acc += gray.astype(np.float32)
            background_init_count += 1
            if background_init_count == 1 or background_init_count % 5 == 0:
                print(f"[INIT] 正在采集背景帧 {background_init_count}/{background_init_frames}", flush=True)

        prev_gray = gray.copy()

        if background_init_count >= background_init_frames:
            background_gray = apply_current_scene_as_background(
                (background_acc / background_init_count).astype(np.uint8)
            )
            background_acc = None
            print(f"[INIT] 背景初始化完成（平均 {background_init_count} 帧，当前空场景已固定为背景）", flush=True)
        continue

    # ===== 用户暂停识别：保留摄像头和串口轮询，但不做检测/分类 =====
    if not recognition_enabled:
        now = time.time()
        debug_frame = frame.copy()
        cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(debug_frame, "Recognition: PAUSED", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_frame, "Type resume to start", (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        if now - last_print_time > print_interval:
            print("[STATUS] RecognitionPaused=True", flush=True)
            last_print_time = now

        if now - last_save_time > save_interval:
            cv2.imwrite("debug_latest.jpg", debug_frame)
            last_save_time = now

        prev_gray = gray.copy()
        continue

    # ===== LOCK 后等待 STM32 完成：此阶段不再做视觉检测 =====
    if state == "WAIT_STM32_DONE":
        now = time.time()

        debug_frame = frame.copy()
        cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(debug_frame, f"State: {state}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_frame, f"Waiting STM32...", (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(debug_frame, f"Result: {final_pred}", (20, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(debug_frame, f"Conf: {final_conf:.2%}", (20, 135),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        wait_meta = get_bin_metadata(final_bin)
        wait_dir = wait_meta["direction_ascii"] if wait_meta else "UNKNOWN"
        wait_rgb = wait_meta["ratio_rgb"] if wait_meta else ("?", "?", "?")
        wait_tol = wait_meta["ratio_tolerance"] if wait_meta else ("?", "?", "?")

        cv2.putText(debug_frame, f"Bin: {final_bin}", (20, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(debug_frame, f"Dir: {wait_dir}", (20, 205),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(debug_frame, f"RGB: {wait_rgb}", (20, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(debug_frame, f"TOL: {wait_tol}", (20, 275),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(debug_frame, f"TargetSent: {target_sent}", (20, 310),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        if now - last_print_time > print_interval:
            wait_meta = get_bin_metadata(final_bin)
            wait_dir = wait_meta["direction_ascii"] if wait_meta else "UNKNOWN"
            wait_rgb = wait_meta["ratio_rgb"] if wait_meta else ("?", "?", "?")
            wait_tol = wait_meta["ratio_tolerance"] if wait_meta else ("?", "?", "?")
            print(
                f"[STATUS] State={state} WaitingSTM32=True "
                f"TargetSent={target_sent} "
                f"Result={final_pred} Conf={final_conf:.2%} Bin={final_bin} "
                f"Dir={wait_dir} RGB={wait_rgb} TOL={wait_tol}",
                flush=True
            )
            last_print_time = now

        if now - last_save_time > save_interval:
            cv2.imwrite("debug_latest.jpg", debug_frame)
            last_save_time = now

        continue

    motion_detected = False
    motion_pixels = 0
    now = time.time()

    # ===== 运动检测 =====
    if prev_gray is not None:
        diff_motion = cv2.absdiff(prev_gray, gray)
        _, thresh_motion = cv2.threshold(diff_motion, motion_binary_threshold, 255, cv2.THRESH_BINARY)
        motion_pixels = cv2.countNonZero(thresh_motion)
        if motion_pixels > motion_threshold:
            motion_detected = True
        if motion_pixels > big_motion_threshold:
            last_big_motion_time = now

    prev_gray = gray.copy()

    # ===== 前景/物体检测 =====
    diff_bg = cv2.absdiff(background_gray, gray)
    _, thresh_obj = cv2.threshold(diff_bg, bg_binary_threshold, 255, cv2.THRESH_BINARY)

    kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
    thresh_obj = cv2.morphologyEx(thresh_obj, cv2.MORPH_OPEN, kernel)
    thresh_obj = cv2.morphologyEx(thresh_obj, cv2.MORPH_CLOSE, kernel, iterations=close_iterations)
    thresh_obj = cv2.dilate(thresh_obj, kernel, iterations=dilate_iterations)

    fg_pixels = cv2.countNonZero(thresh_obj)
    candidate_present = fg_pixels > candidate_fg_pixels_threshold

    contours, _ = cv2.findContours(thresh_obj, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidate_box = None
    tracked_box = None
    max_candidate_area = 0
    max_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < max(200, int(min_contour_area * 0.5)):
            continue

        x, y, bw_box, bh_box = cv2.boundingRect(cnt)
        box_area = bw_box * bh_box
        box_ratio = box_area / roi_area
        contour_ratio = area / roi_area

        # 先挑一个更宽松的候选框，用于早期“看到物体”
        if box_ratio <= full_frame_reject_ratio and contour_ratio <= 0.85:
            if area > max_candidate_area:
                max_candidate_area = area
                candidate_box = (x, y, bw_box, bh_box)

        # 再做更严格的正式目标框过滤
        if area < min_contour_area:
            continue

        if abnormal_contour_filter_enabled():
            if contour_ratio < min_contour_ratio or contour_ratio > max_contour_ratio:
                if now - last_reject_log_time > reject_log_interval:
                    log_event(f"忽略异常轮廓: contour_ratio={contour_ratio:.2%}, area={area:.0f}")
                    last_reject_log_time = now
                continue

            if box_ratio > full_frame_reject_ratio:
                if now - last_reject_log_time > reject_log_interval:
                    log_event(f"忽略接近整屏的异常目标框: box=({x},{y},{bw_box},{bh_box}), ratio={box_ratio:.2%}")
                    last_reject_log_time = now
                continue

            if box_ratio < min_box_ratio or box_ratio > max_box_ratio:
                if now - last_reject_log_time > reject_log_interval:
                    log_event(f"忽略目标框面积比例异常: box=({x},{y},{bw_box},{bh_box}), ratio={box_ratio:.2%}")
                    last_reject_log_time = now
                continue

        # 贴边目标忽略
        touches_edge = (
            x <= edge_margin_px or
            y <= edge_margin_px or
            x + bw_box >= roi_w - edge_margin_px or
            y + bh_box >= roi_h - edge_margin_px
        )
        if touches_edge:
            if now - last_reject_log_time > reject_log_interval:
                log_event(f"忽略贴边目标: box=({x},{y},{bw_box},{bh_box})")
                last_reject_log_time = now
            continue

        # 中央区域过滤
        cx = x + bw_box / 2.0
        cy = y + bh_box / 2.0
        center_ok = (
            center_x_min <= cx <= center_x_max and
            center_y_min <= cy <= center_y_max
        )
        if not center_ok:
            if now - last_reject_log_time > reject_log_interval:
                log_event(f"忽略非中央目标: center=({cx:.1f},{cy:.1f})")
                last_reject_log_time = now
            continue

        if area > max_area:
            max_area = area
            tracked_box = (x, y, bw_box, bh_box)

    working_box = tracked_box if tracked_box is not None else candidate_box

    # ===== 框稳定性判断 =====
    box_is_stable = False
    if working_box is not None:
        if last_valid_box is not None:
            bx, by, bw_box, bh_box = working_box
            lbx, lby, lbw, lbh = last_valid_box

            cx = bx + bw_box / 2.0
            cy = by + bh_box / 2.0
            lcx = lbx + lbw / 2.0
            lcy = lby + lbh / 2.0

            center_move = ((cx - lcx) ** 2 + (cy - lcy) ** 2) ** 0.5
            area = bw_box * bh_box
            last_area = lbw * lbh
            area_change_ratio = abs(area - last_area) / max(last_area, 1)

            box_is_stable = (
                center_move <= box_center_move_threshold and
                area_change_ratio <= box_area_change_ratio_threshold
            )
        last_valid_box = working_box
    else:
        last_valid_box = None

    object_present = working_box is not None

    # ===== 状态机 =====
    if state == "WAIT_OBJECT":
        if candidate_present:
            if object_present_since is None:
                object_present_since = now
                log_event(f"检测到候选前景进入识别区，开始计时确认，fg_pixels={fg_pixels}")
            elif now - object_present_since >= object_hold_time:
                set_state("WAIT_STABLE", f"候选前景持续出现 {now - object_present_since:.2f}s")
                stable_since = None
        else:
            object_present_since = None

    elif state == "WAIT_STABLE":
        if not candidate_present:
            reset_cycle("等待稳定时候选前景消失")
        else:
            cooldown_left = cooldown_after_big_motion - (now - last_big_motion_time)
            if not object_present:
                if stable_since is not None:
                    log_event("候选前景仍在，但正式目标框尚未形成，稳定计时清零")
                stable_since = None
            elif cooldown_left > 0:
                if stable_since is not None:
                    log_event(f"大幅运动后冷却中，稳定计时清零，剩余 {cooldown_left:.2f}s")
                stable_since = None
            else:
                relaxed_box_ok = box_is_stable or motion_pixels <= stable_motion_threshold * 0.35
                if motion_pixels <= stable_motion_threshold and relaxed_box_ok:
                    if stable_since is None:
                        stable_since = now
                        if box_is_stable:
                            log_event(f"目标开始稳定，motion_pixels={motion_pixels}, box_stable=True")
                        else:
                            log_event(f"目标进入宽松稳定判断，motion_pixels={motion_pixels}")
                    elif now - stable_since >= stable_hold_time:
                        set_state("CLASSIFYING", f"目标已稳定 {now - stable_since:.2f}s，开始分类")
                        history.clear()
                else:
                    if stable_since is not None:
                        if motion_pixels > stable_motion_threshold:
                            log_event(f"目标重新晃动，稳定计时清零，motion_pixels={motion_pixels}")
                        elif not relaxed_box_ok:
                            log_event("目标框位置/大小仍不稳定，稳定计时清零")
                    stable_since = None

    elif state == "CLASSIFYING":
        if not candidate_present:
            reset_cycle("分类过程中候选前景消失")
        elif not object_present:
            if now - last_reject_log_time > reject_log_interval:
                log_event("分类过程中候选前景仍在，但正式目标框暂时丢失，等待恢复")
                last_reject_log_time = now
        elif motion_pixels > big_motion_threshold:
            if now - last_reject_log_time > reject_log_interval:
                log_event(f"分类阶段检测到大幅运动，暂缓推理: motion_pixels={motion_pixels}")
                last_reject_log_time = now
        else:
            if now - last_infer_time >= infer_interval:
                bx, by, bw_box, bh_box = working_box
                obj_roi = roi[by:by + bh_box, bx:bx + bw_box]

                if obj_roi.size > 0:
                    rgb = cv2.cvtColor(obj_roi, cv2.COLOR_BGR2RGB)
                    pil_img = Image.fromarray(rgb)

                    input_tensor = preprocess(pil_img)
                    outputs = session.run(None, {input_name: input_tensor})[0]
                    probs = softmax(outputs)

                    pred = int(np.argmax(probs, axis=1)[0])
                    pred_conf = float(np.max(probs, axis=1)[0])

                    if pred_conf >= conf_threshold:
                        pred_label = class_names[pred]
                        history.append((pred_label, pred_conf))
                        log_infer(
                            f"有效分类: pred={pred_label}, conf={pred_conf:.2%}, "
                            f"votes={len(history)}/{history.maxlen}"
                        )
                    else:
                        log_infer(f"低置信度，跳过: conf={pred_conf:.2%}")

                    last_infer_time = now

                    if len(history) >= min_votes:
                        labels = [x[0] for x in history]
                        counter = Counter(labels)
                        label, count = counter.most_common(1)[0]

                        if count >= min_votes:
                            confs = [c for l, c in history if l == label]
                            avg_conf = sum(confs) / len(confs)
                            final_pred = label
                            final_conf = avg_conf
                            final_bin = class_to_bin.get(label, None)

                            target_meta = get_bin_metadata(final_bin)
                            if target_meta is None:
                                log_event(
                                    f"分类锁定: label={final_pred}, conf={final_conf:.2%}, "
                                    f"bin={final_bin}"
                                )
                            else:
                                log_event(
                                    f"分类锁定: label={final_pred}, conf={final_conf:.2%}, "
                                    f"bin={final_bin}, dir={target_meta['direction_ascii']}, "
                                    f"ratio_rgb={target_meta['ratio_rgb']}, "
                                    f"ratio_tol={target_meta['ratio_tolerance']}"
                                )

                            target_cmd = build_target_command(final_bin)
                            target_sent = send_serial_message(ser, target_cmd)
                            target_sent_time = now
                            set_state("WAIT_STM32_DONE", "已锁定结果并发送 TARGET_BIN，停止视觉检测，等待 STM32 完成")

    elif state == "WAIT_STM32_DONE":
        # 此状态已在前面提前 continue，这里保留占位，便于后续扩展。
        pass

    # ===== 背景更新策略 =====
    if allow_background_update(state, candidate_present, motion_pixels):
        background_gray = cv2.addWeighted(background_gray, 0.995, gray, 0.005, 0)

    # ===== 调试图 =====
    debug_frame = frame.copy()
    cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    cgx1 = int(x1 + center_x_min)
    cgy1 = int(y1 + center_y_min)
    cgx2 = int(x1 + center_x_max)
    cgy2 = int(y1 + center_y_max)
    cv2.rectangle(debug_frame, (cgx1, cgy1), (cgx2, cgy2), (0, 255, 255), 1)

    box_ratio_text = "None"
    if working_box is not None:
        bx, by, bw_box, bh_box = working_box
        gx1 = x1 + bx
        gy1 = y1 + by
        gx2 = gx1 + bw_box
        gy2 = gy1 + bh_box
        box_color = (0, 255, 0) if tracked_box is not None else (0, 165, 255)
        cv2.rectangle(debug_frame, (gx1, gy1), (gx2, gy2), box_color, 2)
        box_ratio_text = f"{(bw_box * bh_box) / roi_area:.2%}"

    cooldown_left = max(0.0, cooldown_after_big_motion - (now - last_big_motion_time))

    cv2.putText(debug_frame, f"State: {state}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(debug_frame, f"MotionPixels: {motion_pixels}", (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"FGPixels: {fg_pixels}", (20, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Candidate: {candidate_present}", (20, 135),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"StrictBox: {tracked_box is not None}", (20, 170),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"BoxRatio: {box_ratio_text}", (20, 205),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"BoxStable: {box_is_stable}", (20, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Cooldown: {cooldown_left:.2f}s", (20, 275),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"AutoBG: {enable_auto_background_update}", (20, 310),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"AbnormalFilter: {enable_abnormal_contour_filter}", (20, 345),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(debug_frame, f"Result: {final_pred}", (20, 380),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(debug_frame, f"Conf: {final_conf:.2%}", (20, 415),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(debug_frame, f"Bin: {final_bin}", (20, 450),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # ===== 纯终端状态打印 =====
    if now - last_print_time > print_interval:
        box_info = "None"
        box_ratio_info = "None"
        if working_box is not None:
            bx, by, bw_box, bh_box = working_box
            box_info = f"({bx},{by},{bw_box},{bh_box})"
            box_ratio_info = f"{(bw_box * bh_box / roi_area):.2%}"

        print(
            f"[STATUS] "
            f"State={state} "
            f"MotionPixels={motion_pixels} "
            f"FGPixels={fg_pixels} "
            f"CandidatePresent={candidate_present} "
            f"ObjectPresent={object_present} "
            f"StrictBox={tracked_box is not None} "
            f"Box={box_info} "
            f"BoxRatio={box_ratio_info} "
            f"BoxStable={box_is_stable} "
            f"Cooldown={cooldown_left:.2f}s "
            f"AutoBG={enable_auto_background_update} "
            f"AbnormalFilter={enable_abnormal_contour_filter} "
            f"HistorySize={len(history)} "
            f"Result={final_pred} "
            f"Conf={final_conf:.2%} "
            f"Bin={final_bin}",
            flush=True
        )
        last_print_time = now

    if now - last_save_time > save_interval:
        cv2.imwrite("debug_latest.jpg", debug_frame)
        last_save_time = now

cap.release()
if ser is not None:
    try:
        ser.close()
    except Exception:
        pass
print("[EXIT] 程序结束", flush=True)
