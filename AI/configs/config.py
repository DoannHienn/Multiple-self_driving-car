from pathlib import Path

# ===== UDP / NETWORK =====
LISTEN_IP   = "192.168.5.113"
LISTEN_PORT = 3000
ESP_IP      = "192.168.5.4"
ESP_PORT    = 3001
END_MARKER  = b"\xFF\xD9"

BUFFER_SIZE = 2048  
RBUF_BYTES  = 8 * 1024 * 1024
NO_PACKET_TIMEOUT_S = 5.0
NO_FRAME_TIMEOUT_S  = 4.0
MAX_ACCUM_BYTES     = 3 * 1024 * 1024

# ===== PATHS =====
ROOT = Path(__file__).resolve().parents[2]
LOG_DIR    = ROOT / "AI" / "logs"
CALIB_PATH = ROOT / "AI" / "camera_intrinsics.npz"

# "yolov8" | "pidnet" | "twinlite" | "bisenet" |
# Tìm đến phần ===== PATHS ===== và thêm "segformer" vào dictionary
LANE_MODEL = "segformer"  # Thay đổi model mặc định tại đây
LANE_WEIGHTS = ROOT / "AI" / "LaneDetection" / "Lane_weight" / {
    "yolov8":  "Yolo_v8/best.pt",
    "pidnet":  "PIDNet/best.pt",
    "twinlite":"TwinLite/best.pth",
    "bisenet": "BiseNet/best.pth",
    "segformer": "SegFormer/final_esp32_model" # Tên thư mục chứa config.json và model.safetensors
}[LANE_MODEL.lower()]

# Thêm một phần cấu hình riêng cho SegFormer ở cuối file (nếu cần tùy chỉnh device)

# ===== RUNTIME =====
DEVICE = "0"
SHOW   = True
WIN_NAME = "ACE LANE"
WIN_W, WIN_H = 1280, 720

# ===== YOLOv8 =====
IMGSZ = 640
CONF  = 0.18

# ===== PIDNet =====
PIDNET_H   = 320
PIDNET_W   = 416
PIDNET_THR = 0.75
PIDNET_ARCH = "pidnet_small"

# ===== TwinLiteNet =====
TWIN_H   = 320
TWIN_W   = 416
TWIN_THR = 0.50
TWIN_NUM_CLASSES = 2 

# ===== BiseNetV2 =====
BISENET_H = 256
BISENET_W = 256
BISENET_NUM_CLASSES = 2
