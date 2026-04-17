import cv2
import numpy as np
import torch
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation

class SegformerBackend:
    def __init__(self, weights_dir: str, device: str = "cuda"):
        """
        weights_dir: Đường dẫn tới thư mục chứa config.json và model.safetensors
                     (Ví dụ: './final_esp32_model')
        """
        # 1. Cấu hình thiết bị
        if device in ("cpu", "cuda", "cuda:0", "cuda:1"):
            self.device = torch.device(device)
        else:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        print(f"[SegFormer] Đang tải mô hình từ {weights_dir} lên {self.device}...")

        # 2. Load Processor và Model từ thư mục weights
        try:
            self.processor = SegformerImageProcessor.from_pretrained(weights_dir)
            self.model = SegformerForSemanticSegmentation.from_pretrained(weights_dir)
            self.model.to(self.device).eval()
        except Exception as e:
            raise RuntimeError(f"[SegFormer] Lỗi khi load weights: {e}")

        # 3. Tối ưu hóa CUDA (nếu dùng GPU)
        if self.device.type == 'cuda':
            try:
                torch.backends.cudnn.benchmark = True
                torch.set_float32_matmul_precision("high")
            except Exception:
                pass

        # 4. Warmup (Khởi động nóng mô hình để lần chạy đầu tiên không bị giật)
        try:
            print("[SegFormer] Đang Warmup...")
            with torch.inference_mode():
                # Segformer có thể nhận kích thước linh hoạt, ta dùng tạm 256x256 để warmup
                dummy = torch.zeros(1, 3, 256, 256, device=self.device)
                _ = self.model(dummy)
        except Exception as e:
            print(f"[SegFormer] Cảnh báo Warmup: {e}")

    def infer_mask01(self, frame_bgr: np.ndarray) -> np.ndarray:
        """
        Hàm cốt lõi: Nhận ảnh BGR từ Camera/Video -> Trả về mask nhị phân (0 và 1)
        """
        if frame_bgr is None or frame_bgr.size == 0:
            return np.zeros((1, 1), np.uint8)

        H, W = frame_bgr.shape[:2]

        # Chuyển BGR (OpenCV) sang RGB (chuẩn của Hugging Face)
        img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        with torch.inference_mode():
            # Bước 1: Tiền xử lý bằng processor của Segformer
            inputs = self.processor(images=img_rgb, return_tensors="pt").to(self.device)

            # Bước 2: Chạy qua mô hình
            outputs = self.model(**inputs)
            logits = outputs.logits

            # Bước 3: Phóng to kết quả (logits) về lại đúng kích thước H, W của frame gốc
            upsampled_logits = torch.nn.functional.interpolate(
                logits,
                size=(H, W),
                mode="bilinear",
                align_corners=False,
            )

            # Bước 4: Lấy class có xác suất cao nhất (0 là nền, 1 là làn đường)
            pred = upsampled_logits.argmax(dim=1)[0].cpu().numpy().astype(np.uint8)

        # Đảm bảo đầu ra chỉ toàn 0 và 1, định dạng uint8 theo yêu cầu của pipeline
        mask01 = (pred == 1).astype(np.uint8)
        
        return mask01

    def name(self) -> str:
        return "SegformerBackend"