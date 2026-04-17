# import numpy as np
# import cv2
# from ultralytics import YOLO

# class VehicleDetector():
#     def __init__(self, model_path):
#         print(f"Loading YOLOv8 model from: {model_path}")
#         # YOLOv8 tự động nhận diện phần cứng (CPU/CUDA)
#         self.model = YOLO(model_path) 
#         self.classes = self.model.names
#         print("Classes loaded:", self.classes)
        
#         # --- MA TRẬN BIRD'S EYE VIEW (BEV) ---
#         # Copy từ file lane_geometry.py để đồng bộ không gian bám làn
#         self.SRC_RATIOS = ((0.20, 0.58), (0.10, 0.90), (0.90, 0.90), (0.80, 0.58))
#         self.DST_RATIOS = ((0.25, 0.00), (0.25, 1.00), (0.75, 1.00), (0.75, 0.00))
#         self.M = None
#         self._wh = None
        
#         # Bộ nhớ để lọc nhiễu EMA (Làm mượt giá trị khoảng cách)
#         self.distance_history = {}

#     def _ensure_M(self, W, H):
#         """Khởi tạo ma trận biến đổi phối cảnh (chỉ chạy 1 lần)"""
#         if self._wh == (W, H) and self.M is not None:
#             return
#         src = np.float32([(x*W, y*H) for x, y in self.SRC_RATIOS])
#         dst = np.float32([(x*W, y*H) for x, y in self.DST_RATIOS])
#         self.M = cv2.getPerspectiveTransform(src, dst)
#         self._wh = (W, H)

#     def detect_and_track(self, frame, conf_threshold=0.4):
#         """Nhận diện vật thể và gán ID bám sát"""
#         results = self.model.track(frame, persist=True, tracker="bytetrack.yaml", 
#                                    conf=conf_threshold, verbose=False)
        
#         detections = []
#         annotated_frame = frame.copy()

#         if results[0].boxes is not None and results[0].boxes.id is not None:
#             boxes = results[0].boxes.xyxy.cpu().numpy()
#             track_ids = results[0].boxes.id.int().cpu().tolist()
#             confs = results[0].boxes.conf.cpu().tolist()
#             class_ids = results[0].boxes.cls.int().cpu().tolist()

#             for box, track_id, conf, class_id in zip(boxes, track_ids, confs, class_ids):
#                 x1, y1, x2, y2 = map(int, box)
#                 class_label = self.classes[class_id]
                
#                 detections.append({
#                     'bbox': [x1, y1, x2, y2],
#                     'track_id': track_id,
#                     'conf': conf,
#                     'class_label': class_label
#                 })

#             annotated_frame = results[0].plot()

#         return detections, annotated_frame

#     def calculate_distance_bev(self, cx, y2, frame_width, frame_height, track_id):
#         """Tính khoảng cách thực tế (cm) bằng thuật toán BEV ADAS"""
#         self._ensure_M(frame_width, frame_height)
        
#         # Tọa độ điểm chạm đất của xe đồ chơi
#         point_original = np.array([[[cx, float(y2)]]], dtype=np.float32)
        
#         # Bẻ không gian từ nhìn xiên sang nhìn thẳng từ trên xuống
#         point_bev = cv2.perspectiveTransform(point_original, self.M)
#         y_bev = point_bev[0][0][1]
        
#         # Khoảng cách pixel từ đáy màn hình BEV (mũi xe) tới vật cản
#         distance_pixel = frame_height - y_bev
#         if distance_pixel <= 0: return 9999
        
#         # Tính tỷ lệ thực tế (Làn đường 20cm chiếm 50% độ rộng BEV)
#         pixel_per_lane = frame_width * 0.5 
#         cm_per_pixel = 10.0 / pixel_per_lane 
        
#         raw_distance = distance_pixel * cm_per_pixel
        
#         # Áp dụng bộ lọc EMA chống nhảy số giật cục
#         alpha = 0.3 
#         if track_id not in self.distance_history:
#             self.distance_history[track_id] = raw_distance
#         else:
#             smoothed = (alpha * raw_distance) + ((1 - alpha) * self.distance_history[track_id])
#             self.distance_history[track_id] = smoothed
            
#         return self.distance_history[track_id]
import numpy as np
import cv2
from ultralytics import YOLO

class VehicleDetector():
    def __init__(self, model_path):
        print(f"[AI Core] Loading YOLOv8 model from: {model_path}")
        self.model = YOLO(model_path) 
        self.classes = self.model.names
        print("[AI Core] Classes loaded:", self.classes)
        
        # --- MA TRẬN BIRD'S EYE VIEW (BEV) ---
        # Bốn điểm tạo thành hình thang trên mặt đường camera thu được
        # CẦN HIỆU CHUẨN: Điều chỉnh tỷ lệ này khớp với góc cắm camera thực tế của xe
        self.SRC_RATIOS = ((0.20, 0.58), (0.10, 0.90), (0.90, 0.90), (0.80, 0.58))
        
        # Bốn điểm đích tạo thành hình chữ nhật nhìn từ trên không xuống
        self.DST_RATIOS = ((0.25, 0.00), (0.25, 1.00), (0.75, 1.00), (0.75, 0.00))
        self.M = None
        self._wh = None
        
        # Bộ nhớ lọc nhiễu EMA (Exponential Moving Average) - Chống nhảy số
        self.distance_history = {}

    def _ensure_M(self, W, H):
        """Khởi tạo ma trận biến đổi phối cảnh IPM (Chạy 1 lần)"""
        if self._wh == (W, H) and self.M is not None:
            return
        src = np.float32([(x*W, y*H) for x, y in self.SRC_RATIOS])
        dst = np.float32([(x*W, y*H) for x, y in self.DST_RATIOS])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self._wh = (W, H)

    def detect_and_track(self, frame, conf_threshold=0.4):
        """Nhận diện vật thể và gán ID bám sát bằng ByteTrack"""
        results = self.model.track(frame, persist=True, tracker="bytetrack.yaml", 
                                   conf=conf_threshold, verbose=False,classes=[0, 1,2])
        
        detections = []
        annotated_frame = frame.copy()

        if results[0].boxes is not None and results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            track_ids = results[0].boxes.id.int().cpu().tolist()
            confs = results[0].boxes.conf.cpu().tolist()
            class_ids = results[0].boxes.cls.int().cpu().tolist()

            for box, track_id, conf, class_id in zip(boxes, track_ids, confs, class_ids):
                x1, y1, x2, y2 = map(int, box)
                class_label = self.classes[class_id]
                
                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'track_id': track_id,
                    'conf': conf,
                    'class_label': class_label
                })

            annotated_frame = results[0].plot()

        return detections, annotated_frame

    def calculate_distance_bev(self, cx, y2, frame_width, frame_height, track_id):
        """Tính khoảng cách thực tế (cm) bằng thuật toán BEV ADAS"""
        self._ensure_M(frame_width, frame_height)
        
        # Tọa độ điểm chạm đất của vật cản
        point_original = np.array([[[float(cx), float(y2)]]], dtype=np.float32)
        
        # Áp dụng ma trận nội suy: Bẻ từ nhìn xiên -> Nhìn thẳng từ trên xuống
        point_bev = cv2.perspectiveTransform(point_original, self.M)
        y_bev = point_bev[0][0][1]
        
        # Khoảng cách pixel trên mặt phẳng BEV (Từ mép dưới camera tới vật thể)
        distance_pixel = frame_height - y_bev
        if distance_pixel <= 0: 
            return 9999.0
        
        # --- CẦN HIỆU CHUẨN THỰC TẾ ---
        # Công thức: Khoảng cách thực = Số pixel * (Số cm / 1 pixel)
        # Giả sử: 1/2 chiều rộng màn hình BEV tương đương với 10cm thực tế trên sa bàn
        pixel_per_lane = frame_width * 0.5 
        cm_per_pixel = 10.0 / pixel_per_lane 
        raw_distance = distance_pixel * cm_per_pixel
        
        # Lọc nhiễu EMA (Giúp khoảng cách giảm mượt mà, không bị giật lùi)
        alpha = 0.3 
        if track_id not in self.distance_history:
            self.distance_history[track_id] = raw_distance
        else:
            smoothed = (alpha * raw_distance) + ((1 - alpha) * self.distance_history[track_id])
            self.distance_history[track_id] = smoothed
            
        return self.distance_history[track_id]