import numpy as np
import cv2
from ultralytics import YOLO

class VehicleDetector():
    def __init__(self, model_path):
        print(f"[AI Core] Loading YOLOv8 model from: {model_path}")
        self.model = YOLO(model_path) 
        self.classes = self.model.names
        print("[AI Core] Classes loaded:", self.classes)
        
        self.SRC_RATIOS = ((0.35, 0.45), (0.0, 1.00), (1.0, 1.00), (0.65, 0.45))
        
        self.DST_RATIOS = ((0.25, 0.00), (0.25, 1.00), (0.75, 1.00), (0.75, 0.00))
        self.M = None
        self._wh = None
        self.distance_history = {}

    def _ensure_M(self, W, H):
        if self._wh == (W, H) and self.M is not None:
            return
        src = np.float32([(x*W, y*H) for x, y in self.SRC_RATIOS])
        dst = np.float32([(x*W, y*H) for x, y in self.DST_RATIOS])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self._wh = (W, H)

    def detect_and_track(self, frame, conf_threshold=0.4):
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
        A = 0.000392
        B = -0.478032
        C = 155.789672
        raw_distance = (A * (y2 ** 2)) + (B * y2) + C
        alpha = 0.3 
        if track_id not in self.distance_history:
            self.distance_history[track_id] = raw_distance
        else:
            smoothed = (alpha * raw_distance) + ((1 - alpha) * self.distance_history[track_id])
            self.distance_history[track_id] = smoothed
            
        return self.distance_history[track_id]
    