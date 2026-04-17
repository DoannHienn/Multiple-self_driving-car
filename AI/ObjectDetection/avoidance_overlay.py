# import cv2
# import time

# class AvoidanceOverlay:
#     def __init__(self):
#         self.color_blue = (255, 0, 0)
#         self.color_red = (0, 0, 255)
#         self.color_green = (0, 255, 0)
#         self.color_yellow = (0, 255, 255)

#     def draw(self, frame, poly_pts, use_dynamic_roi, heading, cmd, min_distance, fps):
#         annotated_frame = frame.copy()
#         height, width = annotated_frame.shape[:2]

#         # Xác định chế độ
#         if use_dynamic_roi:
#             roi_color = self.color_blue
#             mode_text = f"MODE: DYNAMIC (Angle: {heading:+.1f})"
#             mode_color = self.color_yellow
#         else:
#             roi_color = self.color_red
#             mode_text = "MODE: STATIC ROI (FIXED)"
#             mode_color = self.color_red

#         # Vẽ Vùng quét ROI
#         cv2.polylines(annotated_frame, [poly_pts], isClosed=True, color=roi_color, thickness=2)
        
#         # In HUD Chế độ & FPS
#         cv2.putText(annotated_frame, mode_text, (width - 450, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
#         cv2.putText(annotated_frame, f'FPS: {int(fps)}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_green, 2)

#         # In Trạng thái Phanh
#         if cmd == 8: # CMD_STOP
#             cv2.putText(annotated_frame, f'EMERGENCY STOP', (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_red, 2)
#         else:
#             cv2.putText(annotated_frame, f'ALL CLEAR', (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_green, 2)

#         # In Khoảng cách
#         if min_distance != 9999.0:
#             cv2.putText(annotated_frame, f'Dist: {min_distance:.1f} cm', (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_yellow, 2)

#         return annotated_frame
import cv2

class AvoidanceOverlay:
    def __init__(self):
        self.color_blue = (255, 0, 0)
        self.color_red = (0, 0, 255)
        self.color_green = (0, 255, 0)
        self.color_yellow = (0, 255, 255)

    def draw(self, frame, poly_pts, use_dynamic_roi, heading, cmd, active_objects, speed, fps, acc_enabled=True): # Thêm tham số acc_enabled
        annotated_frame = frame.copy()
        h, w = annotated_frame.shape[:2]

        # 1. Vẽ ROI
        roi_color = self.color_blue if use_dynamic_roi else self.color_red
        cv2.polylines(annotated_frame, [poly_pts], True, roi_color, 2)
        
        # 2. Vẽ khoảng cách cho TỪNG vật thể
        for obj in active_objects:
            x1, y1, x2, y2 = obj['bbox']
            dist_txt = f"{obj['dist']:.1f}cm"
            # Vẽ nền đen nhỏ cho chữ dễ đọc
            cv2.putText(annotated_frame, dist_txt, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3)
            cv2.putText(annotated_frame, dist_txt, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.color_yellow, 2)

        # 3. Hiển thị Tốc độ & Trạng thái
        speed_txt = f"SPEED: {int(speed)} PWM" if speed > 0 else "SPEED: LANE CTRL"
        cv2.putText(annotated_frame, speed_txt, (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_yellow, 2)
        
        mode_txt = f"MODE: DYNAMIC (Angle: {heading:+.1f})" if use_dynamic_roi else "MODE: STATIC (FIXED)"
        cv2.putText(annotated_frame, mode_txt, (w - 450, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, roi_color, 2)
        cv2.putText(annotated_frame, f'FPS: {int(fps)}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_green, 2)
        

        # Thêm hiển thị trạng thái ACC ở góc phải bên dưới Mode
        acc_txt = "ACC (PID): ON" if acc_enabled else "ACC (PID): OFF (EMERGENCY ONLY)"
        acc_color = self.color_green if acc_enabled else self.color_red
        cv2.putText(annotated_frame, acc_txt, (w - 450, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, acc_color, 2)
        
        # ... (Phần còn lại như cũ) ...

        if cmd == 8:
            cv2.putText(annotated_frame, 'EMERGENCY STOP', (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_red, 2)
        else:
            cv2.putText(annotated_frame, 'ALL CLEAR', (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_green, 2)

        return annotated_frame