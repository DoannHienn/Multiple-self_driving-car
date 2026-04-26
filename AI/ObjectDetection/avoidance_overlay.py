import cv2

class AvoidanceOverlay:
    def __init__(self):
        self.color_blue = (255, 0, 0)
        self.color_red = (0, 0, 255)
        self.color_green = (0, 255, 0)
        self.color_yellow = (0, 255, 255)

    def draw(self, frame, poly_pts, use_dynamic_roi, heading, cmd, active_objects, speed, fps, acc_enabled=True): 
        annotated_frame = frame.copy()
        h, w = annotated_frame.shape[:2]

        roi_color = self.color_blue if use_dynamic_roi else self.color_red
        cv2.polylines(annotated_frame, [poly_pts], True, roi_color, 2)
    
        for obj in active_objects:
            x1, y1, x2, y2 = obj['bbox']
            dist_txt = f"{obj['dist']:.1f}cm"
            cv2.putText(annotated_frame, dist_txt, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3)
            cv2.putText(annotated_frame, dist_txt, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.color_yellow, 2)

        speed_txt = f"SPEED: {int(speed)} PWM" if speed > 0 else "SPEED: LANE CTRL"
        cv2.putText(annotated_frame, speed_txt, (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_yellow, 2)
        
        mode_txt = f"MODE: DYNAMIC (Angle: {heading:+.1f})" if use_dynamic_roi else "MODE: STATIC (FIXED)"
        cv2.putText(annotated_frame, mode_txt, (w - 450, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, roi_color, 2)
        cv2.putText(annotated_frame, f'FPS: {int(fps)}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_green, 2)
        

       
        acc_txt = "ACC (PID): ON" if acc_enabled else "ACC (PID): OFF (EMERGENCY ONLY)"
        acc_color = self.color_green if acc_enabled else self.color_red
        cv2.putText(annotated_frame, acc_txt, (w - 450, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, acc_color, 2)
        

        if cmd == 8:
            cv2.putText(annotated_frame, 'EMERGENCY STOP', (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_red, 2)
        else:
            cv2.putText(annotated_frame, 'ALL CLEAR', (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_green, 2)

        return annotated_frame