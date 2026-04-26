import cv2
import numpy as np
import time
from collections import deque, Counter  
class AvoidanceController:
    def __init__(self, target_distance_cm=13.5):
        self.CMD_STRAIGHT = 1
        self.CMD_STOP = 8
        self.target_distance = target_distance_cm
        self.dangerous_obstacles = ['red_car', 'green_car', 'car']
        self.dynamic_obstacles = ['car']
        
        # PID Parameters
        self.Kp = 8.0
        self.Ki = 0.2
        self.Kd = 25.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
        self.prev_raw_dist = 100.0
        self.deriv_sm = 0.0
        
        # Giới hạn tốc độ PWM
        self.max_speed = 95 
        self.min_speed = 80
        self.speed_out_sm = float(self.min_speed)
        self.prev_cmd = self.CMD_STRAIGHT
        self.acc_enabled = True 
        ####
        self.track_label_history = {}
        self.HISTORY_LENGTH = 5

    def process_logic(self, detections, width, height, dynamic_head_deg, use_dynamic_roi, detector):
        hw_bottom, hw_top = int(width * 0.40), int(width * 0.13)
        
        y_bottom, y_top = height, int(height * 0.45)
        
        cx_mid = int(0.5 * width)
        cx_bottom_px = cx_mid
        cx_top_px = cx_mid - int(dynamic_head_deg * 15.0) if use_dynamic_roi else cx_mid

        poly_pts = np.array([
            [cx_bottom_px - hw_bottom, y_bottom], [cx_bottom_px + hw_bottom, y_bottom], 
            [cx_top_px + hw_top, y_top], [cx_top_px - hw_top, y_top]
        ], np.int32)

        active_objects = [] 
        min_distance = 9999.0
        is_dynamic_target = False

        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            #Them de hieu chuan 
            print(f"Tọa độ y2 của xe trước: {y2}")
            raw_label = str(det['class_label']).lower()
            track_id = det['track_id']

            if track_id not in self.track_label_history:
                self.track_label_history[track_id] = deque(maxlen=self.HISTORY_LENGTH)
                
            self.track_label_history[track_id].append(raw_label)
            
            label_counts = Counter(self.track_label_history[track_id])
            stable_label = label_counts.most_common(1)[0][0]

            if stable_label not in self.dangerous_obstacles: continue
            
            cy_bottom = y2
            cx = (x1 + x2) // 2
            
            pt_center = (cx, cy_bottom)
            pt_left = (x1, cy_bottom)   
            pt_right = (x2, cy_bottom)  
            
            in_center = cv2.pointPolygonTest(poly_pts, pt_center, False) >= 0
            in_left = cv2.pointPolygonTest(poly_pts, pt_left, False) >= 0
            in_right = cv2.pointPolygonTest(poly_pts, pt_right, False) >= 0
            
            is_inside_roi = in_center or in_left or in_right
            
            if stable_label in self.dynamic_obstacles or is_inside_roi:
                dist = detector.calculate_distance_bev(cx, cy_bottom, width, height, track_id)
                
                active_objects.append({'label': stable_label, 'dist': dist, 'bbox': [x1, y1, x2, y2]})
                
                if dist < min_distance:
                    min_distance = dist
                    is_dynamic_target = (stable_label in self.dynamic_obstacles)

        cmd = self.CMD_STRAIGHT
        dt = max(0.001, time.time() - self.prev_time)
        self.prev_time = time.time()
        
        self.Kp = 2.5   
        self.Ki = 0.05  
        self.Kd = 12.0   

        if min_distance != 9999.0:
            if is_dynamic_target and self.acc_enabled:
                
                raw_error = min_distance - self.target_distance
                tolerance = 2.5 
                
                if abs(raw_error) <= tolerance:
                    raw_error = 0.0 
                
                error_p = np.clip(raw_error, -20, 60)
                
                self.integral = np.clip(self.integral + error_p * dt, -400, 400)
                
                raw_deriv = (min_distance - self.prev_raw_dist) / dt
                self.deriv_sm = 0.7 * self.deriv_sm + 0.3 * raw_deriv
                self.prev_raw_dist = min_distance
                
                pid_out = self.min_speed + (self.Kp * error_p) + (self.Ki * self.integral) + (self.Kd * self.deriv_sm)
                
                if min_distance < 10.0:
                    cmd, self.speed_out_sm = self.CMD_STOP, 0.0 
                else:
                    limit = 90 if min_distance < 25 else (95 if min_distance < 45 else self.max_speed)
                    
                    target_s = np.clip(pid_out, 0.0, limit)
                    
                    step = 1.5 if target_s > self.speed_out_sm else -3.0
                    
                    if target_s > self.speed_out_sm:
                        if self.speed_out_sm <= 0.0 and target_s > 0:
                            self.speed_out_sm = 90.0 
                        else:
                            self.speed_out_sm = min(self.speed_out_sm + step, target_s)
                    else:
                        self.speed_out_sm = max(target_s, self.speed_out_sm + step)
                        
                    if self.speed_out_sm <= 0:
                        cmd, self.speed_out_sm = self.CMD_STOP, 0.0
                    else:
                        cmd = f"S{int(self.speed_out_sm)}"
            else:
                if min_distance <= 15.0: 
                    cmd = self.CMD_STOP
                self.speed_out_sm = float(self.min_speed)
        else:
            self.speed_out_sm = -1 

        return cmd, active_objects, poly_pts, self.speed_out_sm