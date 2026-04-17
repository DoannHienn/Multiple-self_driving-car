# # # import cv2
# # # import numpy as np

# # # class AvoidanceController:
# # #     def __init__(self, stop_threshold_cm=25.0):
# # #         self.CMD_STRAIGHT = 1
# # #         self.CMD_STOP = 8
# # #         self.stop_threshold = stop_threshold_cm
# # #         self.dangerous_obstacles = ['red_car', 'green_car', 'car']
# # #         self.prev_cmd = self.CMD_STRAIGHT

# # #     def process_logic(self, detections, width, height, dynamic_head_deg, use_dynamic_roi, detector):
# # #         # 1. TÍNH TOÁN VÙNG CẤM (ROI)
# # #         hw_bottom = int(width * 0.40) 
# # #         hw_top = int(width * 0.15)    
# # #         y_bottom = height
# # #         y_top = int(height * 0.45)    

# # #         if use_dynamic_roi:
# # #             shift_x = int(dynamic_head_deg * 15.0) 
# # #             cx_bottom_px = int(0.5 * width)        
# # #             cx_top_px = int(0.5 * width) - shift_x 
# # #         else:
# # #             cx_bottom_px = int(0.5 * width)
# # #             cx_top_px = int(0.5 * width)

# # #         poly_pts = np.array([
# # #             [cx_bottom_px - hw_bottom, y_bottom], 
# # #             [cx_bottom_px + hw_bottom, y_bottom], 
# # #             [cx_top_px + hw_top, y_top],          
# # #             [cx_top_px - hw_top, y_top]           
# # #         ], np.int32)

# # #         # 2. XỬ LÝ KHOẢNG CÁCH (IPM BEV)
# # #         cmd = self.CMD_STRAIGHT
# # #         min_distance = 9999.0
# # #         closest_obj_name = "Trống"

# # #         for det in detections:
# # #             x1, y1, x2, y2 = det['bbox']
# # #             class_label = str(det['class_label']).lower()
# # #             track_id = det['track_id']
            
# # #             if class_label not in self.dangerous_obstacles: continue
            
# # #             cx = (x1 + x2) // 2
# # #             bottom_center = (int(cx), int(y2))
            
# # #             # Lọc bằng Polygon
# # #             is_inside = cv2.pointPolygonTest(poly_pts, bottom_center, False) >= 0
# # #             if not is_inside: continue 
            
# # #             # Tính khoảng cách BEV
# # #             dist_cm = detector.calculate_distance_bev(cx, y2, width, height, track_id)
            
# # #             if dist_cm < min_distance:
# # #                 min_distance = dist_cm
# # #                 closest_obj_name = class_label

# # #         # 3. QUYẾT ĐỊNH PHANH
# # #         if min_distance <= self.stop_threshold:       
# # #             cmd = self.CMD_STOP
            
# # #         return cmd, min_distance, closest_obj_name, poly_pts
# # import cv2
# # import numpy as np
# # import time

# # class AvoidanceController:
# #     def __init__(self, target_distance_cm=10.0):
# #         self.CMD_STRAIGHT = 1
# #         self.CMD_STOP = 8
# #         self.target_distance = target_distance_cm
        
# #         # Chỉ áp dụng bám đuôi cho 'car', các vật cản khác (như người, đá) vẫn phanh gấp
# #         self.dynamic_obstacles = ['car'] 
# #         self.static_obstacles = ['red_car', 'green_car'] 
        
# #         self.prev_cmd = self.CMD_STRAIGHT

# #         # ==== THÔNG SỐ PID CHO TỐC ĐỘ ====
# #         self.Kp = 15.0  # Tăng tốc độ khi khoảng cách xa
# #         self.Ki = 0.5   # Bù đắp sai số nhỏ
# #         self.Kd = 5.0   # Hãm tốc khi khoảng cách thay đổi quá nhanh
        
# #         self.integral = 0.0
# #         self.prev_error = 0.0
# #         self.prev_time = time.time()
        
# #         # Giới hạn tốc độ PWM (0-255)
# #         self.max_speed = 150 
# #         self.min_speed = 80  # Tốc độ thấp nhất để xe bắt đầu lăn bánh

# #     def process_logic(self, detections, width, height, dynamic_head_deg, use_dynamic_roi, detector):
# #         # 1. TÍNH TOÁN VÙNG CẤM (ROI) - GIỮ NGUYÊN
# #         hw_bottom = int(width * 0.40) 
# #         hw_top = int(width * 0.15)    
# #         y_bottom = height
# #         y_top = int(height * 0.45)    

# #         if use_dynamic_roi:
# #             shift_x = int(dynamic_head_deg * 15.0) 
# #             cx_bottom_px = int(0.5 * width)        
# #             cx_top_px = int(0.5 * width) - shift_x 
# #         else:
# #             cx_bottom_px = int(0.5 * width)
# #             cx_top_px = int(0.5 * width)

# #         poly_pts = np.array([
# #             [cx_bottom_px - hw_bottom, y_bottom], 
# #             [cx_bottom_px + hw_bottom, y_bottom], 
# #             [cx_top_px + hw_top, y_top],          
# #             [cx_top_px - hw_top, y_top]           
# #         ], np.int32)

# #         # 2. TÌM VẬT CẢN GẦN NHẤT
# #         min_distance = 9999.0
# #         closest_obj_name = "Trống"
# #         is_dynamic_target = False

# #         for det in detections:
# #             x1, y1, x2, y2 = det['bbox']
# #             class_label = str(det['class_label']).lower()
# #             track_id = det['track_id']
            
# #             if class_label not in self.dynamic_obstacles and class_label not in self.static_obstacles: 
# #                 continue
            
# #             cx = (x1 + x2) // 2
# #             bottom_center = (int(cx), int(y2))
            
# #             is_inside = cv2.pointPolygonTest(poly_pts, bottom_center, False) >= 0
# #             if not is_inside: continue 
            
# #             dist_cm = detector.calculate_distance_bev(cx, y2, width, height, track_id)
            
# #             if dist_cm < min_distance:
# #                 min_distance = dist_cm
# #                 closest_obj_name = class_label
# #                 is_dynamic_target = (class_label in self.dynamic_obstacles)

# #         # 3. QUYẾT ĐỊNH LỆNH ĐIỀU KHIỂN
# #         cmd = self.CMD_STRAIGHT # Mặc định đi thẳng nhanh nhất

# #         current_time = time.time()
# #         dt = current_time - self.prev_time
# #         if dt <= 0: dt = 0.001 # Tránh chia cho 0
# #         self.prev_time = current_time

# #         if min_distance != 9999.0:
# #             if is_dynamic_target:
# #                 # ==========================================
# #                 # CHẾ ĐỘ BÁM ĐUÔI XE TRƯỚC (CAR) BẰNG PID NÂNG CẤP
# #                 # ==========================================
                
# #                 # 1. Tinh chỉnh lại hệ số PID (Bạn có thể sửa lại trên hàm __init__)
# #                 # Tăng mạnh Kd để hãm phanh từ xa, giảm Kp để bớt hung hăng
# #                 self.Kp = 8.0   
# #                 self.Ki = 0.2   
# #                 self.Kd = 25.0  # TĂNG MẠNH: Đạo hàm (Tốc độ đóng nắp khoảng cách)
                
# #                 raw_error = min_distance - self.target_distance
                
# #                 # KỸ THUẬT 2: Giới hạn sự "thèm khát" tốc độ
# #                 # Dù xe kia cách xa 1 mét, chỉ cho phép PID tính toán dựa trên sai số tối đa 20cm
# #                 error = np.clip(raw_error, -20, 20)
                
# #                 # Tính tích phân (đã mở khóa giới hạn như giải pháp lần trước)
# #                 self.integral += error * dt
# #                 self.integral = np.clip(self.integral, -400, 400) 
                
# #                 # Tính đạo hàm (Vận tốc tiệm cận)
# #                 derivative = (error - self.prev_error) / dt
# #                 self.prev_error = error
                
# #                 # Tổng tín hiệu PID (Có cộng ga nền min_speed)
# #                 pid_output = self.min_speed + (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
                
# #                 # KỸ THUẬT 3: Vùng hãm tốc độ tự động (Braking Zones)
# #                 dynamic_max_speed = self.max_speed
# #                 if min_distance < 30.0:
# #                     dynamic_max_speed = 120  # Dưới 30cm: Cấm chạy quá mức 120
# #                 if min_distance < 20.0:
# #                     dynamic_max_speed = 90   # Dưới 20cm: Cấm chạy quá mức 90 (Chế độ rà phanh)

# #                 if pid_output <= 0:
# #                     cmd = self.CMD_STOP 
# #                 else:
# #                     # Chặn tốc độ xuất ra không bao giờ vượt qua dynamic_max_speed
# #                     calculated_speed = int(np.clip(pid_output, self.min_speed, dynamic_max_speed))
# #                     cmd = f"S{calculated_speed}"
# #             else:
# #                 # ==========================================
# #                 # CHẾ ĐỘ PHANH GẤP VỚI VẬT TĨNH (RED_CAR, GREEN_CAR)
# #                 # ==========================================
# #                 if min_distance <= 10.0: # Khoảng cách an toàn tĩnh
# #                     cmd = self.CMD_STOP
# #                 self.prev_error = 0.0
# #                 self.integral = 0.0
# #         else:
# #             # Đường trống, reset PID
# #             self.prev_error = 0.0
# #             self.integral = 0.0

# #         return cmd, min_distance, closest_obj_name, poly_pts
# import cv2
# import numpy as np
# import time

# class AvoidanceController:
#     def __init__(self, target_distance_cm=12.5):
#         self.CMD_STRAIGHT = 1
#         self.CMD_STOP = 8
#         self.target_distance = target_distance_cm
#         self.dangerous_obstacles = ['red_car', 'green_car', 'car']
#         self.dynamic_obstacles = ['car']
        
#         # PID Parameters
#         self.Kp, self.Ki, self.Kd = 8.0, 0.2, 25.0
#         self.integral = 0.0
#         self.prev_error = 0.0
#         self.prev_time = time.time()
#         self.prev_raw_dist = 100.0
#         self.deriv_sm = 0.0
        
#         self.max_speed = 180 
#         self.min_speed = 60
#         self.speed_out_sm = float(self.min_speed)
#         self.prev_cmd = self.CMD_STRAIGHT

#     def process_logic(self, detections, width, height, dynamic_head_deg, use_dynamic_roi, detector):
#         # 1. Tính toán Dynamic ROI (Giữ nguyên)
#         hw_bottom, hw_top = int(width * 0.40), int(width * 0.15)
#         y_bottom, y_top = height, int(height * 0.45)
        
#         cx_mid = int(0.5 * width)
#         cx_bottom_px = cx_mid
#         cx_top_px = cx_mid - int(dynamic_head_deg * 15.0) if use_dynamic_roi else cx_mid

#         poly_pts = np.array([
#             [cx_bottom_px - hw_bottom, y_bottom], [cx_bottom_px + hw_bottom, y_bottom], 
#             [cx_top_px + hw_top, y_top], [cx_top_px - hw_top, y_top]
#         ], np.int32)

#         # 2. Xử lý đa mục tiêu
#         active_objects = [] # Danh sách vật thể trong ROI
#         min_distance = 9999.0
#         is_dynamic_target = False

#         for det in detections:
#             x1, y1, x2, y2 = det['bbox']
#             label = str(det['class_label']).lower()
#             if label not in self.dangerous_obstacles: continue
            
#             cx, cy_bottom = (x1 + x2) // 2, y2
#             if cv2.pointPolygonTest(poly_pts, (cx, cy_bottom), False) >= 0:
#                 dist = detector.calculate_distance_bev(cx, cy_bottom, width, height, det['track_id'])
                
#                 # Lưu thông tin để vẽ HUD
#                 active_objects.append({'label': label, 'dist': dist, 'bbox': [x1, y1, x2, y2]})
                
#                 if dist < min_distance:
#                     min_distance = dist
#                     is_dynamic_target = (label in self.dynamic_obstacles)

#         # 3. Tính toán PID & Quyết định (Giữ nguyên logic mượt mà)
#         cmd = self.CMD_STRAIGHT
#         dt = max(0.001, time.time() - self.prev_time)
#         self.prev_time = time.time()

#         if min_distance != 9999.0:
#             if is_dynamic_target:
#                 error = np.clip(min_distance - self.target_distance, -20, 60)
#                 self.integral = np.clip(self.integral + error * dt, -400, 400)
#                 raw_deriv = (min_distance - self.prev_raw_dist) / dt
#                 self.deriv_sm = 0.7 * self.deriv_sm + 0.3 * raw_deriv
#                 self.prev_raw_dist = min_distance
                
#                 pid_out = self.min_speed + (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * self.deriv_sm)
                
#                 if pid_out <= 0:
#                     cmd, self.speed_out_sm = self.CMD_STOP, float(self.min_speed)
#                 else:
#                     limit = 90 if min_distance < 25 else (130 if min_distance < 45 else self.max_speed)
#                     target_s = np.clip(pid_out, self.min_speed, limit)
#                     # Slew rate limiter
#                     step = 3.5 if target_s > self.speed_out_sm else -15.0
#                     self.speed_out_sm = np.clip(self.speed_out_sm + step, self.min_speed, target_s) if target_s > self.speed_out_sm else max(target_s, self.speed_out_sm + step)
#                     cmd = f"S{int(self.speed_out_sm)}"
#             else:
#                 if min_distance <= 20.0: cmd = self.CMD_STOP
#                 self.speed_out_sm = float(self.min_speed)
#         else:
#             self.speed_out_sm = -1 # Trạng thái không bám đuôi

#         return cmd, active_objects, poly_pts, self.speed_out_sm
import cv2
import numpy as np
import time

class AvoidanceController:
    # Đặt target mặc định là 12.5cm để kết hợp với Deadband 2.5cm tạo ra khoảng an toàn 10-15cm
    def __init__(self, target_distance_cm=12.5):
        self.CMD_STRAIGHT = 1
        self.CMD_STOP = 8
        self.target_distance = target_distance_cm
        self.dangerous_obstacles = ['red_car', 'green_car', 'car']
        self.dynamic_obstacles = ['car']
        
        # PID Parameters (Đã được tinh chỉnh để chạy êm ái)
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

    def process_logic(self, detections, width, height, dynamic_head_deg, use_dynamic_roi, detector):
        # 1. TÍNH TOÁN VÙNG CẤM (DYNAMIC ROI)
        hw_bottom, hw_top = int(width * 0.40), int(width * 0.15)
        y_bottom, y_top = height, int(height * 0.45)
        
        cx_mid = int(0.5 * width)
        cx_bottom_px = cx_mid
        cx_top_px = cx_mid - int(dynamic_head_deg * 15.0) if use_dynamic_roi else cx_mid

        poly_pts = np.array([
            [cx_bottom_px - hw_bottom, y_bottom], [cx_bottom_px + hw_bottom, y_bottom], 
            [cx_top_px + hw_top, y_top], [cx_top_px - hw_top, y_top]
        ], np.int32)

        # # 2. XỬ LÝ ĐA MỤC TIÊU VÀ TÌM VẬT GẦN NHẤT
        # active_objects = [] # Chứa danh sách các vật để vẽ HUD
        # min_distance = 9999.0
        # is_dynamic_target = False

        # for det in detections:
        #     x1, y1, x2, y2 = det['bbox']
        #     label = str(det['class_label']).lower()
        #     if label not in self.dangerous_obstacles: continue
            
        #     cx, cy_bottom = (x1 + x2) // 2, y2
            
        #     # Kiểm tra xem vật thể có nằm trong vùng hình thang không
        #     is_inside_roi = cv2.pointPolygonTest(poly_pts, (cx, cy_bottom), False) >= 0
            
        #     # ==========================================
        #     # LÔ-GIC ĐẶC QUYỀN:
        #     # - Nếu là 'car' -> Chấp nhận tính toán luôn, bỏ qua ROI!
        #     # - Nếu là vật khác -> Bắt buộc phải is_inside_roi == True
        #     # ==========================================
        #     if label in self.dynamic_obstacles or is_inside_roi:
        #         dist = detector.calculate_distance_bev(cx, cy_bottom, width, height, det['track_id'])
                
        #         # Lưu thông tin vật thể để gửi sang file vẽ Giao diện (Overlay)
        #         active_objects.append({'label': label, 'dist': dist, 'bbox': [x1, y1, x2, y2]})
                
        #         if dist < min_distance:
        #             min_distance = dist
        #             is_dynamic_target = (label in self.dynamic_obstacles)
            
        #     # Kiểm tra xem có nằm trong vùng an toàn (ROI) không
        #     if cv2.pointPolygonTest(poly_pts, (cx, cy_bottom), False) >= 0:
        #         dist = detector.calculate_distance_bev(cx, cy_bottom, width, height, det['track_id'])
                
        #         # Lưu thông tin vật thể để gửi sang file vẽ Giao diện (Overlay)
        #         active_objects.append({'label': label, 'dist': dist, 'bbox': [x1, y1, x2, y2]})
                
        #         if dist < min_distance:
        #             min_distance = dist
        #             is_dynamic_target = (label in self.dynamic_obstacles)
        # 2. XỬ LÝ ĐA MỤC TIÊU VÀ TÌM VẬT GẦN NHẤT
        active_objects = [] # Chứa danh sách các vật để vẽ HUD
        min_distance = 9999.0
        is_dynamic_target = False

        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            label = str(det['class_label']).lower()
            if label not in self.dangerous_obstacles: continue
            
            # ==========================================
            # KIỂM TRA 3 ĐIỂM ĐÁY XE (Dành cho vật tĩnh)
            # ==========================================
            cy_bottom = y2
            cx = (x1 + x2) // 2
            
            pt_center = (cx, cy_bottom)
            pt_left = (x1, cy_bottom)   
            pt_right = (x2, cy_bottom)  
            
            in_center = cv2.pointPolygonTest(poly_pts, pt_center, False) >= 0
            in_left = cv2.pointPolygonTest(poly_pts, pt_left, False) >= 0
            in_right = cv2.pointPolygonTest(poly_pts, pt_right, False) >= 0
            
            is_inside_roi = in_center or in_left or in_right
            
            # ==========================================
            # LÔ-GIC ĐẶC QUYỀN VVIP:
            # - Nếu là 'car': BỎ QUA ROI! Nhìn thấy ở đâu cũng đo khoảng cách bám theo!
            # - Nếu là vật khác: Bắt buộc 1 trong 3 điểm phải chạm ROI mới xử lý.
            # ==========================================
            if label in self.dynamic_obstacles or is_inside_roi:
                dist = detector.calculate_distance_bev(cx, cy_bottom, width, height, det['track_id'])
                
                # Lưu thông tin vật thể để vẽ HUD
                active_objects.append({'label': label, 'dist': dist, 'bbox': [x1, y1, x2, y2]})
                
                if dist < min_distance:
                    min_distance = dist
                    is_dynamic_target = (label in self.dynamic_obstacles)

        # 3. TÍNH TOÁN PID VÀ QUYẾT ĐỊNH
        cmd = self.CMD_STRAIGHT
        dt = max(0.001, time.time() - self.prev_time)
        self.prev_time = time.time()

        if min_distance != 9999.0:
            if is_dynamic_target:
                # ==========================================
                # KỸ THUẬT DEADBAND (VÙNG DUNG SAI 10cm - 15cm)
                # ==========================================
                raw_error = min_distance - self.target_distance
                tolerance = 2.5 # Dung sai ±2.5cm
                
                if abs(raw_error) <= tolerance:
                    raw_error = 0.0 # Đánh lừa PID là đã đạt mục tiêu để xe giữ đều ga
                # ==========================================
                
                # Bóp băng thông sai số để xe không thèm khát tốc độ khi ở quá xa
                error_p = np.clip(raw_error, -20, 60)
                
                # Khâu Tích phân (I) - Đã mở rộng giới hạn để bù ga
                self.integral = np.clip(self.integral + error_p * dt, -400, 400)
                
                # Khâu Đạo hàm (D) có bộ lọc chống sốc (EMA Filter)
                raw_deriv = (min_distance - self.prev_raw_dist) / dt
                self.deriv_sm = 0.7 * self.deriv_sm + 0.3 * raw_deriv
                self.prev_raw_dist = min_distance
                
                # Tính tổng tín hiệu
                pid_out = self.min_speed + (self.Kp * error_p) + (self.Ki * self.integral) + (self.Kd * self.deriv_sm)
                
                # Xử lý lệnh đầu ra
                if pid_out <= 0:
                    cmd, self.speed_out_sm = self.CMD_STOP, float(self.min_speed)
                else:
                    # Vùng hãm tốc độ theo cự ly
                    limit = 90 if min_distance < 25 else (95 if min_distance < 45 else self.max_speed)
                    target_s = np.clip(pid_out, self.min_speed, limit)
                    
                    # Bộ giảm xóc chân ga (Tăng từ từ, phanh dứt khoát)
                    step = 3.5 if target_s > self.speed_out_sm else -6.0
                    if target_s > self.speed_out_sm:
                        self.speed_out_sm = np.clip(self.speed_out_sm + step, self.min_speed, target_s)
                    else:
                        self.speed_out_sm = max(target_s, self.speed_out_sm + step)
                        
                    cmd = f"S{int(self.speed_out_sm)}"
            else:
                # Phanh gấp với vật tĩnh
                if min_distance <= 10.0: 
                    cmd = self.CMD_STOP
                self.speed_out_sm = float(self.min_speed)
        else:
            # Đường trống, tắt chế độ bám đuôi
            self.speed_out_sm = -1 

        return cmd, active_objects, poly_pts, self.speed_out_sm