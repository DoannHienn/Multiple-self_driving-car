import numpy as np

class LaneController:
    def __init__(self):
        # ==== DEADBAND & THRESHOLDS ====
        self.theta_thresh       = 2.1
        self.pos_deadband_m     = 0.01
        self.head_deadband_deg  = 0.5
        
        # ==== SERVO LIMITS & PROTECTION ====
        self.alpha_limit        = 54
        self.alpha_limit_left   = 42 
        self.alpha_limit_right  = 53  
        self.alpha_filter       = 0.30
        self.alpha_slew         = 12.0
        self.min_abs_alpha      = 4.0
         
        # ==== GAIN SCHEDULING ====
        self.left_boost         = 1.45
        self.right_boost        = 1.40
    
        # ==== PID PARAMETERS ====
        self.pos_i_gain         = 2.0
        self.pos_i_forget       = 0.92
        self.pos_i_clip_deg     = 3.0
        self.pos_d_gain_R       = 60.0 
        self.pos_d_forget       = 0.22  
        
        # ==== STEERING TRIM ====
        self.steer_trim_deg     = -0.3
        self.trim_beta          = 0.0025
        self.trim_sm            = float(self.steer_trim_deg)
        
        # ==== SPEED CONTROL ====
        self.base_speed         = 78
        
        # ==== INTERNAL STATE ====
        self.alpha_sm           = None
        self._alpha_out_prev    = 0.0
        self._prev_head         = 0.0
        self._curveL_cnt        = 0
        self._curveR_cnt        = 0
        self._pos_prev          = None
        self._pos_d_ema         = 0.0
        self.pos_i_state        = 0.0
        self.fps_val            = 20.0
        self.speed_sm           = float(self.base_speed)
        
        # ==== INPUT EMA ====
        self.ema_pos            = None
        self.ema_head           = None

    def set_fps(self, fps: float):
        self.fps_val = max(8.0, float(fps) if fps and fps > 0 else 20.0)

    def update_ema(self, pos_m: float, head_deg: float):
        self.ema_pos  = float(pos_m)
        self.ema_head = float(head_deg)

    @staticmethod
    def _soft_db(x: float, db: float) -> float:
        if abs(x) <= db: return 0.0
        return np.sign(x) * (abs(x) - db)

    def decide(self, lane_ok: int):
        
        # ---------------------------------------------------------
        # 1. SIGNAL CONDITIONING (Deadband - Đã bỏ Bias gây trễ)
        # ---------------------------------------------------------
        ema_pos  = float(self.ema_pos  if self.ema_pos  is not None else 0.0)
        ema_head = float(self.ema_head if self.ema_head is not None else 0.0)
        
        # BỎ HOÀN TOÀN CÁC DÒNG BIAS NÀY:
        # bias_mag_L = 0.013
        # bias_mag_R = 0.01
        # bias = bias_mag_L if ema_head > 0 else (bias_mag_R if ema_head < 0 else 0.0)
        # pos_bias = np.sign(-ema_head) * abs(bias)

        # Tính toán sai số trực tiếp qua Deadband (rất nhạy bén)
        pos_err  = self._soft_db(ema_pos, self.pos_deadband_m)
        head_err = self._soft_db(ema_head, self.head_deadband_deg)
        
        d_head   = head_err - self._prev_head
        self._prev_head = head_err

        # ---------------------------------------------------------
        # 2. STATE ESTIMATION (Curve Persistence Counter)
        # ---------------------------------------------------------
        if head_err > +self.theta_thresh:
            self._curveL_cnt = min(self._curveL_cnt + 1, 12); self._curveR_cnt = max(self._curveR_cnt - 1, 0)
        elif head_err < -self.theta_thresh:
            self._curveR_cnt = min(self._curveR_cnt + 1, 12); self._curveL_cnt = max(self._curveL_cnt - 1, 0)
        else:
            self._curveL_cnt = max(self._curveL_cnt - 1, 0); self._curveR_cnt = max(self._curveR_cnt - 1, 0)

        # ---------------------------------------------------------
        # 3. AUTO-TRIM LEARNING
        # ---------------------------------------------------------
        near_straight = abs(head_err) < 1.2
        if near_straight and lane_ok:
            self.trim_sm = (1.0 - self.trim_beta)*self.trim_sm + self.trim_beta * np.clip(ema_pos*120.0, -2.0, 2.0)
        else:
            self.trim_sm = 0.999*self.trim_sm + 0.001*self.steer_trim_deg
            
        # ---------------------------------------------------------
        # 4. PHYSICS-BASED DERIVATIVE (FPS Normalized)
        # ---------------------------------------------------------
        if self._pos_prev is None:
            self._pos_prev = ema_pos
        d_pos_inst  = float(ema_pos - self._pos_prev)
        self._pos_prev = ema_pos
        d_pos_per_s = d_pos_inst * self.fps_val
        self._pos_d_ema = (1.0 - self.pos_d_forget)*self._pos_d_ema + self.pos_d_forget*d_pos_per_s

        # ---------------------------------------------------------
        # 5. GAIN SCHEDULING (Cân bằng: Quyết đoán vào cua, êm ái đường thẳng)
        # ---------------------------------------------------------
        if head_err < -self.theta_thresh:  # RIGHT TURN
            K_head = 0.9 * self.right_boost 
            K_pos  = 7.7 * self.right_boost  # Đủ lớn để bám cua
            K_d    = 0.35 
            limit  = self.alpha_limit_right
            local_filter = 0.45 # Trả lại tốc độ phản ứng cho servo
            local_slew   = 22.0 
            
        elif head_err > +self.theta_thresh: # LEFT TURN
            K_head = 0.9 * self.left_boost
            K_pos  = 7.7 * self.left_boost
            K_d    = 0.35
            limit  = self.alpha_limit_left
            local_filter = 0.35 #0.35
            local_slew   = 22.0 #15.0
            
        else: # STRAIGHT (Đi thẳng)
            K_head, K_pos, K_d = 0.7, 3, 0.65 
            limit        = self.alpha_limit
            local_filter = 0.25 
            local_slew   = 17.0 # Cho phép vô lăng vẫy nhanh để sửa sai kịp thời

        # ---------------------------------------------------------
        # 6. CONDITIONAL INTEGRAL ACTION (Giữ nguyên)
        # ---------------------------------------------------------
        if near_straight and lane_ok:
            self.pos_i_state = self.pos_i_forget*self.pos_i_state + (1.0 - self.pos_i_forget)*pos_err
        else:
            self.pos_i_state *= 0.98
        u_i = np.clip(self.pos_i_state * self.pos_i_gain, -self.pos_i_clip_deg, self.pos_i_clip_deg)
        
        # ---------------------------------------------------------
        # 7. CONTROL LAW (PID TỐI ƯU CÓ TÍNH TOÁN DẠT NGANG)
        # ---------------------------------------------------------
        # K_d bây giờ sẽ nhân với vận tốc dạt ngang (_pos_d_ema) kết hợp 1 chút góc xoay (d_head)
        # Điều này giúp xe có sức ì, chống lại các pha lạng lách vô cớ
        d_term = (K_d * d_head) + (K_pos * 0.15 * self._pos_d_ema)
        
        u = K_head*head_err + K_pos*pos_err + d_term + u_i
        
        u_shaped = u 
        
        # ... (Phần bên dưới giữ nguyên code của bạn)
        if head_err < -self.theta_thresh:  
            u_shaped *= 0.90
            
        alpha_cmd = u_shaped + self.trim_sm
        
        if 1e-3 < abs(alpha_cmd) < self.min_abs_alpha:
            alpha_cmd = np.sign(alpha_cmd) * self.min_abs_alpha
            
        # Giới hạn góc lái an toàn
        if alpha_cmd < 0:
            alpha_cmd = max(alpha_cmd, -limit)
        else:
            alpha_cmd = min(alpha_cmd, +limit)

        sat_right = False
        if head_err < -self.theta_thresh and alpha_cmd > 0:
            if self._curveR_cnt >= 8:
                alpha_cmd = min(alpha_cmd, 0.92*limit); sat_right = True
            elif self._curveR_cnt >= 5 and abs(self._alpha_out_prev) > 0.85*limit:
                alpha_cmd = min(alpha_cmd, 0.96*limit); sat_right = True
        if sat_right:
            self.pos_i_state *= 0.85
            
        if (head_err < -self.theta_thresh) and (pos_err < -0.030):
            alpha_cmd = max(alpha_cmd, -0.85 * limit)

        # ---------------------------------------------------------
        # 8. OUTPUT SMOOTHING & SLEW RATE LIMITING
        # ---------------------------------------------------------
        if self.alpha_sm is None:
            self.alpha_sm = alpha_cmd
        else:
            self.alpha_sm = (1 - local_filter)*self.alpha_sm + local_filter*alpha_cmd

        deg_per_s_cap = 300.0 
        local_slew = min(local_slew, deg_per_s_cap / max(8.0, self.fps_val))
        step = np.clip(self.alpha_sm - self._alpha_out_prev, -local_slew, local_slew)
        alpha_out = self._alpha_out_prev + step
        self._alpha_out_prev = alpha_out

        # ---------------------------------------------------------
        # 9. TỐC ĐỘ BIẾN THIÊN TRƠN (SMOOTH SPEED)
        # ---------------------------------------------------------
        curve_cnt = max(self._curveL_cnt, self._curveR_cnt)
        
        # Trừ tốc độ theo hàm tuyến tính thay vì lệnh if/else giật cục
        speed_drop = 1.2 * abs(head_err) + 0.5 * max(0, curve_cnt - 4)
        target_speed = self.base_speed - speed_drop
            
        if not lane_ok:
            target_speed = min(target_speed, 110)
            
        target_speed = max(60, min(185, target_speed))

        # Bộ lọc EMA Tốc độ
        if not hasattr(self, 'speed_sm'):
            self.speed_sm = float(self.base_speed)
            
        if target_speed < self.speed_sm:
            self.speed_sm = 0.73 * self.speed_sm + 0.25 * target_speed 
        else:
            self.speed_sm = 0.92 * self.speed_sm + 0.08 * target_speed 

        return 1, int(round(self.speed_sm)), int(np.clip(round(alpha_out), -128, 127))