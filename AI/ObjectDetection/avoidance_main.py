import cv2
import numpy as np
import time

# Import các module đã chia nhỏ
from VehicleDetection import VehicleDetector
from avoidance_network import AvoidanceNetwork
from avoidance_controller import AvoidanceController
from avoidance_overlay import AvoidanceOverlay

# Cấu hình chung
IMAGE_RESIZE_DIMS = (1280, 720)
MODEL_PATH = r"F:\AutoCar-LaneKeeping-main\AutoCar-LaneKeeping-main\AI\ObjectDetection\YoloWeights\Best_car.pt"

def main():
    # 1. Khởi tạo các module
    print("[System] Khởi tạo các Module hệ thống...")
    network = AvoidanceNetwork()
    network.start_threads()
    
    detector = VehicleDetector(MODEL_PATH)
    controller = AvoidanceController()
    overlay = AvoidanceOverlay()

    use_dynamic_roi = True
    print("[System] HỆ THỐNG ADAS (DYNAMIC ROI) SẴN SÀNG HOẠT ĐỘNG!")

    while True:
        # Lấy ảnh từ Network
        jpg_data = network.get_latest_image_data()
        if jpg_data is None:
            time.sleep(0.01)
            continue

        start_time = time.time()
        
        # Giải mã ảnh
        np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None: continue
        image = cv2.resize(image, IMAGE_RESIZE_DIMS)
        height, width = image.shape[:2]

        # # BƯỚC 1: Quét YOLO AI
        # detections, yolo_drawn_frame = detector.detect_and_track(image)

        # # BƯỚC 2: Tính toán Logic Vùng cấm & Khoảng cách
        # heading = network.get_heading()
        # cmd, min_dist, obj_name, poly_pts = controller.process_logic(
        #     detections, width, height, heading, use_dynamic_roi, detector
        # )

        # # # BƯỚC 3: Ra lệnh qua UDP (Chỉ gửi khi thay đổi trạng thái)
        # # if cmd != controller.prev_cmd:
        # #     network.send_command(cmd)
        # #     if cmd == controller.CMD_STOP:
        # #         print(f">> [STOP] Phanh gấp! {obj_name} cách {min_dist:.1f} cm")
        # #     else:
        # #         print(f">> [STRAIGHT] Đường trống, xe đi tiếp.")
        # #     controller.prev_cmd = cmd
        
        # # BƯỚC 3: Ra lệnh qua UDP (Chỉ gửi khi thay đổi trạng thái HOẶC khi đang điều tốc PID)
        # # Vì PWM thay đổi liên tục, ta gửi liên tục nếu đang bám đuôi
        # should_send = (cmd != controller.prev_cmd) or str(cmd).startswith('S')
        
        # if should_send:
        #     network.send_command(cmd)
            
        #     if cmd == controller.CMD_STOP:
        #         print(f">> [STOP] Dừng xe! {obj_name} cách {min_dist:.1f} cm")
        #     elif str(cmd).startswith('S'):
        #         speed_val = str(cmd)[1:]
        #         print(f">> [FOLLOW] Bám đuôi {obj_name} cách {min_dist:.1f} cm | Tốc độ: {speed_val}")
        #     else:
        #         print(f">> [STRAIGHT] Đường trống, xe đi tiếp max tốc.")
                
        #     controller.prev_cmd = cmd

        # # BƯỚC 4: Vẽ Giao diện (HUD)
        # elapsed_time = time.time() - start_time
        # fps = 1.0 / elapsed_time if elapsed_time > 0 else 99.0
        
        # final_frame = overlay.draw(
        #     yolo_drawn_frame, poly_pts, use_dynamic_roi, 
        #     heading, cmd, min_dist, fps
        # )

        # cv2.imshow('ACE SYSTEM - OBJECT AVOIDANCE', final_frame)
        # ... (Phần nhận ảnh giữ nguyên) ...

        # BƯỚC 1: Quét YOLO AI
        detections, yolo_drawn_frame = detector.detect_and_track(image)

        # BƯỚC 2: Tính toán Logic (Đã sửa giá trị trả về)
        heading = network.get_heading()
        cmd, active_objects, poly_pts, current_speed = controller.process_logic(
            detections, width, height, heading, use_dynamic_roi, detector
        )

        # BƯỚC 3: Ra lệnh qua UDP
        if cmd != controller.prev_cmd or str(cmd).startswith('S'):
            network.send_command(cmd)
            controller.prev_cmd = cmd

        # BƯỚC 4: Vẽ Giao diện (Truyền thêm active_objects và current_speed)
        elapsed_time = time.time() - start_time
        fps = 1.0 / elapsed_time if elapsed_time > 0 else 99.0
        
        final_frame = overlay.draw(
            yolo_drawn_frame, poly_pts, use_dynamic_roi, 
            heading, cmd, active_objects, current_speed, fps
        )

        cv2.imshow('ACE SYSTEM - OBJECT AVOIDANCE', final_frame)

        # Xử lý phím bấm
        # key = cv2.waitKey(1) & 0xFF
        # if key == 27: # Phím ESC
        #     break
        # elif key == ord('m') or key == ord('M'): 
        #     use_dynamic_roi = not use_dynamic_roi
        #     print(f">>> CHUYỂN CHẾ ĐỘ: {'DYNAMIC' if use_dynamic_roi else 'STATIC'} <<<")
        # ... (code hiển thị khung hình) ...
        cv2.imshow('ACE SYSTEM - OBJECT AVOIDANCE', final_frame)

        # Xử lý phím bấm
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('m'):
            use_dynamic_roi = not use_dynamic_roi
      
        elif key == ord('a'):
            controller.acc_enabled = not controller.acc_enabled

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()