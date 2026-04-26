import cv2
import numpy as np
import time
from VehicleDetection import VehicleDetector
from avoidance_network import AvoidanceNetwork
from avoidance_controller import AvoidanceController
from avoidance_overlay import AvoidanceOverlay

# Cấu hình chung
IMAGE_RESIZE_DIMS = (1280, 720)
MODEL_PATH = r"F:\AutoCar-LaneKeeping-main\AutoCar-LaneKeeping-main\AI\ObjectDetection\YoloWeights\best.pt"

def main():
    # 1. Khởi tạo các module
    print("[System] Init the module.....")
    network = AvoidanceNetwork()
    network.start_threads()
    
    detector = VehicleDetector(MODEL_PATH)
    controller = AvoidanceController()
    overlay = AvoidanceOverlay()

    use_dynamic_roi = True
    print("[System] Ready to running !")

    while True:
        jpg_data = network.get_latest_image_data()
        if jpg_data is None:
            time.sleep(0.01)
            continue

        start_time = time.time()
        
        np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None: continue
        image = cv2.resize(image, IMAGE_RESIZE_DIMS)
        height, width = image.shape[:2]

        detections, yolo_drawn_frame = detector.detect_and_track(image)


        heading = network.get_heading()
        cmd, active_objects, poly_pts, current_speed = controller.process_logic(
            detections, width, height, heading, use_dynamic_roi, detector
        )

        if cmd != controller.prev_cmd or str(cmd).startswith('S'):
            network.send_command(cmd)
            controller.prev_cmd = cmd


        elapsed_time = time.time() - start_time
        fps = 1.0 / elapsed_time if elapsed_time > 0 else 99.0
        
        final_frame = overlay.draw(
            yolo_drawn_frame, poly_pts, use_dynamic_roi, 
            heading, cmd, active_objects, current_speed, fps,
            controller.acc_enabled  
        )

        cv2.imshow('ACE SYSTEM - OBJECT AVOIDANCE', final_frame)
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