import time
import os
import cv2
import numpy as np
import threading 
import socket    
from typing import Optional
from configs import config as C
from utils.udp import make_rx, make_tx
from utils.calib import load_undistort_map
from utils.logger import BenchmarkLogger
from utils.frame_receiver import FrameReceiver
from LaneDetection.lane_pipeline import LanePipeline
from LaneDetection.backends.yolov8_backend import YoloV8Backend
from LaneDetection.backends.pidnet_backend import PIDNetBackend
from LaneDetection.backends.twinlite_backend import TwinLiteBackend
from LaneDetection.backends.bisenetv2_backend import BiseNetV2Backend
from LaneDetection.backends.segformer_backend import SegformerBackend

cmd_member_car = 1  
STOP_CMD = 8       
acc_speed = -1 

IP_AI_MODULE = "127.0.0.1"
PORT_SEND_IMAGE = 7000
PORT_LISTEN_CMD = 7001

IP_MACHINE_2 = "192.168.5.114" 
PORT_ACC = 7002                
IP_MACHINE_3 = "192.168.5.115"
def listen_ai_command():
    global cmd_member_car, acc_speed
    sock_listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_listen.bind((IP_AI_MODULE, PORT_LISTEN_CMD))
    print(f"[Object Detect] Listening at.... {PORT_LISTEN_CMD}...")

    while True:
        try:
            data, _ = sock_listen.recvfrom(1024)
            msg = data.decode('utf-8').strip()
            
    
            if msg.startswith('S'):
                cmd_member_car = 1       
                acc_speed = int(msg[1:]) 
            else:
                cmd_member_car = int(msg)
                acc_speed = -1           
        except Exception as e:
            pass
# ==========================================================

def build_lane_pipeline() -> LanePipeline:
    m = C.LANE_MODEL.lower()
    print(f"[System] Model {m.upper()}")

    if m == "yolov8":
        backend = YoloV8Backend(
            weights=C.LANE_WEIGHTS,
            device=C.DEVICE,
            imgsz=C.IMGSZ,
            conf=C.CONF
        )

    elif m == "pidnet":
        backend = PIDNetBackend(
            weights=C.LANE_WEIGHTS,
            device=C.DEVICE,
            input_h=C.PIDNET_H,
            input_w=C.PIDNET_W,
            thr=C.PIDNET_THR,
            arch=C.PIDNET_ARCH
        )

    elif m == "twinlite":
        backend = TwinLiteBackend(
            weights=C.LANE_WEIGHTS,
            device=C.DEVICE,
            input_h=C.TWIN_H,
            input_w=C.TWIN_W,
            thr=C.TWIN_THR,
            num_classes=C.TWIN_NUM_CLASSES
        )

    elif m == "bisenet":
        backend = BiseNetV2Backend(
            weights=C.LANE_WEIGHTS,
            device=C.DEVICE,
            input_h=C.BISENET_H,
            input_w=C.BISENET_W,
            num_classes=C.BISENET_NUM_CLASSES
        )
    elif m == "segformer":
        backend = SegformerBackend(
            weights_dir=str(C.LANE_WEIGHTS), 
            device=C.DEVICE
        )

    else:
        raise ValueError(f"Unknown LANE_MODEL = {C.LANE_MODEL}")

    return LanePipeline(backend, show_overlay=C.SHOW)


def main():
    # --- A. UDP Setup ---
    print(f"[Network] Listening: {C.LISTEN_IP}:{C.LISTEN_PORT}")
    rx = make_rx(C.LISTEN_IP, C.LISTEN_PORT, C.RBUF_BYTES)
    rx.settimeout(C.NO_PACKET_TIMEOUT_S)

    tx = make_tx()
    esp_addr = (C.ESP_IP, C.ESP_PORT)
    print(f"[Network] ESP32 Target: {esp_addr}")
    
    
    tx_ai = make_tx() 
    threading.Thread(target=listen_ai_command, daemon=True).start()
    # ====================================================================

    # --- B. Build AI Pipeline + Logger ---
    lane = build_lane_pipeline()
    logger = BenchmarkLogger(C.LOG_DIR)
    if C.SHOW:
        cv2.namedWindow(C.WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(C.WIN_NAME, C.WIN_W, C.WIN_H)

    # --- C. Start receiver thread ---
    print("[System] Starting FrameReceiver...")
    frame_q = FrameReceiver(
        rx, C.END_MARKER, C.MAX_ACCUM_BYTES, None, qsize=2
    ).start()

    print("[System] Waiting for camera stream...")
    first_frame = None
    while first_frame is None:
        try:
            first_frame = frame_q.get(timeout=2.0)
        except:
            print("  ... waiting camera ...")

    H_img, W_img = first_frame.shape[:2]
    print(f"[Camera] Connected: {W_img}×{H_img}")

    if os.path.exists(str(C.CALIB_PATH)):
        undist = load_undistort_map(str(C.CALIB_PATH), W_img, H_img)
        if undist is not None:
            frame_q.undist = undist
            print("[Calib] Undistortion applied.")
    else:
        print("[Calib] No calibration file → raw mode.")
    print("    >>> AI READY — STARTING DRIVE <<<")
    fps_display = 0.0
    fps_n = 0
    t_fps0 = time.time()
    last_cmd = None

    try:
        while True:
            try:
                frame = frame_q.get(timeout=1.0)
            except:
                continue

            t0 = time.time()
         
            try:
                _, encoded_img = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                img_bytes = encoded_img.tobytes() + b'\xff\xd9' 
                tx_ai.sendto(img_bytes, (IP_AI_MODULE, PORT_SEND_IMAGE))
            except Exception as e:
                pass
        
            status = lane.step(frame, fps_display if fps_display > 0 else None)
            latency = time.time() - t0
            
            
            ema_head = status.get("ema_head", 0.0)
            try:
                tx_ai.sendto(str(ema_head).encode('utf-8'), (IP_AI_MODULE, 7005))
            except Exception:
                pass
            

            dir_u8   = status["dir"]
            speed_u8 = status["speed"]
            alpha_i8 = status["alpha"]
            overlay  = status.get("overlay", frame)

            if cmd_member_car == STOP_CMD:
                dir_u8 = STOP_CMD   
                speed_u8 = 0        
                alpha_i8 = 0
            elif acc_speed != -1:
    
                dir_u8 = 1
                speed_u8 = acc_speed

            # SEND COMMAND
            cmd = f"{dir_u8} {speed_u8} {alpha_i8}\n"

            tx.sendto(cmd.encode("ascii"), esp_addr)
            
            
            if cmd != last_cmd: 
                try:
                    tx_ai.sendto(cmd.encode("ascii"), (IP_MACHINE_2, PORT_ACC))
                   
                    tx_ai.sendto(cmd.encode("ascii"), (IP_MACHINE_3, PORT_ACC)) 
                except Exception as e:
                    pass 
                last_cmd = cmd
                
            fps_n += 1
            t_now = time.time()
            if t_now - t_fps0 >= 1.0:
                fps_display = fps_n / (t_now - t_fps0)
                fps_n = 0
                t_fps0 = t_now
                # (Trong file main.py, bên trong vòng lặp while True)
            
            logger.write_from_lane(lane.name(), status, fps_display, latency)
            
            if C.SHOW:
                # 1. Cửa sổ ảnh chính (có vạch đường xanh đỏ)
                cv2.imshow(C.WIN_NAME, overlay)
                
                # ==========================================
                # 2. CỬA SỔ ẢNH BEV (ẢNH TRẮNG ĐEN NHÌN TỪ TRÊN XUỐNG)
                # Bắt lấy bev_mask và nhân 255 để hiển thị (từ 0/1 thành 0/255)
                if "bev_mask" in status:
                    bev_image = status["bev_mask"] * 255
                    cv2.imshow("BEV Stream - Chinh SRC_RATIOS", bev_image)
                # ==========================================

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                elif key == ord("p"):
                    lane.draw_points = not lane.draw_points
            # logger.write_from_lane(lane.name(), status, fps_display, latency)
            # if C.SHOW:
            #     cv2.imshow(C.WIN_NAME, overlay)
            #     key = cv2.waitKey(1) & 0xFF
            #     if key in (27, ord("q")):
            #         break
            #     elif key == ord("p"):
            #         lane.draw_points = not lane.draw_points

    except KeyboardInterrupt:
        print("\n[System] Interrupted.")

    finally:
        print("[System] Cleanup...")
        logger.close()
        try: cv2.destroyAllWindows()
        except: pass
        try:
            tx.sendto(b"8 0 0\n", esp_addr)
            tx_ai.sendto(b"8 0 0\n", (IP_MACHINE_2, PORT_ACC)) 
            tx_ai.sendto(b"8 0 0\n", (IP_MACHINE_3, PORT_ACC))
        except:
            pass

        print("[System] Bye.")


if __name__ == "__main__":
    main()