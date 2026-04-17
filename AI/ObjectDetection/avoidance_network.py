import socket
import threading
from queue import Queue

class AvoidanceNetwork:
    def __init__(self, ip="127.0.0.1", port_img=7000, port_poly=7005, port_send=7001):
        self.ip_send = ip
        self.port_send = port_send
        
        # Cấu hình Socket
        self.sock_img = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_img.bind((ip, port_img))
        self.sock_poly = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_poly.bind((ip, port_poly))
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Biến trạng thái
        self.queue_image = Queue(maxsize=1)
        self.dynamic_head_deg = 0.0
        
        # Markers
        self.START_MARKER = b'\xff\xd8' 
        self.END_MARKER = b'\xff\xd9'
        self.BUFFER_SIZE = 65535

    def start_threads(self):
        print("[Network] Khởi động các luồng giao tiếp UDP...")
        threading.Thread(target=self._receive_image_thread, daemon=True).start()
        threading.Thread(target=self._receive_poly_thread, daemon=True).start()

    def _receive_image_thread(self):
        buffer = b''
        while True:
            try:
                data, _ = self.sock_img.recvfrom(self.BUFFER_SIZE)
                buffer += data
                end_idx = buffer.rfind(self.END_MARKER)
                if end_idx != -1:
                    start_idx = buffer.rfind(self.START_MARKER, 0, end_idx)
                    if start_idx != -1:
                        jpg_data = buffer[start_idx:end_idx+2]
                        if self.queue_image.full():
                            try: self.queue_image.get_nowait() 
                            except: pass
                        self.queue_image.put(jpg_data)
                    buffer = buffer[end_idx+2:]
                
                if len(buffer) > 1024 * 1024: buffer = b''
            except Exception: pass

    def _receive_poly_thread(self):
        while True:
            try:
                data, _ = self.sock_poly.recvfrom(1024)
                self.dynamic_head_deg = float(data.decode('utf-8'))
            except Exception: pass

    def get_latest_image_data(self):
        if self.queue_image.empty(): return None
        return self.queue_image.get()

    def get_heading(self):
        return self.dynamic_head_deg

    def send_command(self, cmd):
        try:
            self.sock_send.sendto(str(cmd).encode('utf-8'), (self.ip_send, self.port_send))
        except Exception as e:
            print(f"[Network Error] Lỗi truyền tín hiệu: {e}")