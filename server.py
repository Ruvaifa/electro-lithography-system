import http.server
import json
import os
import sys
import collections
import time
from socketserver import ThreadingMixIn
from api import LithographySystem

class ThreadingHTTPServer(ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True


class LogBuffer:
    def __init__(self, limit=200):
        self.buffer = collections.deque(maxlen=limit)
        self.stdout = sys.stdout
        os.makedirs("logs", exist_ok=True)
        self.log_file = open("logs/system_log.txt", "a", encoding="utf-8")

    def write(self, message):
        self.stdout.write(message)
        if message:
            self.log_file.write(message)
            self.log_file.flush()
        if message.strip():
            self.buffer.append(message.strip())

    def flush(self):
        self.stdout.flush()

log_buffer = LogBuffer()
sys.stdout = log_buffer
sys.stderr = log_buffer

system = LithographySystem()

# Basler camera global variables
camera_instance = None
camera_failed = False

def get_camera_frames():
    global camera_instance, camera_failed
    
    # Try importing cv2 and numpy dynamically to prevent startup failure if missing
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("[CAMERA] OpenCV/Numpy is missing. Stream placeholder will be a static text message.")
        # 1x1 black pixel JPEG bytes fallback
        mock_jpeg = (
            b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00`\x00`\x00\x00\xff\xdb\x00C\x00\x08\x06\x06'
            b'\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a'
            b'\x1f\x1e\x1d\x1a\x1c\x1c $.\' ",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x0b\x08\x00'
            b'\x01\x00\x01\x01\x01\x11\x01\xff\xc4\x00\x1f\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00'
            b'\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xda\x00\x08\x01\x01\x00'
            b'\x00?\x00\x37\xff\xd9'
        )
        while True:
            time.sleep(1.0)
            yield mock_jpeg

    if not camera_instance and not camera_failed:
        try:
            from pypylon import pylon
            from basler_camera import BaslerCamera
            camera_instance = BaslerCamera()
            camera_instance.open()
            camera_instance.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            print("[CAMERA] Basler Camera initialized successfully.")
        except Exception as e:
            print(f"[CAMERA] Physical camera initialization failed: {e}. Using simulated camera.")
            camera_failed = True
            
    while True:
        frame = None
        if camera_instance and not camera_failed:
            try:
                from pypylon import pylon
                if camera_instance.camera.IsGrabbing():
                    grab_result = camera_instance.camera.RetrieveResult(
                        100,
                        pylon.TimeoutHandling_ThrowException,
                    )
                    try:
                        if grab_result.GrabSucceeded():
                            frame = camera_instance.converter.Convert(grab_result).GetArray()
                    finally:
                        grab_result.Release()
            except Exception as e:
                print(f"[CAMERA] Failed to grab physical frame: {e}. Switching to simulated.")
                camera_failed = True
                try:
                    camera_instance.close()
                except Exception:
                    pass
                camera_instance = None
                
        if frame is None:
            # Generate simulated frame
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.line(frame, (320, 0), (320, 480), (45, 45, 45), 1)
            cv2.line(frame, (0, 240), (640, 240), (45, 45, 45), 1)
            cv2.circle(frame, (320, 240), 40, (0, 242, 254), 1)
            cv2.circle(frame, (320, 240), 5, (0, 242, 254), -1)
            
            # Oscillating probe simulated shadow
            t = time.time()
            offset_x = int(12 * np.sin(t * 3.14))
            offset_y = int(12 * np.cos(t * 3.14))
            cv2.rectangle(frame, (312 + offset_x, 232 + offset_y), (328 + offset_x, 248 + offset_y), (255, 0, 127), 1)
            
            cv2.putText(frame, "BASLER LIVE CAMERA STREAM (MODE 8/6)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, f"FPS: 15 | TIME: {time.strftime('%H:%M:%S')}", (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 120, 120), 1, cv2.LINE_AA)
            cv2.putText(frame, "SIMULATED VIEW: NO PHYSICAL HARDWARE DETECTED", (15, 455), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 255), 1, cv2.LINE_AA)
            
            time.sleep(0.066)
            
        _, jpeg = cv2.imencode('.jpg', frame)
        yield jpeg.tobytes()

class APIHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        clean_path = self.path.split("?")[0]
        if clean_path == "/api/camera/stream":
            self.handle_camera_stream()
        elif clean_path.startswith("/api/"):
            self.handle_api_get(clean_path)
        else:
            if clean_path == "/":
                clean_path = "/index.html"
            
            frontend_dir = os.path.join(os.getcwd(), "frontend")
            target_file = os.path.join(frontend_dir, clean_path.lstrip("/"))
            
            if os.path.exists(target_file) and os.path.isfile(target_file):
                self.path = "/frontend" + clean_path
                super().do_GET()
            else:
                self.send_error(404, "File not found")

    def handle_camera_stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            for frame_bytes in get_camera_frames():
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(frame_bytes)}\r\n\r\n'.encode())
                self.wfile.write(frame_bytes)
                self.wfile.write(b'\r\n')
        except Exception:
            pass

    def do_POST(self):
        clean_path = self.path.split("?")[0]
        if clean_path.startswith("/api/"):
            self.handle_api_post(clean_path)
        else:
            self.send_error(404, "Not Found")

    def handle_api_get(self, clean_path):
        endpoint = clean_path[len("/api/"):]
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()

        if endpoint == "ports":
            res = system.list_ports()
            self.wfile.write(json.dumps(res).encode())
        elif endpoint == "status":
            res = system.get_status()
            self.wfile.write(json.dumps(res).encode())
        elif endpoint == "log":
            res = {"logs": list(log_buffer.buffer)}
            self.wfile.write(json.dumps(res).encode())
        elif endpoint == "files":
            coords_dir = os.path.join(os.getcwd(), "coords")
            files = []
            if os.path.exists(coords_dir):
                for f in os.listdir(coords_dir):
                    if f.endswith(".txt"):
                        files.append(f[:-4])
            self.wfile.write(json.dumps(files).encode())
        else:
            self.wfile.write(json.dumps({"error": f"Unknown endpoint {endpoint}"}).encode())

    def handle_api_post(self, clean_path):
        endpoint = clean_path[len("/api/"):]
        
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length)
        payload = {}
        if post_data:
            try:
                payload = json.loads(post_data.decode())
            except Exception:
                pass

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()

        res = {"success": False, "error": "Unknown endpoint"}
        
        if endpoint == "connect":
            x_port = payload.get("x_port", "")
            y_port = payload.get("y_port", "")
            z_port = payload.get("z_port", "")
            res = system.connect(x_port, y_port, z_port)
        elif endpoint == "home":
            res = system.home_all()
        elif endpoint == "move":
            x = float(payload.get("x", 0.0))
            y = float(payload.get("y", 0.0))
            z = float(payload.get("z", 0.0))
            res = system.move(x, y, z)
        elif endpoint == "run_pattern":
            mode_num = int(payload.get("mode", 8))
            res = system.run_pattern(mode_num, payload)
        elif endpoint == "stop":
            res = system.stop()
        elif endpoint == "disconnect":
            res = system.disconnect()

        self.wfile.write(json.dumps(res).encode())

def run_server(port=8080):
    server_address = ('', port)
    httpd = ThreadingHTTPServer(server_address, APIHandler)
    httpd.timeout = 0.5
    print(f"Electro-Lithography Web Dashboard running at http://localhost:{port}/")
    try:
        while True:
            httpd.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        # Restore standard stdout/stderr for terminal shutdown prints
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        print("\n[SYSTEM] KeyboardInterrupt received. Initiating graceful shutdown...")
        try:
            print("[SYSTEM] Stopping any active stage movements...")
            system.stop()
        except Exception:
            pass
        try:
            print("[SYSTEM] Releasing serial connections...")
            system.disconnect()
        except Exception:
            pass
        print("[SYSTEM] Server shutdown completed successfully.")
        os._exit(0)

if __name__ == "__main__":
    port = 8080
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass
    run_server(port)
