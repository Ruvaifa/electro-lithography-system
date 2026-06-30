import http.server
import json
import os
import sys
import collections
from api import LithographySystem

class LogBuffer:
    def __init__(self, limit=200):
        self.buffer = collections.deque(maxlen=limit)
        self.stdout = sys.stdout

    def write(self, message):
        self.stdout.write(message)
        if message.strip():
            self.buffer.append(message.strip())

    def flush(self):
        self.stdout.flush()

log_buffer = LogBuffer()
sys.stdout = log_buffer
sys.stderr = log_buffer

system = LithographySystem()

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
        if self.path.startswith("/api/"):
            self.handle_api_get()
        else:
            original_path = self.path
            if self.path == "/":
                self.path = "/index.html"
            
            frontend_dir = os.path.join(os.getcwd(), "frontend")
            target_file = os.path.join(frontend_dir, self.path.lstrip("/"))
            
            if os.path.exists(target_file) and os.path.isfile(target_file):
                self.path = "/frontend" + original_path
                if original_path == "/":
                    self.path = "/frontend/index.html"
                super().do_GET()
            else:
                self.send_error(404, "File not found")

    def do_POST(self):
        if self.path.startswith("/api/"):
            self.handle_api_post()
        else:
            self.send_error(404, "Not Found")

    def handle_api_get(self):
        endpoint = self.path[len("/api/"):]
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

    def handle_api_post(self):
        endpoint = self.path[len("/api/"):]
        
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
    httpd = http.server.HTTPServer(server_address, APIHandler)
    print(f"Electro-Lithography Web Dashboard running at http://localhost:{port}/")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    print("Server shutting down.")

if __name__ == "__main__":
    port = 8080
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass
    run_server(port)
