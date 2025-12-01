import rclpy
from rclpy.node import Node
import cv2
import threading
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import socketserver

# Global state for camera to be shared across threads
CAMERA = None
CAMERA_LOCK = threading.Lock()
ACTIVE_CLIENTS = 0
WEB_DIR = ""

class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global ACTIVE_CLIENTS, CAMERA, CAMERA_LOCK
        
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open(os.path.join(WEB_DIR, 'index.html'), 'rb') as f:
                self.wfile.write(f.read())
                
        elif self.path == '/main.js':
            self.send_response(200)
            self.send_header('Content-type', 'application/javascript')
            self.end_headers()
            with open(os.path.join(WEB_DIR, 'main.js'), 'rb') as f:
                self.wfile.write(f.read())

        elif self.path == '/video_feed':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            # Manage camera
            with CAMERA_LOCK:
                ACTIVE_CLIENTS += 1
                if CAMERA is None:
                    print('Opening camera /dev/video0')
                    CAMERA = cv2.VideoCapture('/dev/video0')
                    if not CAMERA.isOpened():
                        print('Could not open camera!')
                        ACTIVE_CLIENTS -= 1
                        return

            try:
                while True:
                    if CAMERA and CAMERA.isOpened():
                        ret, frame = CAMERA.read()
                        if ret:
                            ret, buffer = cv2.imencode('.jpg', frame)
                            frame_bytes = buffer.tobytes()
                            
                            self.wfile.write(b'--frame\r\n')
                            self.send_header('Content-Type', 'image/jpeg')
                            self.send_header('Content-Length', len(frame_bytes))
                            self.end_headers()
                            self.wfile.write(frame_bytes)
                            self.wfile.write(b'\r\n')
                        else:
                            time.sleep(0.1)
                    else:
                        time.sleep(0.1)
                    
                    time.sleep(0.03) # Limit framerate
            except Exception as e:
                print(f'Client disconnected: {e}')
            finally:
                with CAMERA_LOCK:
                    ACTIVE_CLIENTS -= 1
                    print(f'Active clients: {ACTIVE_CLIENTS}')
                    if ACTIVE_CLIENTS <= 0:
                        print('No active clients, closing camera')
                        if CAMERA:
                            CAMERA.release()
                            CAMERA = None

        else:
            self.send_error(404)
            self.end_headers()

class WebServer(Node):
    def __init__(self):
        super().__init__('web_server')
        self.get_logger().info('Starting Web Server Node...')
        
        global WEB_DIR
        import ament_index_python.packages
        try:
            share_dir = ament_index_python.packages.get_package_share_directory('hospibot_teleop')
            WEB_DIR = os.path.join(share_dir, 'web')
        except Exception as e:
            self.get_logger().warn(f"Could not find share directory: {e}. Using local 'web' folder.")
            WEB_DIR = os.path.join(os.getcwd(), 'web')

        self.get_logger().info(f"Serving web files from: {WEB_DIR}")

        # Start server in a separate thread
        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def start_server(self):
        # Allow reuse address to avoid "Address already in use" on restarts
        ThreadingHTTPServer.allow_reuse_address = True
        server = ThreadingHTTPServer(('0.0.0.0', 8000), RequestHandler)
        self.get_logger().info('Web server started on http://0.0.0.0:8000')
        server.serve_forever()

def main(args=None):
    rclpy.init(args=args)
    node = WebServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
