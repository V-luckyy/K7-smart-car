# k7_camera/camera_server.py
# 从 K7 板子 ~/Videos/camera_server.py 同步进项目
# 功能：HTTP MJPEG 服务器，流式输出 /dev/video73 的 LRCP 2MV 双目画面
# 访问：http://<K7_IP>:8888/

import cv2
import http.server
import socketserver


class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        cap = cv2.VideoCapture(73)
        try:
            while True:
                ret, frame = cap.read()
                if ret:
                    _, jpg = cv2.imencode('.jpg', frame)
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b'\r\n')
        except Exception:
            pass
        finally:
            cap.release()


def main():
    socketserver.TCPServer(('0.0.0.0', 8888), Handler).serve_forever()


if __name__ == '__main__':
    main()
