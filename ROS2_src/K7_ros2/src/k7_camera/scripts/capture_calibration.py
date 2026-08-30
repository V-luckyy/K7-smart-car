#!/usr/bin/env python3
# k7_camera/capture_calibration.py
# 棋盘格标定图采集脚本 —— 浏览器实时预览 + SSH 终端回车抓帧（无头版）
#
# 为什么不用 cv2.imshow：
#   K7 无头（无 X11），imshow 会触发 Mali GPU 的 EGL/OpenGL 初始化并段错误。
#   改用 HTTP MJPEG 把画面流到 PC 浏览器（http://<K7_IP>:8888/），
#   抓帧触发用 SSH 终端的回车（非阻塞 select 读 stdin），保存的是按下回车那一刻的最新帧。
#
# 曝光处理：
#   - 默认自动曝光 + 预热（丢弃前 --warmup 秒的帧，让自动曝光收敛，避免欠曝）
#   - 可选 --exposure 手动锁曝光（标定时固定曝光更稳定，角点检测一致）
#
# 用法：
#   python3 capture_calibration.py                  # 浏览器看画面，SSH 终端回车抓帧
#   python3 capture_calibration.py --auto 2.0       # 每 2 秒自动抓一帧（无需回车）
#   python3 capture_calibration.py --exposure 350   # 手动锁曝光
#   python3 capture_calibration.py -o ~/calib --warmup 3.0 --port 8888

import argparse
import os
import select
import sys
import threading
import time

import cv2
import http.server
import socketserver

VIDEO_DEV = 73
WIDTH, HEIGHT = 3840, 1080
STREAM_WIDTH = 1280        # 浏览器流宽度（全分辨率 3840 太重，缩到 1280 看棋盘够用）

# 最新帧（主循环写、HTTP 流线程读）
latest_frame = None
frame_lock = threading.Lock()


class ThreadedServer(socketserver.ThreadingTCPServer):
    daemon_threads = True   # 浏览器断开后不阻塞进程退出
    allow_reuse_address = True  # 允许端口复用，避免崩溃后 TIME_WAIT 导致绑定失败


class MjpegHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with frame_lock:
                    frame = latest_frame
                if frame is None:
                    time.sleep(0.01)
                    continue
                # 缩到 STREAM_WIDTH 再编码，降低浏览器带宽（保存仍是全分辨率）
                scale = STREAM_WIDTH / float(frame.shape[1])
                preview = cv2.resize(frame, (STREAM_WIDTH, int(frame.shape[0] * scale)))
                _, jpg = cv2.imencode('.jpg', preview)
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n')
                self.wfile.write(jpg.tobytes())
                self.wfile.write(b'\r\n')
        except Exception:
            pass   # 浏览器断开会抛异常，静默结束该连接

    def log_message(self, *args):
        pass   # 关掉默认访问日志，避免刷屏


def save_pair(frame, actual_w, left_dir, right_dir, idx):
    left = frame[:, : actual_w // 2]
    right = frame[:, actual_w // 2:]
    left_path = os.path.join(left_dir, f"left_{idx:03d}.png")
    right_path = os.path.join(right_dir, f"right_{idx:03d}.png")
    cv2.imwrite(left_path, left)
    cv2.imwrite(right_path, right)
    print(f"[{idx}] 已保存 {left_path} 和 {right_path}")


def stdin_ready():
    return bool(select.select([sys.stdin], [], [], 0)[0])


def main():
    global latest_frame
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", type=float, default=0.0,
                    help="自动采集间隔（秒）；0 表示手动回车抓帧")
    ap.add_argument("-o", "--out", default="~/calibration_images",
                    help="输出根目录")
    ap.add_argument("--dev", type=int, default=VIDEO_DEV,
                    help="V4L2 设备号（默认 73）")
    ap.add_argument("--port", type=int, default=8888,
                    help="预览流 HTTP 端口（默认 8888）")
    ap.add_argument("--warmup", type=float, default=2.0,
                    help="预热秒数：丢弃前几秒帧让自动曝光收敛（默认 2.0）")
    ap.add_argument("--exposure", type=float, default=None,
                    help="手动曝光值（单位因驱动而异；先用 v4l2-ctl 查量程）")
    args = ap.parse_args()

    out_dir = os.path.expanduser(args.out)
    left_dir = os.path.join(out_dir, "left")
    right_dir = os.path.join(out_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    cap = cv2.VideoCapture(args.dev)
    # 关键：MJPG 才能拿到 3840x1080 全分辨率；YUYV 只有 640x480
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        print(f"无法打开 /dev/video{args.dev}，请检查摄像头连接", file=sys.stderr)
        sys.exit(1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"已打开 /dev/video{args.dev}：{actual_w}x{actual_h}")

    # ---- 曝光处理 ----
    if args.exposure is not None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        ok_exp = cap.set(cv2.CAP_PROP_EXPOSURE, args.exposure)
        print(f"手动曝光 value={args.exposure}（设曝光返回 {ok_exp}）")
    else:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        print(f"自动曝光已启用；预热 {args.warmup:.1f}s 让曝光收敛（丢弃预热帧）...")
        t0 = time.time()
        while time.time() - t0 < args.warmup:
            cap.read()
        print("预热完成")

    # 启动浏览器预览流（后台线程）
    server = ThreadedServer(("0.0.0.0", args.port), MjpegHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    print(f"浏览器预览：http://<K7_IP>:{args.port}/   （PC 浏览器打开看实时画面）")
    if args.auto > 0:
        print(f"自动模式：每 {args.auto:.1f}s 抓一帧；SSH 终端输入 q 回车退出")
    else:
        print("手动模式：SSH 终端【回车】抓当前帧，输入 q 回车退出")

    idx = 0
    last_capture = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # 更新最新帧（浏览器流和抓帧共用同一份）
        with frame_lock:
            latest_frame = frame.copy()

        should_capture = False
        if args.auto > 0:
            if time.time() - last_capture >= args.auto:
                should_capture = True
        elif stdin_ready():
            cmd = sys.stdin.readline().strip().lower()
            if cmd == 'q':
                break
            should_capture = True   # 回车或任意非 q 输入都抓当前帧

        if should_capture:
            save_pair(latest_frame, actual_w, left_dir, right_dir, idx)
            idx += 1
            last_capture = time.time()

    cap.release()
    print(f"共保存 {idx} 组，结束。")


if __name__ == "__main__":
    main()
