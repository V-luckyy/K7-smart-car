#!/usr/bin/env python3
"""MPC CSV 轨迹绘图：实时刷新 / 离线回放 / 导出 PNG。

纯 Python + matplotlib，不依赖 ROS2，PC 或 K7 都能跑。
数据来源：k7_mpc 节点写的 ~/mpc_log/mpc_*.csv（每行一条控制周期，写完即 flush）。

用法：
    python3 plot_trajectory.py                                   # 自动用 ~/mpc_log/ 里最新 csv，实时刷新
    python3 plot_trajectory.py --file mpc_xxx.csv                # 指定 csv（可传 Windows/板子上的路径）
    python3 plot_trajectory.py --file mpc_xxx.csv --replay       # 离线回放动画（文件已固定不增长）
    python3 plot_trajectory.py --file mpc_xxx.csv --save-png out.png  # 无显示环境（headless）导出最终图
    python3 plot_trajectory.py --interval 0.5                    # 轮询间隔秒（默认 0.25）

图表内容：
    蓝线      小车真实轨迹（odom 系 odom_x/odom_y）
    橙线      最近一段轨迹（高亮）
    灰虚线    参考 8 字路径（由日志 origin/ref_theta0/path_a 重建，缺列则省略）
    绿 ×      当前参考点（path 系转 odom 系）
    红箭头    当前朝向；黄线为三路红外射线（front/left45/right45）
    文本框    当前 tick / v / omega / d_min / risk / 避障标志 / 求解耗时
"""

import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib

SENSOR_ANGLE_OFFSETS = {"front": 0.0, "left45": math.pi / 4.0, "right45": -math.pi / 4.0}
SENSOR_MAX_RANGE = 1.0          # 射线绘制过滤，与 mpc_lib/common_config.py 一致
TRAIL_POINTS = 60               # 高亮轨迹尾部点数
HEADING_LEN = 0.22              # 朝向箭头长度（m）


def generate_eight_path(a, points=1400):
    """重建参考 8 字路径（path 局部系），与 mpc_lib/path_model.py 一致。"""
    ts = np.linspace(0.0, 2.0 * math.pi, points)
    return a * np.sin(ts), a * np.sin(ts) * np.cos(ts)


def transform_to_odom(px, py, origin, ref_theta0):
    """path 局部系 → odom 系（SE(2) 逆变换，与 mpc_node._to_path_frame 互为逆）。"""
    x0, y0, theta0 = origin
    alpha = ref_theta0 - theta0
    ca, sa = math.cos(alpha), math.sin(alpha)
    return x0 + ca * px + sa * py, y0 - sa * px + ca * py


def latest_log_file():
    log_dir = os.path.expanduser("~/mpc_log")
    if not os.path.isdir(log_dir):
        return None
    files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".csv")]
    return max(files, key=os.path.getmtime) if files else None


class TrajectoryPlot:
    def __init__(self, args, plt, cjk_font=None):
        self.args = args
        self.plt = plt
        self.cjk_font = cjk_font
        self.cols = None            # csv 列名
        self.file = None
        self.reader = None
        self.rows = []              # 已读行（dict）
        self.replay_idx = 0
        self.meta_ready = False
        self.origin = None
        self.ref_theta0 = None
        self.ref_x = None
        self.ref_y = None

        self.fig, self.ax = plt.subplots(figsize=(9, 7))
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")

        self.traj_line, = self.ax.plot([], [], "-", color="#1f77b4", lw=1.5, label="轨迹")
        self.trail_line, = self.ax.plot([], [], "-", color="#ff7f0e", lw=2.2, alpha=0.9, label="最近轨迹")
        self.ref_line, = self.ax.plot([], [], "--", color="#7f7f7f", lw=1.0, label="参考 8 字")
        self.ref_pt, = self.ax.plot([], [], "x", color="#2ca02c", ms=8, mew=2, label="当前参考点")
        self.heading, = self.ax.plot([], [], "-", color="#d62728", lw=2.0)
        self.rays = [
            self.ax.plot([], [], "-", color="#ff7f0e", alpha=0.55, lw=1.2)[0]
            for _ in SENSOR_ANGLE_OFFSETS
        ]
        # 信息框等宽字体，若检测到中文字体则追加为回退，保证中文不显示方框
        mono_family = (["DejaVu Sans Mono", cjk_font] if cjk_font
                       else ["DejaVu Sans Mono"])
        self.info = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes, va="top", ha="left",
            fontsize=9, family=mono_family,
            bbox=dict(boxstyle="round,pad=0.4", fc="white", alpha=0.85),
        )
        self.ax.legend(loc="lower right", fontsize=8)

    # ---------- 数据读取 ----------
    def _open_live(self):
        self.file = open(self.args.file, "r", newline="")
        self.reader = csv.DictReader(self.file)
        self.cols = self.reader.fieldnames

    def _read_new(self):
        """从文件当前位置读新增行（file 对象保持读取游标）。"""
        return [row for row in self.reader]

    @staticmethod
    def _load_all(path):
        """一次性读全文件。"""
        with open(path, "r", newline="") as f:
            reader = csv.DictReader(f)
            return reader.fieldnames, [row for row in reader]

    # ---------- 绘制 ----------
    @staticmethod
    def _g(row, name, default=float("nan")):
        try:
            v = float(row[name])
            return v if math.isfinite(v) else default
        except (KeyError, TypeError, ValueError):
            return default

    def _build_meta(self, row):
        """用日志里的恒定列重建参考路径与坐标系变换（只做一次）。"""
        if self.meta_ready:
            return
        need = ("origin_x", "origin_y", "origin_theta", "ref_theta0", "path_a")
        if self.cols is None or not all(c in self.cols for c in need):
            return
        vals = [self._g(row, c) for c in need]
        if not all(math.isfinite(v) for v in vals):
            return
        self.origin = tuple(vals[:3])
        self.ref_theta0 = vals[3]
        pxs, pys = generate_eight_path(vals[4])
        self.ref_x, self.ref_y = transform_to_odom(pxs, pys, self.origin, self.ref_theta0)
        self.ref_line.set_data(self.ref_x, self.ref_y)
        self.meta_ready = True

    def _render(self, rows):
        if not rows:
            return
        self._build_meta(rows[-1])

        xs = np.array([self._g(r, "odom_x") for r in rows])
        ys = np.array([self._g(r, "odom_y") for r in rows])
        self.traj_line.set_data(xs, ys)
        n = len(xs)
        k = max(0, n - TRAIL_POINTS)
        self.trail_line.set_data(xs[k:], ys[k:])

        last = rows[-1]
        x0, y0, th = self._g(last, "odom_x"), self._g(last, "odom_y"), self._g(last, "odom_theta")
        self.heading.set_data([x0, x0 + HEADING_LEN * math.cos(th)],
                              [y0, y0 + HEADING_LEN * math.sin(th)])

        for i, name in enumerate(SENSOR_ANGLE_OFFSETS):
            d = self._g(last, name)
            if d < SENSOR_MAX_RANGE - 1e-9:
                ang = th + SENSOR_ANGLE_OFFSETS[name]
                self.rays[i].set_data([x0, x0 + d * math.cos(ang)],
                                      [y0, y0 + d * math.sin(ang)])
            else:
                self.rays[i].set_data([], [])

        if self.meta_ready:
            prx, pry = transform_to_odom(self._g(last, "path_x"), self._g(last, "path_y"),
                                         self.origin, self.ref_theta0)
            self.ref_pt.set_data([prx], [pry])
        else:
            self.ref_pt.set_data([], [])

        info = (f"tick={int(self._g(last, 'tick', 0))}\n"
                f"v={self._g(last, 'v'):.2f}  ω={self._g(last, 'omega'):+.2f}\n"
                f"d_min={self._g(last, 'd_min'):.2f} m   risk={self._g(last, 'risk'):.2f}\n"
                f"避障={int(self._g(last, 'avoidance_active', 0))}  "
                f"CBF={int(self._g(last, 'cbf_active', 0))}\n"
                f"solve={self._g(last, 'solve_time_ms'):.0f} ms  "
                f"status={int(self._g(last, 'solver_status', 0))}")
        self.info.set_text(info)

        # 全视图：参考 8 字路径 + 小车轨迹都纳入显示范围（实时/回放/导出一致）
        margin = 0.4
        xmin, xmax = float(np.nanmin(xs)), float(np.nanmax(xs))
        ymin, ymax = float(np.nanmin(ys)), float(np.nanmax(ys))
        if self.ref_x is not None:
            xmin = min(xmin, float(np.min(self.ref_x)))
            xmax = max(xmax, float(np.max(self.ref_x)))
            ymin = min(ymin, float(np.min(self.ref_y)))
            ymax = max(ymax, float(np.max(self.ref_y)))
        self.ax.set_xlim(xmin - margin, xmax + margin)
        self.ax.set_ylim(ymin - margin, ymax + margin)

    def _artists(self):
        return ([self.traj_line, self.trail_line, self.ref_line, self.ref_pt,
                 self.heading, self.info] + self.rays)

    # ---------- 动画 ----------
    def _update_live(self, frame):
        self.rows.extend(self._read_new())
        if self.rows:
            self._render(self.rows)
        return self._artists()

    def _update_replay(self, frame):
        if self.replay_idx < len(self.rows):
            self.replay_idx += 1
        if self.replay_idx > 0:
            self._render(self.rows[:self.replay_idx])
        return self._artists()

    def run(self):
        from matplotlib.animation import FuncAnimation

        if self.args.save_png:
            self.cols, self.rows = self._load_all(self.args.file)
            if not self.rows:
                print(f"错误：{self.args.file} 无有效数据行", file=sys.stderr)
                return 1
            self._render(self.rows)
            self.fig.savefig(self.args.save_png, dpi=150, bbox_inches="tight")
            print(f"已导出：{self.args.save_png}（{len(self.rows)} 行）")
            return 0

        if self.args.replay:
            self.cols, self.rows = self._load_all(self.args.file)
            anim = FuncAnimation(self.fig, self._update_replay,
                                 interval=self.args.interval * 1000, blit=False,
                                 cache_frame_data=False)
        else:
            self._open_live()
            self.rows = self._read_new()      # 先加载已有行，再增量轮询
            anim = FuncAnimation(self.fig, self._update_live,
                                 interval=self.args.interval * 1000, blit=False,
                                 cache_frame_data=False)
        self.plt.show()
        return 0


def parse_args():
    p = argparse.ArgumentParser(description="MPC CSV 轨迹绘图（实时/回放/PNG）")
    p.add_argument("--file", default=None, help="csv 路径；缺省用 ~/mpc_log/ 最新文件")
    p.add_argument("--replay", action="store_true", help="离线回放动画（文件不再增长时用）")
    p.add_argument("--save-png", default=None, help="headless 导出 PNG 路径（用 Agg 后端）")
    p.add_argument("--interval", type=float, default=0.25, help="刷新/回放间隔秒（默认 0.25）")
    return p.parse_args()


def main():
    args = parse_args()
    if args.file is None:
        args.file = latest_log_file()
        if args.file is None:
            print("错误：找不到 ~/mpc_log/ 下的 csv，请用 --file 指定路径", file=sys.stderr)
            return 1
        print(f"自动选用最新日志：{args.file}")

    if not os.path.isfile(args.file):
        print(f"错误：文件不存在 {args.file}", file=sys.stderr)
        return 1

    if args.save_png:
        matplotlib.use("Agg")        # 必须在 import pyplot 之前设置
    import matplotlib.pyplot as plt

    cjk_font = _setup_cjk_font(plt)  # 尽力启用中文字体（找不到则中文变方框，不影响图表）
    plot = TrajectoryPlot(args, plt, cjk_font)
    return plot.run()


def _setup_cjk_font(plt):
    """按常见中文字体名逐一匹配系统已装字体；找到则设为默认并返回字体名，否则返回 None。"""
    from matplotlib import font_manager
    candidates = [
        "Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Micro Hei",
        "PingFang SC", "Source Han Sans SC",
    ]
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in installed:
            plt.rcParams["font.sans-serif"] = [name] + plt.rcParams.get("font.sans-serif", [])
            plt.rcParams["axes.unicode_minus"] = False  # 负号正常显示
            return name
    plt.rcParams["axes.unicode_minus"] = False
    return None


if __name__ == "__main__":
    sys.exit(main())
