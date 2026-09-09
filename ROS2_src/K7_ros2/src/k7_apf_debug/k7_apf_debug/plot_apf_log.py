#!/usr/bin/env python3
# coding=utf-8
"""APF + Stanley 调参日志绘图。

读取 apf_recorder 输出的 CSV，出图：
  [左上] 实际轨迹 XY + 参考圆（含起点），判断循迹整体偏差/收敛
  [右上] cte（横向误差）与 psi_e（航向误差）随时间：调 K_PSI/K_CTE
  [左下] v_cmd vs v_act：看线速度限幅/遇障减速是否生效
  [右下] w_cmd / w_stanley / w_apf / w_act：看转弯分里 APF 避障分量多大、有无饱和/振荡
另存一张"三路测距"图（有障碍数据时才画）：调 RHO_0/K_REP。

用法：
    ros2 run k7_apf_debug plot_apf <xxx.csv>
    ros2 run k7_apf_debug plot_apf <xxx.csv> --show      # 弹出窗口逐张看
"""

import argparse
import csv
import math
import os

import matplotlib
# 默认 Agg：K7 上无显示器也能出图。想在 PC 上弹窗交互看，用 --interactive（内部切回 TkAgg）
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FIELDS = ['t', 'x', 'y', 'theta', 'cte', 'psi_e',
          'v_cmd', 'w_cmd', 'w_stanley', 'w_apf', 'v_act', 'w_act',
          'd_front', 'd_left', 'd_right']

# 尽量选一个能显示中文的字体（无则退回默认，中文标题可能变方块）
def _setup_cjk_font():
    import matplotlib.font_manager as fm
    for name in ['Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'Source Han Sans SC',
                 'Microsoft YaHei', 'SimHei', 'PingFang SC', 'Arial Unicode MS']:
        if any(f.name == name for f in fm.fontManager.ttflist):
            plt.rcParams['font.sans-serif'] = [name]
            break
    plt.rcParams['axes.unicode_minus'] = False


def load(path):
    rows = {k: [] for k in FIELDS}
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            for k in FIELDS:
                try:
                    rows[k].append(float(r[k]))
                except (TypeError, ValueError):
                    rows[k].append(float('nan'))
    return rows


def plot_main(rows, radius, out, ref='circle', line_len=3.0):
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))

    # 1. XY + 参考路径
    a = ax[0, 0]
    a.plot(rows['x'], rows['y'], '-', lw=1.2, label='actual')
    if ref == 'line':
        a.plot([0, line_len], [0, 0], '--', lw=0.8, color='gray', label='ref line')
        a.set_title('轨迹 vs 参考直线 (y=0)')
    else:
        th = [2 * math.pi * i / 360 for i in range(361)]
        a.plot([radius * math.cos(t) for t in th],
               [radius * math.sin(t) for t in th], '--', lw=0.8,
               color='gray', label='ref circle')
        a.set_title('轨迹 vs 参考圆')
    if rows['x'] and rows['y']:
        a.plot(rows['x'][0], rows['y'][0], 'gs', ms=8, label='start')
    a.set_xlabel('x (m)'); a.set_ylabel('y (m)')
    a.legend(); a.axis('equal'); a.grid(alpha=.3)

    # 2. cte & psi_e
    a = ax[0, 1]
    a.plot(rows['t'], rows['cte'], label='cte (m)')
    a.plot(rows['t'], rows['psi_e'], label='psi_e (rad)')
    a.axhline(0, color='gray', lw=.8)
    a.set_xlabel('t (s)'); a.set_title('横向误差 / 航向误差'); a.legend(); a.grid(alpha=.3)

    # 3. v_cmd vs v_act
    a = ax[1, 0]
    a.plot(rows['t'], rows['v_cmd'], label='v_cmd')
    a.plot(rows['t'], rows['v_act'], label='v_act')
    a.set_xlabel('t (s)'); a.set_ylabel('m/s'); a.set_title('线速度 指令 vs 实际')
    a.legend(); a.grid(alpha=.3)

    # 4. w 分量
    a = ax[1, 1]
    a.plot(rows['t'], rows['w_cmd'], lw=1.4, label='w_cmd')
    a.plot(rows['t'], rows['w_stanley'], label='w_stanley')
    a.plot(rows['t'], rows['w_apf'], label='w_apf')
    a.plot(rows['t'], rows['w_act'], ls=':', label='w_act')
    a.set_xlabel('t (s)'); a.set_ylabel('rad/s'); a.set_title('角速度：循迹/避障分量')
    a.legend(); a.grid(alpha=.3)

    fig.suptitle(os.path.basename(out), fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out, dpi=120)
    print(f'图已保存: {out}')
    return fig


def plot_dist(rows, out):
    d = {k: rows[k] for k in ('t', 'd_front', 'd_left', 'd_right')}
    if max(d['d_front'] + d['d_left'] + d['d_right']) >= 4.9:
        return None
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(d['t'], d['d_front'], label='front 0°')
    ax.plot(d['t'], d['d_left'], label='left +45°')
    ax.plot(d['t'], d['d_right'], label='right -45°')
    ax.set_xlabel('t (s)'); ax.set_ylabel('m'); ax.set_title('三路测距')
    ax.legend(); ax.grid(alpha=.3)
    fig.tight_layout()
    out2 = out.replace('.png', '_dist.png')
    fig.savefig(out2, dpi=120)
    print(f'图已保存: {out2}')
    return fig


def main():
    ap = argparse.ArgumentParser(description='APF+Stanley 调参日志绘图')
    ap.add_argument('csv', help='apf_recorder 输出的 csv')
    ap.add_argument('--ref', choices=['circle', 'line'], default='circle',
                    help='参考路径类型：circle=参考圆(默认) / line=直线 y=0 沿 +x')
    ap.add_argument('--radius', type=float, default=0.6, help='参考圆半径 (ref=circle 时用，默认0.6)')
    ap.add_argument('--line-len', type=float, default=3.0, help='参考直线长度 m (ref=line 时用，默认3.0)')
    ap.add_argument('--out', default=None, help='输出 png 路径(默认同目录 <名>_plot.png)')
    ap.add_argument('--interactive', action='store_true',
                    help='PC 上有显示时弹窗逐张看（内部切 TkAgg，须在出图前切，故放这里）')
    args = ap.parse_args()

    if args.interactive:
        matplotlib.use('TkAgg')     # 必须在任何 figure 创建前切换
    _setup_cjk_font()

    rows = load(args.csv)
    out = args.out or os.path.splitext(args.csv)[0] + '_plot.png'
    figs = [plot_main(rows, args.radius, out, args.ref, args.line_len)]
    dfig = plot_dist(rows, out)
    if dfig is not None:
        figs.append(dfig)

    if args.interactive:
        for f in figs:
            plt.show()


if __name__ == '__main__':
    main()
