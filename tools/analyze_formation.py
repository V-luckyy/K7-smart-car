"""Unified multi-car formation analysis + plotting.

Usage:  python analyze_formation.py <data_dir>

Reads from <data_dir>:
    odom_car1.csv / odom_car2.csv / odom_car3.csv      (columns: t,x,y,yaw)
    follow_car2.log / follow_car3.log                  (FollowLeader --feedback output)

Writes to <data_dir>:
    formation_plot.png    (reference spiral + three-car paths in the fleet frame,
                           plus follower gap/lateral error over time)

Prints: quantitative metrics (formation error, latency, path length/speed, tracking error).
Keep only raw data + plot for each run; this script is shared and reused.
"""
import csv
import math
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

CARS = ['car1', 'car2', 'car3']
# Each follower odom origin is that vehicle's initial fleet pose; shift back to
# the leader's fleet frame so every trajectory (and the reference) share axes.
FLEET_OFFSET = {'car1': (0.0, 0.0), 'car2': (-0.8, 0.0), 'car3': (-1.6, 0.0)}
COLORS = {'car1': 'tab:blue', 'car2': 'tab:orange', 'car3': 'tab:green'}
# Spiral parameters must match k7_tasks/ref_path.py outward_spiral().
SPIRAL = dict(inner=0.5, outer=1.5, turns=2, count=500)


def parse_follow(path):
    gap, lat, age = [], [], []
    cur = {}
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            line = line.strip()
            m = re.match(r'gap_error:\s*([-\d.eE+]+)', line)
            if m:
                cur['gap'] = float(m.group(1))
            m = re.match(r'lateral_error:\s*([-\d.eE+]+)', line)
            if m:
                cur['lat'] = float(m.group(1))
            m = re.match(r'leader_age:\s*([-\d.eE+]+)', line)
            if m and 'gap' in cur and 'lat' in cur:
                gap.append(cur['gap'])
                lat.append(cur['lat'])
                age.append(float(m.group(1)))
                cur = {}
    return gap, lat, age


def stats(vals):
    if not vals:
        return {}
    n = len(vals)
    mean = sum(vals) / n
    rms = math.sqrt(sum(v * v for v in vals) / n)
    return dict(n=n, mean=mean, rms=rms, max_abs=max(abs(v) for v in vals),
                std=math.sqrt(sum((v - mean) ** 2 for v in vals) / n))


def fmt(d):
    return {k: round(v, 4) for k, v in d.items()}


def read_odom(path):
    ts, xs, ys = [], [], []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            ts.append(float(row['t']))
            xs.append(float(row['x']))
            ys.append(float(row['y']))
    return ts, xs, ys


def path_speed(ts, xs, ys):
    length, max_v = 0.0, 0.0
    for i in range(1, len(xs)):
        d = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        dt = ts[i] - ts[i - 1]
        if dt > 0:
            max_v = max(max_v, d / dt)
        length += d
    dur = ts[-1] - ts[0] if len(ts) > 1 else 0.0
    return length, dur, (length / dur if dur > 0 else 0.0), max_v


def reference_spiral():
    inner, outer = SPIRAL['inner'], SPIRAL['outer']
    turns, count = SPIRAL['turns'], SPIRAL['count']
    end_angle = turns * 2 * math.pi
    growth = (outer - inner) / end_angle
    pts = [(0.0, 0.0)]
    for i in range(count):
        a = end_angle * i / (count - 1)
        r = inner + growth * a
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def main(data_dir):
    print('=== follower formation error & latency (FollowLeader feedback) ===')
    for car in ['car2', 'car3']:
        gap, lat, age = parse_follow(os.path.join(data_dir, 'follow_' + car + '.log'))
        print(car + ' gap_error(m):    ' + str(fmt(stats(gap))))
        print(car + ' lateral_error(m):' + str(fmt(stats(lat))))
        print(car + ' leader_age(s):   ' + str(fmt(stats(age))))
    print('=== three-car path length & speed (own odom frame) ===')
    odom = {}
    for car in CARS:
        ts, xs, ys = read_odom(os.path.join(data_dir, 'odom_' + car + '.csv'))
        length, dur, avg, mx = path_speed(ts, xs, ys)
        ox, oy = FLEET_OFFSET[car]
        odom[car] = ([x + ox for x in xs], [y + oy for y in ys])
        print('%s: len=%.2f m dur=%.1f s avg=%.3f m/s peak=%.3f m/s'
              % (car, length, dur, avg, mx))
    print('=== leader tracking error (vs reference spiral) ===')
    spiral = reference_spiral()
    _, lxs, lys = read_odom(os.path.join(data_dir, 'odom_car1.csv'))
    errs = [min(math.hypot(x - px, y - py) for px, py in spiral)
            for x, y in zip(lxs, lys)]
    print('car1 track_err(m): ' + str(fmt(stats(errs))))

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    ax.plot([p[0] for p in spiral], [p[1] for p in spiral], 'k--', lw=1.2,
            alpha=0.7, label='reference spiral')
    for car in CARS:
        xs, ys = odom[car]
        ax.plot(xs, ys, color=COLORS[car], label=car + ' odom', lw=1.5, alpha=0.9)
        ax.scatter([xs[0]], [ys[0]], color=COLORS[car], marker='o', s=70,
                   zorder=5, edgecolor='k')
        ax.scatter([xs[-1]], [ys[-1]], color=COLORS[car], marker='s', s=70,
                   zorder=5, edgecolor='k')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Formation in fleet frame (circle=start, square=end)')

    for car in ['car2', 'car3']:
        gap, lat, age = parse_follow(os.path.join(data_dir, 'follow_' + car + '.log'))
        ax2.plot(range(len(gap)), gap, color=COLORS[car], lw=1.2,
                 label=car + ' gap_error')
        ax2.plot(range(len(lat)), lat, color=COLORS[car], lw=0.9, alpha=0.5,
                 linestyle=':', label=car + ' lateral_error')
    ax2.axhline(0.0, color='k', lw=0.6)
    ax2.set_xlabel('feedback sample (0.5 s)')
    ax2.set_ylabel('error (m)')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)
    ax2.set_title('Follower formation error over time')

    plt.tight_layout()
    out = os.path.join(data_dir, 'formation_plot.png')
    plt.savefig(out, dpi=120)
    print('saved:', out)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '.')
