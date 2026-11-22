"""Formation error analysis for the per-car reference-path design.

Two different questions, two different references:

  leader   follows the reference shape. Its target is the reference point at the
           leader's own arc progress, so this number is the lateral
           (cross-track) tracking error.
  follower follows the leader. Its target is the leader's ACTUAL pose at the
           same timestamp plus the target formation slot, so this number also
           contains any along-track drift between the two cars.

At t = 0 the leader sits on the reference start (error 0) while the followers
still stand in the initial column, so their error equals the distance between
the initial slot and the target slot.

Usage:  python analyze_formation.py <data_dir> [spiral|figure8]
"""
import csv
import math
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ROS2_src/K7_ros2/src/k7_tasks'))
from k7_tasks.ref_path import SHAPES, INITIAL_FORMATION, TARGET_FORMATION  # noqa: E402

CARS = ['car1', 'car2', 'car3']
COLORS = {'car1': 'tab:blue', 'car2': 'tab:orange', 'car3': 'tab:green'}


def read_odom(path):
    ts, xs, ys, yaws = [], [], [], []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            ts.append(float(row['t']))
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            yaws.append(float(row.get('yaw', 0.0)))
    return ts, xs, ys, yaws


def build(path):
    """Return (xs, ys, arc) for a spatial path."""
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    arc = [0.0]
    for i in range(1, len(path)):
        arc.append(arc[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
    return xs, ys, arc


def point_at(xs, ys, arc, u):
    if u <= arc[0]:
        return xs[0], ys[0]
    if u >= arc[-1]:
        return xs[-1], ys[-1]
    lo, hi = 0, len(arc) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if arc[mid] <= u:
            lo = mid
        else:
            hi = mid
    frac = (u - arc[lo]) / max(arc[hi] - arc[lo], 1e-9)
    return xs[lo] + frac * (xs[hi] - xs[lo]), ys[lo] + frac * (ys[hi] - ys[lo])


def project_arc(xs, ys, arc, x, y):
    """Arc length of the closest point on the path to (x, y)."""
    best_d, best_u = float('inf'), 0.0
    for i in range(len(xs) - 1):
        ax, ay, bx, by = xs[i], ys[i], xs[i + 1], ys[i + 1]
        dx, dy = bx - ax, by - ay
        seg = dx * dx + dy * dy
        t = 0.0 if seg <= 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg))
        px, py = ax + t * dx, ay + t * dy
        dist = math.hypot(x - px, y - py)
        if dist < best_d:
            best_d, best_u = dist, arc[i] + t * (arc[i + 1] - arc[i])
    return best_u


def interp(ts, vs, t):
    if t <= ts[0]:
        return vs[0]
    if t >= ts[-1]:
        return vs[-1]
    lo, hi = 0, len(ts) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if ts[mid] <= t:
            lo = mid
        else:
            hi = mid
    frac = (t - ts[lo]) / max(ts[hi] - ts[lo], 1e-9)
    return vs[lo] + frac * (vs[hi] - vs[lo])


def interp_yaw(ts, yaws, t):
    """Wrap-safe yaw interpolation (linear on the unit circle, not on the angle)."""
    c = interp(ts, [math.cos(a) for a in yaws], t)
    s = interp(ts, [math.sin(a) for a in yaws], t)
    return math.atan2(s, c)


def slot_xy(gap, lateral, yaw):
    """Slot offset expressed in the world frame for a body heading `yaw`."""
    return (-gap * math.cos(yaw) - lateral * math.sin(yaw),
            -gap * math.sin(yaw) + lateral * math.cos(yaw))


def stats(vals):
    if not vals:
        return {}
    n = len(vals)
    mean = sum(vals) / n
    return dict(n=n, mean=mean, rms=math.sqrt(sum(v * v for v in vals) / n),
                max=max(vals), std=math.sqrt(sum((v - mean) ** 2 for v in vals) / n))


def fmt(d):
    return {k: round(v, 2) for k, v in d.items()}


def main(data_dir, shape='spiral'):
    data_dir = str(Path(data_dir).resolve())
    if shape not in SHAPES:
        shape = 'spiral'
    ref = SHAPES[shape]()
    rx, ry, rarc = build(ref)

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    ax.plot(rx, ry, 'k--', lw=1.2, alpha=0.7, label='reference ' + shape)

    # The leader is read first: every follower target is built from its pose.
    lt, lx, ly, lh = read_odom(os.path.join(data_dir, 'odom_car1.csv'))
    yaw0 = lh[0]

    def leader_pose(t):
        return interp(lt, lx, t), interp(lt, ly, t), interp_yaw(lt, lh, t)

    for car in CARS:
        ts, xs, ys, yaws = read_odom(os.path.join(data_dir, 'odom_' + car + '.csv'))

        if car == 'car1':
            # Target: nearest point on the reference (lateral tracking error).
            # Taking the reference point at the leader's own arc length would
            # fold in a constant phase lag: the car runs a slightly tighter
            # spiral (19.83 m vs 20.14 m), so equal arc length is a few degrees
            # ahead of equal angle, which shows up as a fake 17 cm offset.
            fx, fy = lx, ly
            errs = []
            for i in range(len(ts)):
                u = project_arc(rx, ry, rarc, lx[i], ly[i])
                tx, ty = point_at(rx, ry, rarc, u)
                errs.append(100.0 * math.hypot(lx[i] - tx, ly[i] - ty))
        else:
            # Target: leader ACTUAL pose at the same timestamp + target slot.
            g0, l0 = INITIAL_FORMATION[car]
            g1, l1 = TARGET_FORMATION[car]
            ox, oy = slot_xy(g0, l0, yaw0)
            fx = [x + ox for x in xs]
            fy = [y + oy for y in ys]
            errs, target = [], []
            for i, t in enumerate(ts):
                px, py, pyaw = leader_pose(t)
                sx, sy = slot_xy(g1, l1, pyaw)
                tx, ty = px + sx, py + sy
                target.append((tx, ty))
                errs.append(100.0 * math.hypot(fx[i] - tx, fy[i] - ty))
            ax.plot([p[0] for p in target], [p[1] for p in target], ':',
                    color=COLORS[car], lw=1.1, alpha=0.85,
                    label=car + ' target = leader + slot')

        length = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                     for i in range(1, len(xs)))
        dur = ts[-1] - ts[0] if len(ts) > 1 else 0.0
        print('%s: len=%.2f m dur=%.1f s avg=%.3f m/s err_cm=%s'
              % (car, length, dur, length / dur if dur > 0 else 0.0, fmt(stats(errs))))

        ax.plot(fx, fy, color=COLORS[car], lw=1.4, label=car)
        ax.scatter([fx[0]], [fy[0]], color=COLORS[car], marker='o', s=60,
                   zorder=5, edgecolor='k')
        ax.scatter([fx[-1]], [fy[-1]], color=COLORS[car], marker='s', s=60,
                   zorder=5, edgecolor='k')
        ax2.semilogy(ts, [max(e, 1e-2) for e in errs], color=COLORS[car], lw=1.2, label=car)

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Leader vs reference; followers vs leader-actual + slot '
                 '(circle = start, square = end)')

    ax2.set_xlabel('t (s, per-car relative time)')
    ax2.set_ylabel('error to target (cm, log scale)')
    ax2.grid(True, alpha=0.3, which='both')
    ax2.legend(fontsize=8)
    ax2.set_title('Leader -> reference; follower -> leader + slot (same timestamp)')

    plt.tight_layout()
    out = os.path.join(data_dir, 'formation_plot.png')
    plt.savefig(out, dpi=120)
    print('saved:', out)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '.',
         sys.argv[2] if len(sys.argv) > 2 else 'spiral')
