"""Bounded planar geometry and command authority, independent of ROS."""
import math
import uuid


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def finite(*values):
    return all(math.isfinite(v) for v in values)


def limit_drive(v, w, track, wheel_limit, forward_only=False):
    if not finite(v, w, track, wheel_limit) or track <= 0 or wheel_limit <= 0:
        return 0.0, 0.0
    if forward_only:
        v = max(0.0, v)
        w = max(-2 * v / track, min(2 * v / track, w))
    left, right = v - w * track / 2, v + w * track / 2
    scale = max(1.0, abs(left) / wheel_limit, abs(right) / wheel_limit)
    return v / scale, w / scale


def project(path, x, y):
    """Nearest segment: distance, signed left correction, yaw, arc, total."""
    best = None
    arc = 0.0
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        t = max(0.0, min(1.0, ((x-a[0])*dx + (y-a[1])*dy) / length**2))
        px, py = a[0] + t*dx, a[1] + t*dy
        distance = math.hypot(x-px, y-py)
        item = (distance, ((px-x)*(-dy) + (py-y)*dx) / length,
                math.atan2(dy, dx), arc + t*length)
        if best is None or distance < best[0]:
            best = item
        arc += length
    if best is None:
        raise ValueError('path must contain a nonzero segment')
    return (*best, arc)


def valid_scan(ranges, minimum, maximum):
    return finite(minimum, maximum) and 0 < minimum < maximum and len(ranges) > 0 and all(
        (math.isfinite(r) and minimum <= r <= maximum) or r == math.inf
        for r in ranges)


def control_ready(odom_fresh, hardware_fresh, enabled, scan_fresh, scan_ok, no_perception_test):
    return odom_fresh and hardware_fresh and enabled and (
        no_perception_test or (scan_fresh and scan_ok))


class Authority:
    def __init__(self, candidate_timeout=0.25):
        self.source = 0
        self.token = bytes(16)
        self.held = False
        self.candidate = None
        self.timeout = candidate_timeout
        self.reason = 'idle'

    def revoke(self, reason='stopped'):
        self.source, self.token, self.candidate = 0, bytes(16), None
        self.reason = reason

    def select(self, source, stopped, ready):
        if source == 0:
            self.revoke()
            return True
        if source not in (1, 2) or self.held or self.source or not stopped or not ready:
            return False
        self.source, self.token = source, uuid.uuid4().bytes
        self.candidate = None
        self.reason = 'waiting for candidate'
        return True

    def hold(self, value):
        self.held = value
        self.revoke('hold' if value else 'released; explicit new task required')

    def receive(self, token, v, w, now):
        if self.source and token == self.token and finite(v, w):
            self.candidate = (v, w, now)

    def output(self, now, ready, heartbeat):
        if not self.source:
            return 0.0, 0.0
        if not ready or not heartbeat:
            self.revoke('sensor or task heartbeat lost')
            return 0.0, 0.0
        if self.candidate is None:
            return 0.0, 0.0
        v, w, stamp = self.candidate
        if not 0 <= now - stamp <= self.timeout:
            self.revoke('candidate timeout')
            return 0.0, 0.0
        self.reason = 'running'
        return v, w


def stanley_turn(path, pose, speed, measured_speed, p):
    x, y, yaw = pose
    distance = p['control_point']
    _, cte, heading, _, _ = project(path, x + distance*math.cos(yaw),
                                    y + distance*math.sin(yaw))
    heading_error = wrap(heading-yaw)
    steer = heading_error + math.atan2(p['stanley_gain']*cte,
                                       abs(measured_speed)+p['soft_speed'])
    steer = max(-p['max_steer'], min(p['max_steer'], steer))
    w_track = speed * math.tan(steer) / distance
    return w_track, cte, heading_error


def stanley_only(path, pose, speed, measured_speed, p):
    w_track, cte, heading_error = stanley_turn(path, pose, speed, measured_speed, p)
    v, w = limit_drive(speed, w_track, p['track'], p['wheel_limit'], True)
    return v, w, {'cte': cte, 'heading_error': heading_error,
                   'w_stanley': w_track, 'w_apf': 0.0}


def apf_stanley(path, pose, speed, measured_speed, angles, ranges, p):
    w_track, cte, heading_error = stanley_turn(path, pose, speed, measured_speed, p)
    repulsion = 0.0
    nearest = p['slow_distance']
    for angle, r in zip(angles, ranges):
        if math.isfinite(r):
            nearest = min(nearest, r)
            if r < p['influence_distance']:
                strength = min(p['max_repulsion'],
                               (1/max(r, 0.05)-1/p['influence_distance']) / max(r*r, 0.01))
                # Break head-on symmetry with one configured bypass direction.
                side = -math.sin(angle) if abs(angle) > 0.03 else -p['bypass_side']
                repulsion += strength * side / max(1, len(ranges))
    factor = max(0.0, min(1.0, (nearest-p['stop_distance']) /
                          (p['slow_distance']-p['stop_distance'])))
    v = speed * factor
    w = w_track * factor + p['apf_gain'] * repulsion
    v, w = limit_drive(v, w, p['track'], p['wheel_limit'], True)
    return v, w, {'cte': cte, 'heading_error': heading_error,
                   'w_stanley': w_track, 'w_apf': p['apf_gain']*repulsion}
