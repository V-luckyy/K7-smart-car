import math
from collections import deque
from k7_control.core import finite, project, wrap


class LeaderTrail:
    """Bounded path history in the follower odom frame."""
    def __init__(self, capacity=256, spacing=0.03):
        self.points = deque(maxlen=capacity)
        self.spacing = spacing
        self.session = None

    def append(self, point, session):
        if not finite(*point):
            raise ValueError('non-finite leader position')
        if self.session is not None and session != self.session:
            raise ValueError('leader restarted; realignment required')
        self.session = session
        if not self.points or math.dist(point, self.points[-1]) >= self.spacing:
            if self.points and math.dist(point, self.points[-1]) > 0.5:
                raise ValueError('leader pose discontinuity')
            self.points.append(point)

    def seed(self, own, leader, leader_yaw, session, lateral_tolerance, yaw_tolerance):
        dx, dy = leader[0]-own[0], leader[1]-own[1]
        c, s = math.cos(leader_yaw), math.sin(leader_yaw)
        if (not finite(*own, *leader, leader_yaw) or c*dx+s*dy <= 0 or
                abs(-s*dx+c*dy) > lateral_tolerance or
                abs(wrap(leader_yaw-own[2])) > yaw_tolerance):
            raise ValueError('initial formation must be aligned with follower behind leader')
        self.points.clear()
        self.points.extend((own[:2], leader))
        self.session = session

    def reference(self, own, gap, leader_speed, max_speed, gain):
        points = list(self.points)
        _, _, _, own_arc, total = project(points, *own)
        target_arc = max(0.0, total-gap)
        if target_arc <= own_arc + 0.03:
            return points, 0.0, total-own_arc-gap
        selected = [points[0]]
        arc = 0.0
        for a, b in zip(points, points[1:]):
            length = math.dist(a, b)
            if arc+length >= target_arc:
                t = (target_arc-arc)/max(length, 1e-6)
                selected.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])))
                break
            selected.append(b)
            arc += length
        error = total-own_arc-gap
        speed = max(0.0, min(max_speed, leader_speed+gain*error))
        return selected, speed, error
