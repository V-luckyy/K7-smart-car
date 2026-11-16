import math
from collections import deque
from k7_control.core import finite, project, wrap


class LeaderTrail:
    """Leader-relative slot history in the follower odom frame (left positive)."""
    def __init__(self, capacity=256, spacing=0.03):
        self.points = deque(maxlen=capacity)
        self.spacing = spacing
        self.session = None
        self.last_leader = None

    @staticmethod
    def slot(leader, yaw, gap, lateral_offset):
        c, s = math.cos(yaw), math.sin(yaw)
        return (leader[0]-gap*c-lateral_offset*s,
                leader[1]-gap*s+lateral_offset*c)

    def append(self, point, session, yaw=0.0, gap=0.0, lateral_offset=0.0):
        if not finite(*point, yaw, gap, lateral_offset):
            raise ValueError('non-finite leader position')
        if self.session is not None and session != self.session:
            raise ValueError('leader restarted; realignment required')
        target = self.slot(point, yaw, gap, lateral_offset)
        if ((self.last_leader is not None and math.dist(point, self.last_leader) > 0.5) or
                (self.points and math.dist(target, self.points[-1]) > 0.5)):
            raise ValueError('leader pose discontinuity')
        self.session, self.last_leader = session, point
        # Keep a live endpoint without growing history for every DDS sample.
        if self.points and math.dist(target, self.points[-1]) < 1e-6:
            return
        if len(self.points) < 2 or math.dist(target, self.points[-2]) >= self.spacing:
            self.points.append(target)
        else:
            self.points[-1] = target

    def seed(self, own, leader, leader_yaw, session, gap, lateral_offset,
             lateral_tolerance, yaw_tolerance):
        c, s = math.cos(leader_yaw), math.sin(leader_yaw)
        expected = self.slot(leader, leader_yaw, gap, lateral_offset)
        if (not finite(*own, *leader, leader_yaw, gap, lateral_offset) or
                math.dist(own[:2], expected) > lateral_tolerance or
                abs(wrap(leader_yaw-own[2])) > yaw_tolerance):
            raise ValueError('initial pose does not match requested formation offset')
        self.points.clear()
        self.points.append(expected)
        self.session, self.last_leader = session, leader

    def reference(self, own, gap, leader_speed, max_speed, gain):
        points = list(self.points)
        _, _, _, own_arc, total = project(points, *own)
        target_arc = max(0.0, total-gap)
        if target_arc <= own_arc + 0.002:
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
