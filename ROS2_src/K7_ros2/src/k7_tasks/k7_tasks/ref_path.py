import math


def outward_spiral():
    # Gentle spiral: inner radius 0.5 m (above the ~0.24 m steering limit),
    # so the leader can hold its target speed without turning-rate saturation.
    inner_radius, outer_radius, turns, count = 0.5, 1.5, 2, 500
    end_angle = turns * 2 * math.pi
    growth = (outer_radius - inner_radius) / end_angle
    points = [(0.0, 0.0, 0.0)]  # straight lead-in from the origin
    for i in range(count):
        angle = end_angle * i / (count - 1)
        radius = inner_radius + growth * angle
        x, y = radius * math.cos(angle), radius * math.sin(angle)
        dx = growth * (math.cos(angle) - angle * math.sin(angle))
        dy = growth * (math.sin(angle) + angle * math.cos(angle))
        points.append((x, y, math.atan2(dy, dx)))
    return points


def main(args=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from k7_msgs.action import FollowPath

    rclpy.init(args=args)
    node = Node('ref_path')
    try:
        client = ActionClient(node, FollowPath, 'task/follow_path')
        path = Path()
        namespace = node.get_namespace().strip('/')
        path.header.frame_id = namespace + '/odom' if namespace else 'odom'
        for x, y, yaw in outward_spiral():
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(pose)

        goal = FollowPath.Goal(path=path, target_speed=0.10,
                               position_tolerance=0.10, yaw_tolerance=0.15)
        client.wait_for_server()
        accepted = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, accepted)
        handle = accepted.result()
        if not handle or not handle.accepted:
            node.get_logger().error('Reference path goal rejected')
            return
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result)
        node.get_logger().info(f'Reference path finished: {result.result().result.reason}')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
