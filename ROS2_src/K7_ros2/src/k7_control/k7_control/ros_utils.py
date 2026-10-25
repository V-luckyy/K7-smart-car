import math
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor


def yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1) > 0.01:
        return math.nan
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def permission(text):
    try:
        value = bytes.fromhex(text)
        return value if len(value) == 16 else bytes(16)
    except (TypeError, ValueError):
        return bytes(16)


def parameters(node, defaults):
    # Nodes cache settings at startup; reject changes that would only alter the parameter service.
    descriptor = ParameterDescriptor(read_only=True)
    return {name: node.declare_parameter(name, value, descriptor).value for name, value in defaults.items()}


def diagnostic(node, name, values, level=0, message=''):
    result = DiagnosticArray()
    result.header.stamp = node.get_clock().now().to_msg()
    result.status = [DiagnosticStatus(name=name, level=level, message=message,
                                     values=[KeyValue(key=str(k), value=str(v))
                                             for k, v in values.items()])]
    return result


def fields(msg):
    return {v.key: v.value for status in msg.status for v in status.values}


def stamp_fresh(node, stamp, timeout):
    age = node.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9
    return -0.05 <= age <= timeout
