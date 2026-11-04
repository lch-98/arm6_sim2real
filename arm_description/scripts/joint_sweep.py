#!/usr/bin/env python3
"""관절마다 위상을 다르게 준 사인파로 /joint_states를 발행하는 데모 노드 (RViz 동작 확인용)"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# (관절 이름, 중심값 rad, 진폭 rad, 주기 s) — 진폭은 관절 한계 안쪽으로 잡음
SWEEP = [
    ("joint1", 0.0, 1.2, 12.0),
    ("joint2", 0.3, 0.5, 8.0),
    ("joint3", -0.5, 0.7, 7.0),
    ("joint4", 0.0, 1.5, 9.0),
    ("joint5", 0.6, 0.6, 5.0),
    ("joint6", 0.0, 2.0, 6.0),
    ("finger_left_joint", 0.015, 0.015, 3.0),
    ("finger_right_joint", 0.015, 0.015, 3.0),  # mimic 관절도 같이 발행
]


class JointSweep(Node):
    def __init__(self):
        super().__init__("joint_sweep")
        self.prefix = self.declare_parameter("prefix", "").value
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.t0 = self.get_clock().now()
        self.create_timer(0.02, self.tick)  # 50 Hz

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        msg = JointState()
        msg.header.stamp = now.to_msg()
        for name, c, a, T in SWEEP:
            msg.name.append(self.prefix + name)
            msg.position.append(c + a * math.sin(2.0 * math.pi * t / T))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
