#!/usr/bin/env python3
"""
wheel_joint_state_publisher — turn the Pico's /wheel_speeds into /joint_states
so robot_state_publisher animates the wheels in RViz.

The RP2040 firmware publishes:
    /wheel_speeds   std_msgs/Float32MultiArray   [FL, RL, RR, FR] in rad/s @ 20 Hz

This node integrates each wheel's angular velocity into a joint position and
republishes as sensor_msgs/JointState with the URDF joint names.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# Order must match the firmware's wheel array: FL=0, RL=1, RR=2, FR=3
JOINT_NAMES = [
    'FL_wheel_joint',
    'RL_wheel_joint',
    'RR_wheel_joint',
    'FR_wheel_joint',
]


class WheelJointStatePublisher(Node):
    def __init__(self):
        super().__init__('wheel_joint_state_publisher')

        self.position = [0.0] * len(JOINT_NAMES)
        self.velocity = [0.0] * len(JOINT_NAMES)
        self.last_time = None

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(
            Float32MultiArray, 'wheel_speeds', self.on_wheel_speeds, 10)

        self.get_logger().info(
            'wheel_joint_state_publisher up: /wheel_speeds -> /joint_states')

    def on_wheel_speeds(self, msg: Float32MultiArray):
        now = self.get_clock().now()

        # Integrate angular velocity -> position using wall time between messages.
        if self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
        else:
            dt = 0.0
        self.last_time = now

        n = min(len(msg.data), len(JOINT_NAMES))
        for i in range(n):
            w = float(msg.data[i])
            self.velocity[i] = w
            self.position[i] += w * dt

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = JOINT_NAMES
        js.position = self.position
        js.velocity = self.velocity
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = WheelJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
