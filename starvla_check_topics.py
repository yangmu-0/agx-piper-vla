#!/usr/bin/env python3
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

IMAGE_TOPICS = [
    "/top/color/image_raw",
    "/left/color/image_raw",
    "/right/color/image_raw",
]
STATE_TOPICS = [
    ("/joint_states_left", JointState),
    ("/joint_states_right", JointState),
    ("/end_pose_left", Pose),
    ("/end_pose_right", Pose),
]

class TopicReadyNode(Node):
    def __init__(self):
        super().__init__("starvla_topic_ready_check")
        self.seen = {topic: False for topic in IMAGE_TOPICS}
        self.seen.update({topic: False for topic, _ in STATE_TOPICS})
        for topic in IMAGE_TOPICS:
            self.create_subscription(Image, topic, self._make_cb(topic), qos_profile_sensor_data)
        state_qos = QoSProfile(depth=10)
        for topic, msg_type in STATE_TOPICS:
            self.create_subscription(msg_type, topic, self._make_cb(topic), state_qos)

    def _make_cb(self, topic):
        def cb(_msg):
            if not self.seen[topic]:
                print(f"OK {topic} message received", flush=True)
            self.seen[topic] = True
        return cb

    def missing(self):
        return [topic for topic, ok in self.seen.items() if not ok]


def main():
    timeout_s = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    rclpy.init()
    node = TopicReadyNode()
    deadline = time.time() + timeout_s
    try:
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.missing():
                print("StarVLA ROS topics are ready.", flush=True)
                return 0
        print("FAIL missing ROS topics: " + ", ".join(node.missing()), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    raise SystemExit(main())
