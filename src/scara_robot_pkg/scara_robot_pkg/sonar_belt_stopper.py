#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from conveyorbelt_msgs.srv import ConveyorBeltControl

DETECT_THRESHOLD = 0.049  # metres — 5cm sensor height, 4.5mm chip raises floor to ~0.0455m
COOLDOWN_SEC = 10.0      # ignore re-triggers for this long after an event


class SonarBeltStopper(Node):
    def __init__(self):
        super().__init__('sonar_belt_stopper')

        self._belt1_client = self.create_client(ConveyorBeltControl, '/CONVEYORPOWER')
        self._belt2_client = self.create_client(ConveyorBeltControl, '/belt2/CONVEYORPOWER')

        self._belt1_ready_pub = self.create_publisher(Bool, 'belt1/object_ready', 10)
        self._belt2_ready_pub = self.create_publisher(Bool, 'belt2/object_ready', 10)

        self._belt1_cooldown_until: float = 0.0
        self._belt2_cooldown_until: float = 0.0

        self.create_subscription(
            Range, 'belt1/sonar',
            lambda msg: self._check(msg, self._belt1_client, self._belt1_ready_pub,
                                    '_belt1_cooldown_until', 'belt1'),
            10,
        )
        self.create_subscription(
            Range, 'belt2/sonar',
            lambda msg: self._check(msg, self._belt2_client, self._belt2_ready_pub,
                                    '_belt2_cooldown_until', 'belt2'),
            10,
        )

        self.get_logger().info(
            'SonarBeltStopper started. Threshold: %.3f m, cooldown: %.1f s'
            % (DETECT_THRESHOLD, COOLDOWN_SEC)
        )

    def _check(self, msg: Range, client, pub, cooldown_attr: str, label: str) -> None:
        now = time.monotonic()
        if now < getattr(self, cooldown_attr):
            return  # still in cooldown — ignore
        if msg.range < DETECT_THRESHOLD:
            self.get_logger().info(
                '%s sonar: object at %.3f m — stopping belt and publishing event.' % (label, msg.range)
            )
            req = ConveyorBeltControl.Request()
            req.power = 0.0
            client.call_async(req)
            pub.publish(Bool(data=True))
            setattr(self, cooldown_attr, now + COOLDOWN_SEC)


def main():
    rclpy.init()
    node = SonarBeltStopper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
