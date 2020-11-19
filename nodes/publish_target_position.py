#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from threading import Lock
from geometry_msgs.msg import Vector3
from mim_msgs.srv import TurtlesimTarget


class TurtlesimTargetPublisher(Node):
    def __init__(self):
        super().__init__("turtlesim_target_position", namespace="turtle1")

        self.lock = Lock()
        self.position = [6.0, 6.0, 0.0]
        self.topic_name = "target"
        self.service_name = "set_target_position"
        self.timer_period = 0.1
        self.target = Vector3()

        self.service = self.create_service(
            TurtlesimTarget, self.service_name, self._set_target_position_callback
        )

        self.publisher = self.create_publisher(Vector3, self.topic_name, 10)

        self.timer = self.create_timer(self.timer_period, self._timer_callback)

    def _set_target_position_callback(self, request, response):
        with self.lock:
            self.position[:] = [
                request.position.x,
                request.position.y,
                request.position.z,
            ]

        return response

    def _timer_callback(self):
        with self.lock:
            self.target.x, self.target.y, self.target.z = self.position
        self.publisher.publish(self.target)


def main(args=None):
    rclpy.init(args=args)
    turtlesim_target_publisher = TurtlesimTargetPublisher()
    rclpy.spin(turtlesim_target_publisher)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
