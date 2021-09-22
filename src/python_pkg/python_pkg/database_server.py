#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@created: 21.09.21
@author: felix
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue

SIMPLE_DATABASE = {
    "3611660238": "/home/felix/music/the-second.mp3",
    "61341241": "/home/felix/music/the-third.wav",
}

DEFAULT_TRACK = "/home/felix/music/"


class DatabaseServerNode(Node):

    def __init__(self):
        super().__init__("database_server")
        self.server_ = self.create_service(GetParameters, "get_music_path", self.callback_get_file_path)
        self.logger_info("Hello ROS2")

    def callback_get_file_path(self, request, response):
        self.logger_info(f'received: {request = }')
        track_path = SIMPLE_DATABASE.get(str(request.names[0]), DEFAULT_TRACK)
        value = ParameterValue()
        value.string_value = track_path
        response.values = [value, ]
        return response

    def logger_info(self, text: str):
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = DatabaseServerNode()
    try:
        rclpy.spin(node)  # will hold/keep alive node
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
