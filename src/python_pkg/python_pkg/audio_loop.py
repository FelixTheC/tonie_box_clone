#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time

import rclpy
import ujson
import subprocess
from typing import Optional
from rclpy.node import Node

from example_interfaces.msg import String, Bool
from rcl_interfaces.srv import GetParameters
from python_pkg.volume_percent import get_init_volume_percent


# play music/the-second.mp3
# 3611660238
# play /home/felix/music/the-second.mp3 trim '=80.5'  jump to pos
# amixer -c 0 cget numid=1  // read current volume information
# amixer -c 0 cset numid=1 63% // set audio to specific volume percent
DEFAULT_TRACK = "/home/felix/music/Wavecont-Inspiring-Full.mp3"


class AudioLoopNode(Node):

    def __init__(self):
        super().__init__("audio_loop")
        self.card_id: Optional[str]
        self.music_started = False
        self.pid = None
        self.logger_info("Audio loop started")
        self.volume_percent = get_init_volume_percent()
        self.start_time = 0
        self.running_seconds = 0
        self.track_paused = False
        self.track = ""

        self.subscriber_mfrc = self.create_subscription(String,
                                                        "mfrc_connection",
                                                        self.callback_mfrc_connection,
                                                        qos_profile=1)

        self.subscriber_volume_down = self.create_subscription(Bool,
                                                               "volume_down",
                                                               self.callback_volume_down,
                                                               qos_profile=1)

        self.subscriber_volume_up = self.create_subscription(Bool,
                                                             "volume_up",
                                                             self.callback_volume_up,
                                                             qos_profile=1)

        self.subscriber_pause = self.create_subscription(Bool,
                                                         "pause_track",
                                                         self.callback_pause_track,
                                                         qos_profile=1)

    def logger_info(self, text: str):
        self.get_logger().info(text)

    def logger_error(self, text: str):
        self.get_logger().error(text)

    def set_audio_volume(self):
        if 0 <= self.volume_percent <= 100:
            subprocess.Popen(["amixer", "-c", "0", "cset", "numid=1", f"{self.volume_percent}%"])

    def callback_pause_track(self, msg: Bool):
        if msg.data:
            if self.track_paused:
                process = subprocess.Popen(["play", self.track, "trim", f"={self.running_seconds}"])
                self.pid = process.pid
                self.music_started = True
                self.track_paused = False
                self.start_time = time.time()
            else:
                self.running_seconds += time.time() - self.start_time
                self.stop_music()
                self.track_paused = True
                self.music_started = True

    def callback_volume_down(self, msg: Bool):
        if msg.data:
            if 0 <= self.volume_percent - 5:
                self.volume_percent -= 5
                self.set_audio_volume()

    def callback_volume_up(self, msg: Bool):
        # amixer -D pulse sset Master 5%+
        if msg.data:
            if self.volume_percent + 5 <= 100:
                self.volume_percent += 5
                self.set_audio_volume()

    def get_track(self):
        client = self.create_client(GetParameters, "get_music_path")
        if not client.service_is_ready():
            self.track = DEFAULT_TRACK
        future = client.call_async(GetParameters.Request(names=[self.card_id]))
        future.add_done_callback(self.callback_get_track)

    def callback_get_track(self, future):
        try:
            track = future.result().values[0].string_value
            self.track = track if track else DEFAULT_TRACK
            self.logger_info(f'Received track {self.track}')
        except Exception as err:
            self.logger_error(f'{err}')

    def play_music(self):
        if self.card_id and self.music_started:
            return
        elif self.card_id and self.track and not self.music_started:
            self.logger_info(f'start track {self.track} for {self.card_id = }')
            process = subprocess.Popen(["play", self.track])
            self.start_time = time.time()
            self.running_seconds = 0
            self.pid = process.pid
            self.logger_info(f'{self.pid = }')
            self.music_started = True
        elif self.card_id is None and self.music_started:
            print("Music stopped")
            self.stop_music()

    def stop_music(self):
        if self.pid:
            subprocess.Popen(["kill", str(self.pid)])
        self.music_started = False
        print("Music stopped. New card detected.")

    def callback_mfrc_connection(self, msg: String):
        data = ujson.loads(msg.data)
        if data["status"] == "ok":
            card_id = data["card_id"]
            if card_id != self.card_id:
                self.stop_music()
            self.card_id = card_id
            if not self.track:
                self.get_track()
        else:
            self.card_id = None
            self.track_paused = False
            self.track = ""
        self.play_music()


def main(args=None):
    rclpy.init(args=args)
    node = AudioLoopNode()
    try:
        rclpy.spin(node)  # will hold/keep alive node
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
