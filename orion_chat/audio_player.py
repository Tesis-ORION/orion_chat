#!/usr/bin/env python3
import sys, sounddevice as sd, numpy as np
from queue import Queue
from loguru import logger
import time
import rclpy
from rclpy.node import Node
from audio_messages.msg import AudioInfo, AudioData

class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        self.info_queue = Queue()
        self.data_queue = Queue()
        self.subscription_info = self.create_subscription(
            AudioInfo, 'audio_info', self.info_cb, 10)
        self.subscription_data = self.create_subscription(
            AudioData, 'audio_data', self.data_cb, 10)

        # espera breve para arrancar bien
        time.sleep(2)
        logger.info("AudioPlayer listo")

        # arrancar hilo de reproducción
        self.create_timer(0.1, self.play_cb)

    def info_cb(self, msg: AudioInfo):
        self.info_queue.put(msg)

    def data_cb(self, msg: AudioData):
        self.data_queue.put(msg.data)

    def play_cb(self):
        if not self.info_queue.empty() and not self.data_queue.empty():
            info = self.info_queue.get()
            data = self.data_queue.get()
            # reproducir buffer
            sd.play(data, samplerate=info.sample_rate)
            sd.wait()  # bloquea hasta terminar
            self.get_logger().info(f"Reproducido uuid={info.uuid}")

def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
