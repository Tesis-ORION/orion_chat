#!/usr/bin/env python3
import io
import re
import os
import subprocess
import tempfile
import multiprocessing as mp
import pygame

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

VOICE = "es-VE-SebastianNeural"

def tts_worker(text_queue: mp.Queue, audio_queue: mp.Queue):
    """Proceso que convierte texto a audio MP3 usando edge-tts y lo pone en audio_queue."""
    while True:
        text = text_queue.get()
        if text is None:
            break

        # Crea un archivo temporal para el MP3
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Llama al CLI de edge-tts
            subprocess.run([
                "edge-tts",
                "--voice", VOICE,
                "--text", text,
                "--write-media", tmp_path
            ], check=True)

            # Lee el MP3 en memoria
            with open(tmp_path, "rb") as f:
                audio_queue.put(f.read())

        except subprocess.CalledProcessError as e:
            # Si falla la síntesis, puedes loguearlo aquí
            continue

        finally:
            # Limpia el archivo
            try:
                os.remove(tmp_path)
            except OSError:
                pass

def playback_worker(audio_queue: mp.Queue):
    """Proceso que reproduce los audios encolados."""
    pygame.mixer.pre_init(44100, -16, 2, 1024)
    pygame.init()
    while True:
        data = audio_queue.get()
        if data is None:
            break
        try:
            pygame.event.pump()
            pygame.mixer.music.load(io.BytesIO(data), "mp3")
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.event.pump()
                pygame.time.wait(10)
        except Exception:
            continue

class OrionTTSNode(Node):
    def __init__(self):
        super().__init__("orion_tts")
        self.subscription = self.create_subscription(
            String, "orion_response", self.listener_callback, 10
        )
        self.get_logger().info("Nodo TTS optimizado iniciado.")

        self.text_queue = mp.Queue()
        self.audio_queue = mp.Queue()

        # Arranca dos procesos para producción
        self.tts_procs = []
        for _ in range(2):
            p = mp.Process(
                target=tts_worker,
                args=(self.text_queue, self.audio_queue),
                daemon=True
            )
            p.start()
            self.tts_procs.append(p)

        # Proceso de reproducción
        self.playback_proc = mp.Process(
            target=playback_worker,
            args=(self.audio_queue,),
            daemon=True
        )
        self.playback_proc.start()

    def listener_callback(self, msg: String):
        # Limpia prefijo [ORION] y encola
        text = re.sub(r"^\[?ORION[:\]\s]+", "", msg.data).strip()
        self.get_logger().info(f"Encolando para TTS: {text}")
        self.text_queue.put(text)

    def destroy_node(self):
        for _ in self.tts_procs:
            self.text_queue.put(None)
        self.audio_queue.put(None)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OrionTTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
