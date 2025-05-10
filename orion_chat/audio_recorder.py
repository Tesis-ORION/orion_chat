#!/usr/bin/env python3
import uuid
import sounddevice as sd
import numpy as np
from loguru import logger
import rclpy
from rclpy.node import Node
from audio_messages.msg import AudioInfo, AudioData
from std_msgs.msg import Bool
from datetime import datetime

class AudioRecorder(Node):
    def __init__(self):
        super().__init__('audio_recorder')
        # Publishers
        self.pub_web  = self.create_publisher(Bool,      'web',        10)
        self.pub_info = self.create_publisher(AudioInfo, 'audio_info', 10)
        self.pub_data = self.create_publisher(AudioData, 'audio_data', 10)

        # Parámetros de audio
        self.CHANNELS    = 1
        self.SAMPLE_RATE = 44100

        # — Aquí ajustas la tolerancia: —
        self.silence_threshold    = 0.02   # umbral para considerar silencio
        self.silence_max_duration = 1.0    # s de silencio antes de publicar
        # — fin de parámetros —

        # Tamaño de cada bloque
        self.chunk_duration = 0.1
        self.chunk_size     = int(self.SAMPLE_RATE * self.chunk_duration)

        # Estado interno
        self.speech_buffer  = []
        self.silence_chunks = 0
        self.is_speaking    = False

        self.device = sd.default.device[0]
        logger.info(f"Dispositivo de audio: {self.device}")

    def sound_cb(self, indata, frames, time, status):
        if status:
            logger.warning(status)

        level = np.max(np.abs(indata))
        if level > self.silence_threshold:
            # voz activa
            self.speech_buffer.append(indata.copy())
            self.silence_chunks = 0
            self.is_speaking    = True
        else:
            # silencio
            if self.is_speaking:
                self.silence_chunks += 1
                if self.silence_chunks * self.chunk_duration >= self.silence_max_duration:
                    self._end_of_utterance()

    def _end_of_utterance(self):
        # concatenar y resetear
        data = np.concatenate(self.speech_buffer, axis=0)
        self.speech_buffer.clear()
        self.is_speaking    = False
        self.silence_chunks = 0

        # 1) Publicar false en /web
        web_msg = Bool(data=False)
        self.pub_web.publish(web_msg)

        # 2) Publicar AudioInfo
        uid = str(uuid.uuid4())
        info = AudioInfo(
            num_channels=self.CHANNELS,
            sample_rate=self.SAMPLE_RATE,
            subtype='float32',
            uuid=uid
        )
        self.pub_info.publish(info)

        # 3) Publicar AudioData
        msg = AudioData()
        msg.data = data.astype(np.float32)
        self.pub_data.publish(msg)

        self.get_logger().info(
            f"► Publicado utterance uuid={uid}, muestras={data.shape[0]}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AudioRecorder()

    stream = sd.InputStream(
        device=node.device,
        channels=node.CHANNELS,
        samplerate=node.SAMPLE_RATE,
        blocksize=node.chunk_size,
        callback=node.sound_cb
    )
    stream.start()
    node.get_logger().info("Recorder activo: detectando silencio y publicando false…")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop()
        stream.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
