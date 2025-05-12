#!/usr/bin/env python3
import uuid
import sounddevice as sd
import numpy as np
from loguru import logger
import rclpy
from rclpy.node import Node
from audio_messages.msg import AudioInfo, AudioData
from std_msgs.msg import Bool

class AudioRecorder(Node):
    def __init__(self):
        super().__init__('audio_recorder')
        # ParÃ¡metro para device_index
        self.CHANNELS = 1

        # 1) Listar y filtrar dispositivos
        devices = sd.query_devices()
        valid = [
            i for i, d in enumerate(devices)
            if d['max_input_channels'] >= self.CHANNELS
            and 'default' not in d['name'].lower()
        ]
        if not valid:
            raise RuntimeError("No se encontró un dispositivo de entrada válido")
        auto_idx = valid[0]

        # 2) Ajustar sample rate al nativo
        self.SAMPLE_RATE = int(devices[auto_idx]['default_samplerate'])

        # 3) Selección final
        self.device = auto_idx
        logger.info(f"Usando dispositivo {self.device}: {devices[self.device]['name']}")

        # Publishers
        self.pub_web  = self.create_publisher(Bool,      'web',        10)
        self.pub_info = self.create_publisher(AudioInfo, 'audio_info', 10)
        self.pub_data = self.create_publisher(AudioData, 'audio_data', 10)

        # Tolerancia de silencio
        self.silence_threshold = 0.02
        self.silence_max_duration = 1.0

        # Tamaños de bloque
        self.chunk_duration = 0.1
        self.chunk_size     = int(self.SAMPLE_RATE * self.chunk_duration)

        # Estado interno
        self.speech_buffer  = []
        self.silence_chunks = 0
        self.is_speaking    = False

    def sound_cb(self, indata, frames, time, status):
        if status:
            logger.warning(status)
        level = np.max(np.abs(indata))
        if level > self.silence_threshold:
            self.speech_buffer.append(indata.copy()); self.silence_chunks = 0; self.is_speaking = True
        else:
            if self.is_speaking:
                self.silence_chunks += 1
                if self.silence_chunks * self.chunk_duration >= self.silence_max_duration:
                    self._end_of_utterance()

    def _end_of_utterance(self):
        data = np.concatenate(self.speech_buffer, axis=0)
        self.speech_buffer.clear(); self.is_speaking = False; self.silence_chunks = 0

        self.pub_web.publish(Bool(data=False))
        uid = str(uuid.uuid4())
        self.pub_info.publish(AudioInfo(
            num_channels=self.CHANNELS,
            sample_rate=self.SAMPLE_RATE,
            subtype='float32',
            uuid=uid
        ))
        msg = AudioData(); msg.data = data.astype(np.float32)
        self.pub_data.publish(msg)
        self.get_logger().info(f"Publicado utterance uuid={uid}, muestras={data.shape[0]}")

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
    node.get_logger().info("Recorder activo")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop(); stream.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
