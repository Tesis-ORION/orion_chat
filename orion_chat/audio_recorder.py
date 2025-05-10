#!/usr/bin/env python3
import uuid
import numpy as np
import sounddevice as sd
from loguru import logger
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from audio_messages.msg import AudioInfo, AudioData

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

        # Tolerancia de silencio
        self.silence_threshold    = 0.02   # nivel mínimo para considerar “voz”
        self.silence_max_duration = 1.0    # s de silencio antes de enviar :contentReference[oaicite:3]{index=3}

        # Tamaño de bloque
        self.chunk_duration = 0.1
        self.chunk_size     = int(self.SAMPLE_RATE * self.chunk_duration)  # callbacks cada 0.1 s :contentReference[oaicite:4]{index=4}

        # Estado interno
        self.speech_buffer  = []
        self.silence_chunks = 0
        self.is_speaking    = False

        # Registrar dispositivo default y dispositivos de entrada
        default_dev = sd.default.device
        logger.info(f"Dispositivo por defecto sounddevice: {default_dev}")   # muestra índice o par (in, out) :contentReference[oaicite:5]{index=5}
        logger.debug(f"Dispositivos input disponibles:\n{sd.query_devices(kind='input')}")  # listado completo :contentReference[oaicite:6]{index=6}

    def sound_cb(self, indata, frames, time, status):
        # Log para verificar recepción de datos
        logger.debug(f"sound_cb: frames={frames}, status={status}, shape={indata.shape}")  # confirmar callback :contentReference[oaicite:7]{index=7}
        if status:
            logger.warning(status)

        level = np.max(np.abs(indata))
        if level > self.silence_threshold:
            # Hay voz: acumulamos
            self.speech_buffer.append(indata.copy())
            self.silence_chunks = 0
            self.is_speaking    = True
        else:
            # Silencio: si veníamos hablando, medir duración
            if self.is_speaking:
                self.silence_chunks += 1
                if self.silence_chunks * self.chunk_duration >= self.silence_max_duration:
                    self._end_of_utterance()

    def _end_of_utterance(self):
        # Concatenar y resetear buffer
        data = np.concatenate(self.speech_buffer, axis=0)
        self.speech_buffer.clear()
        self.is_speaking = False
        self.silence_chunks = 0

        # 1) Publicar false en /web
        self.pub_web.publish(Bool(data=False))

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

    # Stream usando dispositivo default (no pasamos `device`) :contentReference[oaicite:8]{index=8}
    stream = sd.InputStream(
        channels=node.CHANNELS,
        samplerate=node.SAMPLE_RATE,
        blocksize=node.chunk_size,
        callback=node.sound_cb
    )
    stream.start()
    node.get_logger().info("Recorder activo: detectando silencio…")

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
