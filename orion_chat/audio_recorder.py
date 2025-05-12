#!/usr/bin/env python3
import uuid
import queue
import struct
import webrtcvad
import sounddevice as sd
import numpy as np
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

        # Audio config
        default_dev      = sd.default.device[0]
        self.sample_rate = 48000
        self.channels    = 1

        # VAD config
        self.vad = webrtcvad.Vad(2)  # Modo agresividad 0–3 
        self.frame_ms     = 30       # Duración de frame en ms (10,20 o 30) 
        self.frame_size   = int(self.sample_rate * self.frame_ms / 1000)
        self.byte_per_frame = self.frame_size * 2  # 16-bit PCM -> 2 bytes

        # State machine thresholds
        self.max_silence_frames = int(0.5 * 1000 / self.frame_ms)  # 0.5 s de silencio
        self.min_speech_frames  = int(0.1 * 1000 / self.frame_ms)  # 0.1 s de habla antes de START 

        logger.info(f"Device: {default_dev} @ {self.sample_rate}Hz")

    def run(self):
        q = queue.Queue()
        # Callback para InputStream: envía bloques raw PCM al queue
        def callback(indata, frames, time, status):
            if status:
                logger.warning(f"PortAudio status: {status}")
            q.put(bytes(indata))


        stream = sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=self.frame_size,
            dtype='int16',
            channels=self.channels,
            callback=callback
        )
        stream.start()
        self.get_logger().info("Recorder activo con WebRTC VAD")

        try:
            ring_buffer = []
            speech_frames = []
            in_speech = False
            silence_count = 0
            speech_count = 0

            while rclpy.ok():
                frame = q.get()  # Bloque de 16-bit PCM
                is_speech = self.vad.is_speech(frame, self.sample_rate)

                if not in_speech:
                    # Buscamos el inicio de la frase
                    ring_buffer.append(frame)
                    if len(ring_buffer) > self.min_speech_frames:
                        ring_buffer.pop(0)
                    if is_speech:
                        speech_count += 1
                        if speech_count > self.min_speech_frames:
                            # Utterance START
                            uid = str(uuid.uuid4())
                            self.pub_web.publish(Bool(data=True))
                            info = AudioInfo(
                                num_channels=self.channels,
                                sample_rate=self.sample_rate,
                                subtype='int16',
                                uuid=uid
                            )
                            self.pub_info.publish(info)
                            self.get_logger().info(f"Utterance START uuid={uid}")
                            # Iniciar buffer de voz con pre-roll
                            speech_frames = ring_buffer.copy()
                            in_speech = True
                            silence_count = 0
                    else:
                        speech_count = 0
                else:
                    # Dentro de una frase, acumulamos el frame
                    speech_frames.append(frame)
                    if not is_speech:
                        silence_count += 1
                        if silence_count > self.max_silence_frames:
                            # Utterance END
                            full_data = b''.join(speech_frames)
                            # Convertir a float32 para AudioData
                            pcm = np.frombuffer(full_data, dtype=np.int16).astype(np.float32) / 32768.0
                            msg = AudioData()
                            msg.data = pcm.flatten().tolist()
                            self.pub_data.publish(msg)
                            self.pub_web.publish(Bool(data=False))
                            self.get_logger().info(
                                f"Utterance END uuid={uid}, frames={len(speech_frames)}"
                            )
                            # Reset
                            in_speech = False
                            speech_count = 0
                            ring_buffer.clear()
                    else:
                        silence_count = 0

                # Girar ROS en hilo aparte no bloqueante
                rclpy.spin_once(self, timeout_sec=0)

        finally:
            stream.stop()
            stream.close()

def main(args=None):
    rclpy.init(args=args)
    node = AudioRecorder()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
