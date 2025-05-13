#!/usr/bin/env python3
import os
import tempfile
import uuid

import numpy as np
import rclpy
import whisper
import difflib
import soundfile as sf
import librosa

from rclpy.node import Node
from std_msgs.msg import Bool, String
from audio_messages.msg import AudioInfo, AudioData


class OrionSTTNode(Node):
    def __init__(self):
        super().__init__('orion_stt')

        # Publicador final de texto
        self.pub_text = self.create_publisher(String, 'orion_input', 10)

        # Suscriptor al flag web
        self.web_flag = False
        self.create_subscription(Bool, 'web', self.web_cb, 10)

        # Cargar modelo Whisper “medium” en CPU
        self.get_logger().info("Cargando modelo Whisper small en modo CPU...")
        self.whisper_model = whisper.load_model("small", device="cpu")
        self.get_logger().info("Modelo Whisper cargado correctamente.")

        # Subscriptores de audio
        self._pending_info = None
        self._pending_frames = []
        self.create_subscription(AudioInfo, 'audio_info', self.info_cb, 10)
        self.create_subscription(AudioData, 'audio_data', self.data_cb, 10)

    def web_cb(self, msg: Bool):
        self.web_flag = msg.data

    def info_cb(self, msg: AudioInfo):
        # Guardar metadatos
        self._pending_info = msg

    def data_cb(self, msg: AudioData):
        # Acumular fragmentos raw a 44.1 kHz float32
        self._pending_frames.append(np.array(msg.data, dtype=np.float32))

        # Solo procesar cuando tengamos metadatos
        if not self._pending_info:
            return

        # Construir y guardar WAV temporal resampleado
        wav_path = self._build_temp_wav(self._pending_info, self._pending_frames)

        try:
            # Transcribir forzando español
            result = self.whisper_model.transcribe(
                wav_path,
                language='es'
            )
            transcript = result.get("text", "").strip()
            self.get_logger().info(f"Transcripción: {transcript!r}")

            # Decidir envío
            send_ok = False
            if self.web_flag:
                send_ok = True
            else:
                txt_low = transcript.lower()
                if 'orion' in txt_low:
                    send_ok = True
                else:
                    for word in txt_low.split():
                        if difflib.SequenceMatcher(None, word, 'orion').ratio() >= 0.6:
                            send_ok = True
                            break

            if send_ok and transcript:
                msg_out = String(data=transcript)
                self.pub_text.publish(msg_out)
                self.get_logger().info(f"Publicado en 'orion_input': {transcript!r}")

        except Exception as e:
            self.get_logger().error(f"Error en Whisper: {e}")

        finally:
            # Limpiar
            if os.path.exists(wav_path):
                os.remove(wav_path)
            self._pending_info = None
            self._pending_frames.clear()

    def _build_temp_wav(self, info: AudioInfo, frames_list):
        """
        Concatena float32@44.1k → resample a 16k → salva float32 WAV.
        Devuelve ruta del WAV.
        """
        # 1) Concat raw
        raw = np.concatenate(frames_list, axis=0)

        # 2) Resample a 16 kHz
        audio_16k = librosa.resample(raw,
                                     orig_sr=info.sample_rate,
                                     target_sr=16000)

        # 3) Guardar WAV float32
        tmp = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
        tmp_path = tmp.name
        tmp.close()

        sf.write(tmp_path,
                 audio_16k,
                 16000,
                 subtype='FLOAT')
        self.get_logger().debug(f"WAV temporal resampleado escrito en {tmp_path}")
        return tmp_path


def main(args=None):
    rclpy.init(args=args)
    node = OrionSTTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción manual, cerrando nodo.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
