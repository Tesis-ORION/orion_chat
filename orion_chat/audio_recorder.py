#!/usr/bin/env python3
import sys
import tempfile
import threading
import queue
import wave
import os
import requests
import rclpy
from rclpy.node import Node

# Configuración
FLASK_URL = "http://localhost:5000/upload"
SAMPLE_RATE = 16000       # Frecuencia de muestreo en Hz
CHANNELS = 1              # Mono

class AudioRecorderNode(Node):
    def __init__(self):
        super().__init__('audio_recorder')
        self.get_logger().info("AudioRecorderNode inicializado.")

    def run(self):
        # Bucle principal: graba y envía continuamente
        while rclpy.ok():
            # 1) Espera ENTER para comenzar
            self.get_logger().info("Presiona ENTER para comenzar a grabar...")
            try:
                input()
            except EOFError:
                break

            # 2) Preparar grabación
            record_queue = queue.Queue()
            stop_event = threading.Event()

            def callback(indata, frames, time, status):
                if status:
                    print(f"[!] {status}", file=sys.stderr)
                record_queue.put(indata.copy())

            # 3) Iniciar grabación en hilo
            recorder = threading.Thread(
                target=self._record_audio,
                args=(record_queue, stop_event, callback),
                daemon=True
            )
            recorder.start()

            # 4) Detener con ENTER
            self.get_logger().info("Grabando... presiona ENTER nuevamente para detener.")
            input()
            stop_event.set()
            recorder.join()
            self.get_logger().info("Grabación finalizada.")

            # 5) Recolectar frames
            frames = []
            while not record_queue.empty():
                frames.append(record_queue.get())

            # 6) Guardar WAV temporal
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                wav_path = tmp.name
            with wave.open(wav_path, 'wb') as wf:
                wf.setnchannels(CHANNELS)
                wf.setsampwidth(2)  # 16 bits = 2 bytes
                wf.setframerate(SAMPLE_RATE)
                wf.writeframes(b''.join(frames))
            self.get_logger().info(f"Audio guardado en {wav_path}")

            # 7) Enviar al servidor Flask en segundo plano (silencioso)
            threading.Thread(target=self._send_audio, args=(wav_path,), daemon=True).start()

            # 8) Esperar ENTER para iniciar siguiente grabación
            self.get_logger().info("Presiona ENTER para iniciar la siguiente grabación.")
            try:
                input()
            except EOFError:
                break

        self.get_logger().info("AudioRecorderNode finalizado.")

    def _send_audio(self, wav_path):
        try:
            with open(wav_path, 'rb') as f:
                files = {'file': (os.path.basename(wav_path), f, 'audio/wav')}
                resp = requests.post(FLASK_URL, files=files)
                resp.raise_for_status()
            # Eliminar archivo temporal tras envío exitoso
            os.remove(wav_path)
        except Exception as e:
            self.get_logger().error(f"Error al enviar audio o eliminar archivo: {e}")

    def _record_audio(self, record_queue, stop_event, callback):
        import sounddevice as sd
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype='int16',
            callback=callback
        ):
            while not stop_event.is_set():
                sd.sleep(100)


def main(args=None):
    rclpy.init(args=args)
    node = AudioRecorderNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción manual, terminando.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
