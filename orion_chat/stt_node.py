import os
import sys
import json
import time
import difflib
import pyaudio
import pygame
from vosk import Model, KaldiRecognizer
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OrionSTTNode(Node):
    def __init__(self):
        super().__init__('orion_stt')
        self.publisher_ = self.create_publisher(String, 'orion_input', 10)
        # Define la palabra de activación y su umbral de similitud
        self.wake_word = "orion"
        self.wake_threshold = 0.8
        pygame.mixer.init()
        pkg_share = get_package_share_directory('orion_chat')
        model_path = os.path.join(pkg_share, 'model', 'vosk-model-small-es-0.42')
        if not os.path.exists(model_path):
            self.get_logger().error(f"El modelo no se encontró en {model_path}.")
            sys.exit(1)
        self.model = Model(model_path)
        self.get_logger().info("Modelo Vosk cargado correctamente.")

        # Inicializa PyAudio y abre el stream de audio
        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(format=pyaudio.paInt16, channels=1,rate=16000, input=True, frames_per_buffer=4000)
        self.stream.start_stream()
        self.get_logger().info("Stream de audio iniciado.")

    def is_wake_word(self, text):
        # Convertir a minúsculas y separar en palabras
        text = text.lower().strip()
        words = text.split()
        for word in words:
            ratio = difflib.SequenceMatcher(None, word, self.wake_word).ratio()
            if ratio >= self.wake_threshold:
                return True
        return False
    
    def play_activation_sound(self):
        try:
            pkg_share = get_package_share_directory("orion_chat")
            sound_path = os.path.join(pkg_share, "sounds", "activation_sound.mp3")
            pygame.mixer.music.load(sound_path)
            pygame.mixer.music.play()
            time.sleep(0.2)
        except Exception as e:
            self.get_logger().error(f"Error al reproducir beep: {e}")

    def listen_for_wake_word(self):
        self.get_logger().info("Esperando la palabra de activación...")
        recognizer = KaldiRecognizer(self.model, 16000)
        while rclpy.ok():
            data = self.stream.read(4000, exception_on_overflow=False)
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                text = result.get("text", "")
                self.get_logger().info(f"Reconocido (wake word): '{text}'")
                if self.is_wake_word(text):
                    self.get_logger().info("¡Palabra de activación detectada!")
                    return True
            time.sleep(0.1)
        return False

    def listen_for_command(self, duration=5):
        self.play_activation_sound()
        self.get_logger().info("Grabando comando...")
        # Reinicia el recognizer para capturar el comando (reset de estado)
        recognizer = KaldiRecognizer(self.model, 16000)
        start_time = time.time()
        command_text = ""
        while time.time() - start_time < duration:
            data = self.stream.read(4000, exception_on_overflow=False)
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                command_text += " " + result.get("text", "")
            time.sleep(0.1)
        final_result = json.loads(recognizer.FinalResult())
        command_text += " " + final_result.get("text", "")
        return command_text.strip()

    def run(self):
        while rclpy.ok():
            # Esperar la palabra de activación
            if self.listen_for_wake_word():
                # Una vez activada, escuchar el comando
                command = self.listen_for_command(duration=5)
                if command:
                    self.get_logger().info(f"Comando reconocido: '{command}'")
                    msg = String()
                    msg.data = command
                    self.publisher_.publish(msg)
                else:
                    self.get_logger().info("No se reconoció ningún comando.")
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = OrionSTTNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Terminando nodo por interrupción manual.")
    finally:
        node.stream.stop_stream()
        node.stream.close()
        node.pa.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

