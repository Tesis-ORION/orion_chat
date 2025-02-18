import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import speech_recognition as sr
import pygame
import time
import tempfile
import os
import sys




class OrionSTTNode(Node):
    def __init__(self):
        super().__init__("orion_stt")
        self.publisher = self.create_publisher(String, "orion_input", 10)
        # Inicializamos el recognizer solo para capturar audio (sin STT remoto)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        pygame.mixer.init()
        self.wake_words = [
            "hola orion", "hola orión", "hola oriom", "hola oriam", "ola orion",
            "hola horion", "hola horyon", "hola oryan", "hola orían", "hola oriana", "hola, orión.", "hola orian"
        ]
        # Cargar el modelo Whisper local para español (por ejemplo, "small")
        self.whisper_model = whisper.load_model("small") 

    def play_activation_sound(self):
        try:
            pygame.mixer.music.load("activation_sound.mp3")
            pygame.mixer.music.play()
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"Error al reproducir beep: {e}")

    def is_wake_word_detected(self, text):
        text = text.lower().strip()
        for wake_word in self.wake_words:
            if wake_word in text:
                return True
        return False

    def record_audio_to_file(self, source, timeout=None):
        # Capturamos audio usando la fuente ya abierta
        self.recognizer.adjust_for_ambient_noise(source, duration=0.1)
        self.get_logger().info("Grabando audio...")
        self.play_activation_sound()
        audio = self.recognizer.listen(source, phrase_time_limit=timeout)
        wav_data = audio.get_wav_data()
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
        with open(temp_file.name, "wb") as f:
            f.write(wav_data)
        return temp_file.name

    def listen_for_wake_word(self):
        """Escucha hasta detectar la frase de activación."""
        with self.microphone as source:
            self.get_logger().info("Esperando 'Hola ORION'...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            while True:
                try:
                    audio = self.recognizer.listen(source)
                    wav_data = audio.get_wav_data()
                    # Guardamos temporalmente para transcribir
                    with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as temp_file:
                        temp_file.write(wav_data)
                        temp_file_path = temp_file.name
                    # Transcribe con Whisper, forzando el idioma español
                    result = self.whisper_model.transcribe(temp_file_path, language="es")
                    text = result["text"].strip().lower()
                    if self.is_wake_word_detected(text):
                        self.get_logger().info("¡Activado! Preparando para comando...")
                        return
                except Exception as e:
                    self.get_logger().error(f"Error en STT (wake word): {e}")

    def listen_and_publish(self):
        """Escucha el comando del usuario y lo publica."""
        with self.microphone as source:
            self.get_logger().info("Escuchando comando (máx. 10 s)...")
            # Reducimos el ajuste de ruido para que sea más rápido
            self.recognizer.adjust_for_ambient_noise(source, duration=0.2)
            # Señalamos que el sistema ya está listo
            try:
                audio_file = self.record_audio_to_file(source, timeout=10)
                result = self.whisper_model.transcribe(audio_file, language="es")
                text = result["text"].strip()
                self.get_logger().info(f"Comando: {text}")
                msg = String()
                msg.data = text
                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error al transcribir el comando: {e}")

    def run(self):
        """Bucle principal del nodo STT."""
        while rclpy.ok():
            self.listen_for_wake_word()
            self.listen_and_publish()

def main(args=None):
    rclpy.init(args=args)
    node = OrionSTTNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
