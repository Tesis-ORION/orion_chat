import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import speech_recognition as sr
import pygame
import time
import tempfile
from ament_index_python.packages import get_package_share_directory
import difflib  

class OrionSTTNode(Node):
    def __init__(self):
        super().__init__("orion_stt")
        self.publisher = self.create_publisher(String, "orion_input", 10)
        # Inicializamos el recognizer solo para capturar audio (sin STT remoto)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        pygame.mixer.init()
        # Definir la palabra de activación objetivo y el umbral de similitud
        self.target_wake_word = "hola orion"
        self.wake_threshold = 0.8  # Umbral ajustable
        # Cargar el modelo Whisper local para español
        self.whisper_model = whisper.load_model("small") 

    def play_activation_sound(self):
        try:
            pkg_share = get_package_share_directory("orion_chat")
            sound_path = os.path.join(pkg_share, "sounds", "activation_sound.mp3")
            pygame.mixer.music.load(sound_path)
            pygame.mixer.music.play()
            time.sleep(0.2)
        except Exception as e:
            self.get_logger().error(f"Error al reproducir beep: {e}")

    def is_wake_word_detected(self, text):
        text = text.lower().strip()
        # Calcula el ratio de similitud entre la transcripción y la palabra de activación
        ratio = difflib.SequenceMatcher(None, text, self.target_wake_word).ratio()
        return ratio >= self.wake_threshold

    def record_audio_to_file(self, source):
        # Ajuste del ruido ambiental
        self.recognizer.adjust_for_ambient_noise(source, duration=0.1)
        # Ajustar pause_threshold para que espere un silencio mayor antes de cortar la grabación
        self.recognizer.pause_threshold = 1.0  # Valor aumentado para permitir pausas sin cortar
        self.recognizer.non_speaking_duration = 0.5
        self.get_logger().info("Grabando audio (esperando a que finalice la frase)...")
        self.play_activation_sound()
        # No se utiliza phrase_time_limit para que se grabe hasta que se detecte un silencio prolongado
        audio = self.recognizer.listen(source)
        wav_data = audio.get_wav_data()
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
        with open(temp_file.name, "wb") as f:
            f.write(wav_data)
        return temp_file.name

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
        with self.microphone as source:
            self.get_logger().info("Ajustando ruido ambiental para detectar 'Hola ORION'...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            self.get_logger().info("¡Listo para escuchar la palabra de activación!")
            while True:
                try:
                    audio = self.recognizer.listen(source)
                    wav_data = audio.get_wav_data()
                    with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as temp_file:
                        temp_file.write(wav_data)
                        temp_file_path = temp_file.name
                    result = self.whisper_model.transcribe(temp_file_path, language="es")
                    text = result["text"].strip().lower()
                    if self.is_wake_word_detected(text):
                        self.get_logger().info("¡Palabra de activación detectada! Preparando para comando...")
                        return
                except Exception as e:
                    self.get_logger().error(f"Error en STT (wake word): {e}")

    def listen_and_publish(self):
        with self.microphone as source:
            self.get_logger().info("Ajustando ruido ambiental para escuchar comando...")
            self.recognizer.adjust_for_ambient_noise(source, duration=0.2)
            self.get_logger().info("¡Listo para escuchar comando! (Se grabará hasta detectar silencio prolongado)")
            try:
                audio_file = self.record_audio_to_file(source)
                result = self.whisper_model.transcribe(audio_file, language="es")
                text = result["text"].strip()
                msg = String()
                msg.data = text
                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error al transcribir el comando: {e}")

    def run(self):
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
