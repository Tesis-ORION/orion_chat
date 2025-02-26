import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import pygame
import io

class OrionTTSNode(Node):
    def __init__(self):
        super().__init__("orion_tts")
        self.subscription = self.create_subscription(
            String,
            "orion_response",
            self.listener_callback,
            10
        )
        pygame.mixer.init()

    def clean_orion_prefix(self, text):
        # Reemplaza tanto "[ORION: ]" (con o sin espacios internos) como "[ORION]:" por cadena vacía.
        return re.sub(r'(\[ORION:\s*\]|\[ORION\]:)', '', text).strip()

    def listener_callback(self, msg):
        text = self.clean_orion_prefix(msg.data)  # Limpiar la respuesta antes de hablar
        self.get_logger().info(f"Reproduciendo voz: {text}")
        self.speak(text)

    def speak(self, text):
        try:
            # Convertir texto a voz sin guardar archivo
            tts = gTTS(text=text, lang="es", slow=False)
            audio_stream = io.BytesIO()
            tts.write_to_fp(audio_stream)
            audio_stream.seek(0)

            # Inicializar pygame mixer y reproducir desde memoria
            pygame.mixer.init()
            pygame.mixer.music.load(audio_stream, "mp3")
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                pass  # Esperar hasta que termine de hablar

        except Exception as e:
            self.get_logger().error(f"Error en TTS: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OrionTTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


