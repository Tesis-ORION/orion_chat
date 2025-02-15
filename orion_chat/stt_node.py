import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import pygame
import time

class OrionSTTNode(Node):
    def __init__(self):
        super().__init__("orion_stt")
        self.publisher = self.create_publisher(String, "orion_input", 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Inicializar pygame para reproducir el beep
        pygame.mixer.init()

        # Lista de posibles variaciones de "Hola ORION"
        self.wake_words = [
            "hola orion", "hola orión", "hola oriom", "hola oriam", "ola orion",
            "hola horion", "hola horyon", "hola oryan", "hola orían"
        ]

    def play_activation_sound(self):
        """ Reproduce un beep cuando detecta 'Hola ORION' """
        try:
            pygame.mixer.music.load("activation_sound.mp3")  # Archivo de sonido (lo generaremos abajo)
            pygame.mixer.music.play()
            time.sleep(0.5)  # Esperar 500ms para que el sonido se escuche bien
        except Exception as e:
            self.get_logger().error(f"Error al reproducir beep: {e}")

    def is_wake_word_detected(self, text):
        """ Verifica si el texto detectado es similar a 'Hola ORION' """
        text = text.lower().strip()
        for wake_word in self.wake_words:
            if wake_word in text:
                return True
        return False

    def listen_for_wake_word(self):
        """ Escucha hasta detectar 'Hola ORION' (o variaciones similares) y activa el reconocimiento de comandos """
        with self.microphone as source:
            self.get_logger().info("Esperando la frase 'Hola ORION'...")

            self.recognizer.adjust_for_ambient_noise(source, duration=1)

            while True:  # Bucle infinito hasta detectar 'Hola ORION'
                try:
                    audio = self.recognizer.listen(source)  # No hay timeout aquí
                    text = self.recognizer.recognize_google(audio, language="es-ES").lower()
                    self.get_logger().info(f"Detectado: {text}")  # Mostrar qué escuchó

                    if self.is_wake_word_detected(text):
                        self.get_logger().info("¡Activado! Ahora escucha el comando...")
                        self.play_activation_sound()  # Emitir beep
                        return  # Salir de la función y pasar a escuchar el comando

                except sr.UnknownValueError:
                    self.get_logger().warn("No se entendió el audio.")
                except sr.RequestError as e:
                    self.get_logger().error(f"Error con el reconocimiento de voz: {e}")

    def listen_and_publish(self):
        """ Escucha el comando después de detectar 'Hola ORION' y lo publica en ROS2 (con timeout) """

        with self.microphone as source:
            self.get_logger().info("Escuchando comando (máx. 10 segundos)...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)

            try:
                audio = self.recognizer.listen(source, timeout=10)  # Aquí sí hay timeout
                text = self.recognizer.recognize_google(audio, language="es-ES")

                self.get_logger().info(f"Comando detectado: {text}")

                # Publicar en ROS2
                msg = String()
                msg.data = text
                self.publisher.publish(msg)

            except sr.UnknownValueError:
                self.get_logger().warn("No se entendió el comando, intenta de nuevo.")
            except sr.WaitTimeoutError:
                self.get_logger().warn("Tiempo de espera agotado, vuelve a decir 'Hola ORION'.")
            except sr.RequestError as e:
                self.get_logger().error(f"Error con el reconocimiento de voz: {e}")

    def run(self):
        """ Bucle principal del nodo: espera 'Hola ORION' y luego escucha el comando """
        while rclpy.ok():
            self.listen_for_wake_word()  # Espera hasta que alguien diga "Hola ORION"
            self.listen_and_publish()  # Luego escucha el comando y lo envía

def main(args=None):
    rclpy.init(args=args)
    node = OrionSTTNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
