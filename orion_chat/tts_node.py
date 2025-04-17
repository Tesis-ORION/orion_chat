import threading
import queue
import concurrent.futures
import io
from gtts import gTTS
import pygame
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Colas para el procesamiento TTS
text_queue = queue.Queue()    # Textos pendientes de conversión
audio_queue = queue.Queue()   # Audios (bytes) convertidos

def tts_conversion_worker(text):
    
    # Convierte el texto a audio usando gTTS y retorna los datos de audio (bytes).
    
    try:
        print("[TTS Conversion] Procesando texto:", text)
        tts = gTTS(text=text, lang="es", slow=False)
        audio_stream = io.BytesIO()
        tts.write_to_fp(audio_stream)
        audio_stream.seek(0)
        return audio_stream.read()
    except Exception as e:
        print("Error en conversión TTS:", e)
        return None

def text_handler_worker():
    
    # Extrae textos de la cola, los procesa a audio de forma concurrente y coloca el audio resultante en la cola.
    
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        while True:
            text = text_queue.get()
            if text is None:
                break
            future = executor.submit(tts_conversion_worker, text)
            audio_data = future.result()
            if audio_data:
                audio_queue.put(audio_data)
            text_queue.task_done()

def audio_playback_worker():
    
    # Extrae audios convertidos de la cola y los reproduce secuencialmente.
    
    pygame.mixer.init()
    while True:
        audio_data = audio_queue.get()
        if audio_data is None:
            break
        try:
            audio_stream = io.BytesIO(audio_data)
            pygame.mixer.music.load(audio_stream, "mp3")
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        except Exception as e:
            print("Error en reproducción de audio:", e)
        audio_queue.task_done()

class OrionTTSNode(Node):
    def __init__(self):
        super().__init__("orion_tts")
        self.subscription = self.create_subscription(
            String,
            "orion_response",
            self.listener_callback,
            10
        )
        self.get_logger().info("Nodo TTS iniciado, esperando mensajes en 'orion_response'.")

    def clean_orion_prefix(self, text):
        return re.sub(r'(\[ORION:\s*\]|\[ORION\]:)', '', text).strip()

    def listener_callback(self, msg):
        text = self.clean_orion_prefix(msg.data)
        self.get_logger().info(f"Encolando TTS: {text}")
        text_queue.put(text)

def main():
    rclpy.init()
    node = OrionTTSNode()

    playback_thread = threading.Thread(target=audio_playback_worker, daemon=True)
    playback_thread.start()

    conversion_thread = threading.Thread(target=text_handler_worker, daemon=True)
    conversion_thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


