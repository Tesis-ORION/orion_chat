import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import re
from collections import deque
from ament_index_python.packages import get_package_share_directory
import time

class OrionChatNode(Node):
    def __init__(self):
        super().__init__("orion_chat")
        self.subscription = self.create_subscription(
            String,
            "orion_input",
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, "orion_response", 10)
        # Memoria rotacional para los últimos 10 intercambios (chat)
        self.chat_history = deque(maxlen=10)
        
        # Valores predeterminados
        self.external_document = (
            "Información predeterminada: No se cargaron documentos externos."
        )
        self.system_prompt = (
            "Eres ORION, un robot avanzado con una personalidad inteligente, amigable y con un toque de humor. "
            "Respondes de forma precisa y basada en datos verificados."
        )
        self.load_rag_resources()
        self.last_message = None
        self.is_processing = False

    def load_rag_resources(self):
        try:
            pkg_share = get_package_share_directory("orion_chat")
            resources_dir = os.path.join(pkg_share, "resource")
        except Exception as e:
            self.get_logger().error(f"Error al obtener el directorio share: {e}")
            return

        external_docs = []
        system_prompt_override = None

        if os.path.isdir(resources_dir):
            for filename in os.listdir(resources_dir):
                if filename.endswith(".json"):
                    filepath = os.path.join(resources_dir, filename)
                    try:
                        with open(filepath, 'r', encoding='utf-8') as f:
                            data = json.load(f)
                        if "system_prompt" in data:
                            system_prompt_override = data["system_prompt"]
                        if "external_document" in data:
                            external_docs.append(data["external_document"])
                        self.get_logger().info(f"Recurso cargado: {filename}")
                    except Exception as e:
                        self.get_logger().error(f"Error cargando {filename}: {e}")
            if external_docs:
                self.external_document = "\n".join(external_docs)
            if system_prompt_override:
                self.system_prompt = system_prompt_override
        else:
            self.get_logger().warn("No se encontró la carpeta 'resources' en el paquete.")

    def listener_callback(self, msg):
        # Evitar mensajes duplicados o si ya se está procesando
        if msg.data == self.last_message or self.is_processing:
            return
        self.is_processing = True
        self.last_message = msg.data

        user_message = msg.data
        self.get_logger().info(f"Recibido en ROS: {user_message}")

        # Actualizar historial
        self.chat_history.append(f"Usuario: {user_message}")

        # Generar prompt aumentado
        augmented_prompt = self.generate_augmented_prompt(user_message)
        # Llamada en streaming al LLM
        self.get_logger().info("Llamando al LLM en modo streaming...")
        self.process_stream(augmented_prompt)

        self.is_processing = False

    def generate_augmented_prompt(self, user_message):
        history_text = "\n".join(list(self.chat_history))
        prompt = (
            f"{self.system_prompt}\n\n"
            f"Contexto reciente:\n{history_text}\n\n"
            f"Documentos adicionales:\n{self.external_document}\n\n"
            f"Pregunta del usuario: {user_message}\n\n"
            f"Respuesta:"
        )
        return prompt

    def process_stream(self, augmented_prompt):
        system_prompt = (
            "Eres ORION, un robot avanzado diseñado para la interacción humano-robot en educación e investigación. "
            "Tu personalidad es inteligente, amigable y con un toque de humor robótico. "
            "Respondes de forma precisa, lógica y con curiosidad científica. "
            "Tus respuestas siempre comienzan con '[ORION]:' y son concisas."
        )
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": augmented_prompt}
        ]
        payload = {
            "model": "llama3.2:3b",
            "messages": messages,
            "stream": True
        }
        url = "http://localhost:11434/api/chat"
        try:
            response = requests.post(url, json=payload, stream=True)
            response.raise_for_status()
        except Exception as e:
            self.get_logger().error(f"Error al comunicarse con Ollama: {e}")
            return

        buffer = ""
        punctuation_marks = {'.', ',', '!', '?'}
        self.get_logger().info("Respuesta del LLM (streaming):")
        token_found = False
        for line in response.iter_lines(decode_unicode=True):
            if line:
                token_found = True
                try:
                    data = json.loads(line)
                    token = data.get("token")
                    if token is None:
                        message = data.get("message")
                        if message and isinstance(message, dict):
                            token = message.get("content", "")
                    print(token, end='', flush=True)
                    buffer += token
                    if token and token[-1] in punctuation_marks:
                        phrase = buffer.strip()
                        self.get_logger().info(f"\n[Publicando a TTS]: {phrase}")
                        ros_msg = String()
                        ros_msg.data = phrase
                        self.publisher.publish(ros_msg)
                        self.chat_history.append(f"ORION: {phrase}")
                        buffer = ""
                except Exception as e:
                    self.get_logger().error(f"Error procesando stream: {e}")
        if not token_found:
            self.get_logger().warn("No se recibió token del LLM.")
        if buffer:
            phrase = buffer.strip()
            self.get_logger().info(f"\n[Publicando a TTS]: {phrase}")
            ros_msg = String()
            ros_msg.data = phrase
            self.publisher.publish(ros_msg)
            self.chat_history.append(f"ORION: {phrase}")

def main(args=None):
    rclpy.init(args=args)
    node = OrionChatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
