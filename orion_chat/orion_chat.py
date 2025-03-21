#!/usr/bin/env python3
import os
import json
import re
import time
import threading
from collections import deque
import unicodedata
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import requests

def remove_accents(input_str):
    """Elimina acentos y caracteres diacríticos de la cadena."""
    nfkd_form = unicodedata.normalize('NFD', input_str)
    return "".join([c for c in nfkd_form if not unicodedata.combining(c)])

def convert_number(text):
    """Convierte el texto a un número (soporta dígitos o palabras en español)."""
    try:
        return float(text)
    except ValueError:
        try:
            from text_to_num import text2num
            return float(text2num(text, "es"))
        except Exception:
            return None

# --- Capa de Movimiento basada en LLM ---
class MovementLayer:
    def __init__(self, config, cmd_vel_publisher, node_logger):
        self.cmd_vel_pub = cmd_vel_publisher
        self.config = config  # Contiene el RAG de movimiento
        self.logger = node_logger
        self.stop_timer = None

    def process_movement_command(self, message: str):
        """
        Utiliza el LLM para generar un comando de movimiento en formato JSON y lo ejecuta.
        Se espera que el LLM devuelva algo como:
        {"linear_x": 0.5, "angular_z": 0.0, "duration": 5.0}
        """
        system_prompt = self.config.get("system_prompt", "")
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": message}
        ]
        payload = {
            "model": self.config.get("model", "llama3.2:3b"),
            "messages": messages,
            "stream": False
        }
        url = "http://localhost:11434/api/chat"
        try:
            response = requests.post(url, json=payload)
            response.raise_for_status()
            data = response.json()
            # Se intenta extraer el contenido generado por el LLM
            content = ""
            if "message" in data and isinstance(data["message"], dict):
                content = data["message"].get("content", "")
            elif "response" in data:
                content = data["response"]
            content = content.strip()
            self.logger.info(f"Respuesta del LLM para movimiento: {content}")
            # Parsear el contenido como JSON
            command = json.loads(content)
            linear_x = float(command.get("linear_x", self.config.get("default_linear_speed", 0.5)))
            angular_z = float(command.get("angular_z", self.config.get("default_angular_speed", 0.5)))
            duration = float(command.get("duration", self.config.get("default_duration", 5.0)))
        except Exception as e:
            self.logger.error(f"Error procesando comando de movimiento: {e}")
            return None, f"Error procesando comando de movimiento: {e}"
        
        # Publicar el mensaje Twist
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        # Programar la detención tras 'duration' segundos (si es mayor que 0)
        if duration > 0:
            def stop_robot():
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                self.cmd_vel_pub.publish(stop_twist)
            self.stop_timer = threading.Timer(duration, stop_robot)
            self.stop_timer.daemon = True
            self.stop_timer.start()
        return command, f"Ejecutando comando: linear_x={linear_x}, angular_z={angular_z}, duration={duration}."

# --- Nodo principal que integra Conversación y Movimiento ---
class OrionChatMovementNode(Node):
    def __init__(self):
        super().__init__("orion_chat_movement")
        self.subscription = self.create_subscription(String, "orion_input", self.listener_callback, 10)
        self.response_pub = self.create_publisher(String, "orion_response", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.chat_history = deque(maxlen=10)
        self.last_message = None
        self.is_processing = False
        self.current_mode = "conversation"  # Modo por defecto

        # Cargar archivos de configuración (RAG) desde la carpeta resource
        self.conversation_config = self.load_resource("conversation_config.json")
        self.movement_config = self.load_resource("movement_config.json")

        self.movement_layer = MovementLayer(self.movement_config, self.cmd_vel_pub, self.get_logger())

        # Frases clave para cambiar de modo (se evaluarán por coincidencia exacta)
        self.activate_movement_phrases = ["modo movimiento", "modo de movimiento", "muevete", "muévete", "quiero que te muevas", "hora de moverse"]
        self.activate_conversation_phrases = ["modo conversación", "modo conversacion", "hablar contigo", "quiero hablar"]

        self.get_logger().info("Nodo iniciado en modo conversación.")

    def load_resource(self, filename):
        """Carga un archivo JSON desde la carpeta resource del paquete."""
        try:
            pkg_share = get_package_share_directory("orion_chat")
            resources_dir = os.path.join(pkg_share, "resource")
        except Exception as e:
            self.get_logger().error(f"Error al obtener el directorio share: {e}")
            return {}
        resource_data = {}
        if os.path.isdir(resources_dir):
            for file in os.listdir(resources_dir):
                if file == filename:
                    filepath = os.path.join(resources_dir, file)
                    try:
                        with open(filepath, 'r', encoding='utf-8') as f:
                            resource_data = json.load(f)
                        self.get_logger().info(f"Recurso cargado: {file}")
                    except Exception as e:
                        self.get_logger().error(f"Error cargando {file}: {e}")
        else:
            self.get_logger().warn("No se encontró la carpeta 'resource' en el paquete.")
        return resource_data

    def listener_callback(self, msg: String):
        if msg.data == self.last_message or self.is_processing:
            return
        self.is_processing = True
        self.last_message = msg.data
        user_message = msg.data
        self.get_logger().info(f"Recibido: {user_message}")
        self.chat_history.append(f"Usuario: {user_message}")
        user_message_lower = user_message.lower().strip()

        # Verificar si se solicita cambio de modo (evaluación exacta)
        if user_message_lower in self.activate_movement_phrases:
            if self.current_mode != "movement":
                self.current_mode = "movement"
                response = "Cambiando a modo movimiento. Ahora la IA creará los comandos de movimiento automáticamente."
            else:
                response = "Ya estoy en modo movimiento."
            self.publish_response(response)
            self.is_processing = False
            return

        if user_message_lower in self.activate_conversation_phrases:
            if self.current_mode != "conversation":
                self.current_mode = "conversation"
                response = "Cambiando a modo conversación. Podemos seguir dialogando normalmente."
            else:
                response = "Ya estoy en modo conversación."
            self.publish_response(response)
            self.is_processing = False
            return

        # Procesar según el modo actual
        if self.current_mode == "movement":
            # En modo movimiento, la IA genera el JSON con los comandos y se publican
            command, response = self.movement_layer.process_movement_command(user_message)
            self.publish_response(response)
        else:
            # Modo conversación: genera prompt aumentado y usa streaming para la respuesta
            augmented_prompt = self.generate_augmented_prompt(user_message)
            self.process_stream(augmented_prompt)
        self.is_processing = False

    def generate_augmented_prompt(self, user_message):
        history_text = "\n".join(list(self.chat_history))
        system_prompt = self.conversation_config.get("system_prompt", "")
        external_document = self.conversation_config.get("external_document", "")
        prompt = (
            f"{system_prompt}\n\n"
            f"Contexto reciente:\n{history_text}\n\n"
            f"Documentos adicionales:\n{external_document}\n\n"
            f"Pregunta del usuario: {user_message}\n\n"
            f"Responde de forma natural y conversacional."
        )
        return prompt

    def process_stream(self, augmented_prompt):
        """
        Envía la solicitud a la API LLM en modo streaming y publica fragmentos en TTS
        al detectar signos de puntuación.
        """
        system_prompt = self.conversation_config.get("system_prompt", "")
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": augmented_prompt}
        ]
        payload = {
            "model": self.conversation_config.get("model", "llama3.2:3b"),
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
                        self.publish_response(phrase)
                        self.chat_history.append(f"ORION: {phrase}")
                        buffer = ""
                except Exception as e:
                    self.get_logger().error(f"Error procesando stream: {e}")
        if not token_found:
            self.get_logger().warn("No se recibió token del LLM.")
        if buffer:
            phrase = buffer.strip()
            self.publish_response(phrase)
            self.chat_history.append(f"ORION: {phrase}")

    def publish_response(self, text: str):
        """Publica la respuesta en el tópico de salida (TTS)."""
        msg = String()
        msg.data = text
        self.response_pub.publish(msg)
        self.get_logger().info(f"Respuesta: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = OrionChatMovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
