#!/usr/bin/env python3
import os
import json
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
from text_to_num import text2num
import difflib

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
            return float(text2num(text, "es"))
        except Exception:
            return None

class MovementLayer:
    def __init__(self, config, cmd_vel_publisher, node_logger):
        self.cmd_vel_pub = cmd_vel_publisher
        self.config = config  # Contiene el RAG de movimiento
        self.logger = node_logger
        self.stop_timer = None

    def process_movement_command(self, message: str):
        """
        Llama al LLM para obtener comando(s) de movimiento en formato JSON.
        Se espera que la respuesta sea un objeto JSON simple o un arreglo JSON.
        Inmediatamente lanza la ejecución de los comandos en un hilo.
        """
        system_prompt = self.config.get("system_prompt", "")
        messages_payload = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": message}
        ]
        payload = {
            "model": "qwen2.5:3b",
            "messages": messages_payload,
            "stream": False
        }
        url = "http://localhost:11434/api/chat"
        try:
            response = requests.post(url, json=payload)
            response.raise_for_status()
            data = response.json()
            content = ""
            if "message" in data and isinstance(data["message"], dict):
                content = data["message"].get("content", "")
            elif "response" in data:
                content = data["response"]
            content = content.strip()
            self.logger.info(f"Respuesta del LLM para movimiento: {content}")
            command = json.loads(content)
        except Exception as e:
            err_msg = f"Error procesando comando de movimiento: {e}"
            self.logger.error(err_msg)
            return None, err_msg

        # Lanzar la ejecución de comandos en un hilo para no bloquear
        threading.Thread(target=self.execute_commands, args=(command,)).start()
        # Retornamos el comando y un mensaje interno de que se inició la ejecución
        return command, "Comando recibido y en ejecución."

    def execute_commands(self, command):
        """
        Ejecuta el/los comando(s) recibidos (JSON) de forma secuencial.
        Cada comando se ejecuta publicando el mensaje de Twist y se programa
        la detención mediante un Timer sin bloquear el hilo principal.
        """
        def execute_single_command(cmd):
            try:
                if "linear_x" in cmd:
                    linear_x = float(cmd["linear_x"])
                else:
                    # Si es giro y no se define linear_x, usar 0.
                    if "angular_z" in cmd and float(cmd["angular_z"]) != 0:
                        linear_x = 0.0
                    else:
                        linear_x = self.config.get("default_linear_speed", 0.5)
                if "angular_z" in cmd:
                    angular_z = float(cmd["angular_z"])
                else:
                    if "linear_x" in cmd and float(cmd["linear_x"]) != 0:
                        angular_z = 0.0
                    else:
                        angular_z = self.config.get("default_angular_speed", 0.5)
                duration = float(cmd.get("duration", self.config.get("default_duration", 5.0)))
            except Exception as e:
                self.logger.error(f"Error procesando los parámetros del comando: {e}")
                return

            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
            self.logger.info(f"Ejecutando comando: linear_x={linear_x}, angular_z={angular_z}, duration={duration}")

            # Programar la detención tras 'duration' segundos si es mayor que 0
            if duration > 0:
                def stop_robot():
                    stop_twist = Twist()
                    stop_twist.linear.x = 0.0
                    stop_twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(stop_twist)
                    self.logger.info("Comando finalizado, robot detenido.")
                self.stop_timer = threading.Timer(duration, stop_robot)
                self.stop_timer.daemon = True
                self.stop_timer.start()
        # Si se trata de varios comandos
        if isinstance(command, list):
            for cmd in command:
                execute_single_command(cmd)
                try:
                    dur = float(cmd.get("duration", self.config.get("default_duration", 5.0)))
                except Exception:
                    dur = self.config.get("default_duration", 5.0)
                time.sleep(dur + 0.1)
        else:
            execute_single_command(command)

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

        # Frases clave para cambiar de modo
        self.activate_movement_phrases = ["preparate.", "preparate", "Preparate", "Preparate.", "Prepárate."]
        self.activate_conversation_phrases = ["Hablemos.", "Quiero hablar."]

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
        normalized_user_message = remove_accents(user_message_lower)

        # Verificar cambio de modo con coincidencia difusa
        normalized_movement_phrases = [remove_accents(phrase.lower()) for phrase in self.activate_movement_phrases]
        if difflib.get_close_matches(normalized_user_message, normalized_movement_phrases, n=1, cutoff=0.8):
            if self.current_mode != "movement":
                self.current_mode = "movement"
                response = "Cambiando a modo movimiento."
            else:
                response = "Ya estoy en modo movimiento."
            self.publish_response(response)
            self.is_processing = False
            return

        normalized_conversation_phrases = [remove_accents(phrase.lower()) for phrase in self.activate_conversation_phrases]
        if difflib.get_close_matches(normalized_user_message, normalized_conversation_phrases, n=1, cutoff=0.8):
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
            # En modo movimiento, se lanza el procesamiento en un hilo
            threading.Thread(target=self.handle_movement_command, args=(user_message,)).start()
            self.is_processing = False
            return
        else:
            augmented_prompt = self.generate_augmented_prompt(user_message)
            self.process_stream(augmented_prompt)
        self.is_processing = False

    def handle_movement_command(self, user_message):
        """
        Procesa el mensaje en modo movimiento:
          1. Obtiene el/los comando(s) LLM en formato JSON y lanza su ejecución de forma asíncrona.
          2. En paralelo, solicita al LLM una respuesta natural basada en el mensaje del usuario.
        """
        # Llamada al LLM para obtener comando(s) y lanzar su ejecución (no espera a que se terminen)
        command, internal_msg = self.movement_layer.process_movement_command(user_message)
        self.get_logger().info(f"Comando JSON recibido: {command}")

        # Lanzar en otro hilo la solicitud de confirmación en lenguaje natural,
        # utilizando únicamente el mensaje original del usuario.
        threading.Thread(target=self.send_natural_confirmation, args=(user_message,)).start()

    def send_natural_confirmation(self, user_message):
        """
        Solicita al LLM que responda en lenguaje natural confirmando la acción a partir del mensaje original.
        La respuesta se publica en el tópico de salida.
        """
        confirmation_prompt = (
            "Responde de forma natural confirmando la acción de movimiento, sin incluir datos técnicos. "
            f"El usuario indicó: '{user_message}'."
        )
        payload = {
            "model": "llama3.1",
            "messages": [
                {"role": "system", "content": "Eres un experto en dar respuestas naturales para confirmar acciones de movimiento de robots."},
                {"role": "user", "content": confirmation_prompt}
            ],
            "stream": False
        }
        url = "http://localhost:11434/api/chat"
        try:
            response = requests.post(url, json=payload)
            response.raise_for_status()
            data = response.json()
            content = ""
            if "message" in data and isinstance(data["message"], dict):
                content = data["message"].get("content", "")
            elif "response" in data:
                content = data["response"]
            content = content.strip()
        except Exception as e:
            content = f"Error al obtener confirmación: {e}"
        # Publicar solo la respuesta natural (para TTS)
        self.publish_response(content)

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
        self.streaming_mode = True
        self.first_stream_message_sent = False  # Reiniciamos para la nueva respuesta en streaming
        system_prompt = self.conversation_config.get("system_prompt", "")
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": augmented_prompt}
        ]
        payload = {
            "model": "llama3.1",
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
        self.streaming_mode = False  # Finalizamos el modo streaming al terminar


    def publish_response(self, text: str):
        """Publica la respuesta (solo lenguaje natural) en el tópico de salida (TTS)."""
        msg = String()
        if getattr(self, 'streaming_mode', False):
            # Si estamos en streaming, solo al primer mensaje se le agrega el prefijo
            if not getattr(self, 'first_stream_message_sent', False):
                msg.data = f"[ORION]: {text}"
                self.first_stream_message_sent = True
            else:
                msg.data = text
        else:
            # Para respuestas que no son streaming, se agrega el prefijo siempre.
            msg.data = f"[ORION]: {text}"
        self.response_pub.publish(msg)
        self.get_logger().info(f"Respuesta publicada: {msg.data}")


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
