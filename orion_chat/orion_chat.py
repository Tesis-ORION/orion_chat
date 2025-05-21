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
from text_to_num import text2num
import difflib
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


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
            if "message" in data and isinstance(data["message"], dict):
                content = data["message"].get("content", "").strip()
            elif "response" in data:
                content = data["response"].strip()
            else:
                content = ""
            self.logger.info(f"Respuesta del LLM para movimiento: {content}")
            command = json.loads(content)
        except Exception as e:
            err_msg = f"Error procesando comando de movimiento: {e}"
            self.logger.error(err_msg)
            return None, err_msg

        threading.Thread(target=self.execute_commands, args=(command,)).start()
        return command, "Comando recibido y en ejecución."

    def execute_commands(self, command):
        def execute_single_command(cmd):
            try:
                if "linear_x" in cmd:
                    linear_x = float(cmd["linear_x"])
                else:
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

def remove_accents(input_str):
    """Elimina acentos y caracteres diacríticos de la cadena."""
    nfkd_form = unicodedata.normalize('NFD', input_str)
    return "".join([c for c in nfkd_form if not unicodedata.combining(c)])

class OrionChatMovementNode(Node):
    def __init__(self):
        super().__init__("orion_chat_movement")
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            String,
            'orion_input',
            self.listener_callback,
            qos_profile=self.qos
        )
        self.response_pub = self.create_publisher(
            String,
            'orion_response',
            qos_profile=self.qos
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.chat_history = deque(maxlen=10)
        self.last_message = None
        self.is_processing = False
        self.current_mode = "conversation"

        # Cargar archivos de configuración
        self.movement_config    = self.load_resource("movement_config.json")
        self.conversation_config = self.load_resource("conversation_config.json")

        # --- Cambio clave: instanciar aquí ---
        self.movement_layer = MovementLayer(
            self.movement_config,
            self.cmd_vel_pub,
            self.get_logger()
        )

        # Inicializar manejo de emoción
        self.current_emotion = 'neutral'
        self.emotion_subscriber = self.create_subscription(
            String,
            self.conversation_config.get("emotion_topic", "/emotion"),
            self.emotion_callback,
            qos_profile=self.qos
        )

        # Frases base para difflib
        bases_mov = ["preparate", "prepárate"]
        bases_conv = ["hablemos", "quiero hablar"]

        # Variantes con "orion"
        self.movement_keys = []
        for p in bases_mov:
            norm = remove_accents(p)
            self.movement_keys += [
                norm,
                f"orion {norm}",
                f"{norm} orion",
            ]

        self.conversation_keys = []
        for p in bases_conv:
            norm = remove_accents(p)
            self.conversation_keys += [
                norm,
                f"orion {norm}",
                f"{norm} orion",
            ]

        # Construir system_prompt
        desc    = self.conversation_config["description"]
        ctx     = self.conversation_config["context"]
        instr   = "\n".join(self.conversation_config["instructions"])
        ext_doc = self.conversation_config.get("external_document", "")
        self.system_prompt = (
            f"{desc}\n"
            f"Contexto: {ctx}\n"
            f"{ext_doc}\n"
            f"Instrucciones:\n{instr}\n"
        )

    def emotion_callback(self, msg: String):
        """Actualiza la emoción actual."""
        self.current_emotion = msg.data
        self.get_logger().info(f"Emoción actualizada a: {self.current_emotion}")
        # Ya no instanciamos movement_layer aquí

    def load_resource(self, filename):
        try:
            pkg_share     = get_package_share_directory("orion_chat")
            resources_dir = os.path.join(pkg_share, "resource")
        except Exception as e:
            self.get_logger().error(f"Error al obtener el directorio share: {e}")
            return {}
        resource_data = {}
        if os.path.isdir(resources_dir):
            for file in os.listdir(resources_dir):
                if file == filename:
                    path = os.path.join(resources_dir, file)
                    try:
                        with open(path, 'r', encoding='utf-8') as f:
                            resource_data = json.load(f)
                        self.get_logger().info(f"Recurso cargado: {file}")
                    except Exception as e:
                        self.get_logger().error(f"Error cargando {file}: {e}")
        else:
            self.get_logger().warn("No se encontró la carpeta 'resource'.")
        return resource_data

    def listener_callback(self, msg: String):
        if msg.data == self.last_message or self.is_processing:
            return
        self.is_processing = True
        self.last_message = msg.data
        user_message = msg.data
        self.get_logger().info(f"Recibido: {user_message}")
        self.chat_history.append(f"Usuario: {user_message}")
        normalized = remove_accents(user_message.lower().replace(",", "").strip())

        # Cambio de modo movimiento
        if difflib.get_close_matches(normalized, self.movement_keys, n=1, cutoff=0.8):
            response = ("Cambiando a modo movimiento." 
                        if self.current_mode != "movement"
                        else "Ya estoy en modo movimiento.")
            self.current_mode = "movement"
            self.publish_response(response)
            self.is_processing = False
            return

        # Cambio de modo conversación
        if difflib.get_close_matches(normalized, self.conversation_keys, n=1, cutoff=0.8):
            response = ("Cambiando a modo conversación. Podemos seguir dialogando normalmente."
                        if self.current_mode != "conversation"
                        else "Ya estoy en modo conversación.")
            self.current_mode = "conversation"
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
        command, _ = self.movement_layer.process_movement_command(user_message)
        self.get_logger().info(f"Comando JSON recibido: {command}")
        threading.Thread(
            target=self.send_natural_confirmation,
            args=(user_message,),
            daemon=True
        ).start()

    def send_natural_confirmation(self, user_message):
        """
        Solicita al LLM una confirmación en lenguaje natural en streaming,
        publicando cada frase según llegue.
        """
        # Prepara modo streaming
        self.streaming_mode = True
        self.first_stream_message_sent = False

        # Construye el prompt de confirmación
        confirmation_prompt = (
            "Responde de forma natural confirmando la acción de movimiento, "
            "sin incluir datos técnicos. "
            f"El usuario indicó: '{user_message}'."
        )
        system_prompt = "Eres un experto en dar respuestas naturales para confirmar acciones de movimiento de robots."

        payload = {
            "model": "gemma3",
            "messages": [
                {"role": "system",   "content": system_prompt},
                {"role": "user",     "content": confirmation_prompt}
            ],
            "stream": True
        }

        url = "http://localhost:11434/api/chat"
        try:
            # peticion en streaming
            response = requests.post(url, json=payload, stream=True)
            response.raise_for_status()
        except Exception as e:
            self.get_logger().error(f"Error al comunicarse para confirmación: {e}")
            # Publica un único mensaje de error
            self.publish_response(f"[ORION]: Error al obtener confirmación: {e}")
            self.streaming_mode = False
            return

        buffer = ""


        # Procesa cada línea de la respuesta stream
        for line in response.iter_lines(decode_unicode=True):
            if not line:
                continue
            token_received = True
            try:
                data = json.loads(line)
                # Extrae el token o el content dentro de message
                token = data.get("token")
                if token is None:
                    msg_obj = data.get("message")
                    token = msg_obj.get("content", "") if isinstance(msg_obj, dict) else ""
                buffer += token

                # Cuando llegamos a un signo de puntuación, publicamos la frase completa
                if buffer and (
                    buffer[-1] in { '!', '?'}
                    or re.search(r'(?<!\d)\.(?!\d)$', buffer)
                ):
                    phrase = buffer.strip()
                    self.publish_response(phrase)
                    # Opcional: llevar historial de confirmaciones
                    self.chat_history.append(f"[ORION]: {phrase}")
                    buffer = ""
            except Exception as e:
                self.get_logger().error(f"Error procesando token de confirmación: {e}")

        # Si quedó algo en el buffer, publicarlo también
        if buffer:
            self.publish_response(buffer.strip())
            self.chat_history.append(f"[ORION]: {buffer.strip()}")

        # Fin del streaming
        self.streaming_mode = False

    def generate_augmented_prompt(self, user_message):
        history_text = "\n".join(list(self.chat_history))
# Usar el prompt construido a partir de las instrucciones
        system_prompt = self.system_prompt        
        external_document = self.conversation_config.get("external_document", "")
        prompt = (
            f"{system_prompt}\n"
            f"El usuario está sintiendo **{self.current_emotion}**.\n\n"
            f"Contexto reciente:\n{history_text}\n\n"
            f"Documentos adicionales:\n{external_document}\n\n"
            f"Pregunta del usuario: {user_message}\n\n"
            f"Responde de forma natural y breve, y formatea la salida como \"[{self.current_emotion}]: respuesta\"."
        )
        return prompt

    def process_stream(self, augmented_prompt):
        self.streaming_mode = True
        self.first_stream_message_sent = False  # Reiniciamos para la nueva respuesta en streaming
        # Usar el system_prompt completo construido en __init__
        system_prompt = self.system_prompt
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": augmented_prompt}
        ]
        payload = {
            "model": "gemma3",
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
                    if buffer and (
                        buffer[-1] in { '!', '?'}
                        or re.search(r'(?<!\d)\.(?!\d)$', buffer)
                    ):
                        phrase = buffer.strip()
                        self.publish_response(phrase)
                        self.chat_history.append(f"[ORION]: {phrase}")
                        buffer = ""
                except Exception as e:
                    self.get_logger().error(f"Error procesando stream: {e}")
        if not token_found:
            self.get_logger().warn("No se recibió token del LLM.")
        if buffer:
            phrase = buffer.strip()
            self.publish_response(phrase)
            self.chat_history.append(f"[ORION]: {phrase}")
        self.streaming_mode = False  # Finalizamos el modo streaming al terminar


    def publish_response(self, text: str):
        """Publica la respuesta (solo lenguaje natural) en el tópico de salida (TTS)."""
        """ Quitar un posible prefijo "[ORION]:" que ya venga en el texto """
        clean_text = re.sub(r'^\s*\[ORION\]\s*:\s*', '', text)
        msg = String()
        if getattr(self, 'streaming_mode', False):
            # Si estamos en streaming, solo al primer mensaje se le agrega el prefijo
            if not getattr(self, 'first_stream_message_sent', False):
                msg.data = f"[ORION]: {clean_text}"
                self.first_stream_message_sent = True
            else:
                msg.data = clean_text
        else:
            # Para respuestas que no son streaming, se agrega el prefijo siempre.
            msg.data = f"[ORION]: {clean_text}"
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
