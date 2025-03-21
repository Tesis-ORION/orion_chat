#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import requests
import re
from collections import deque
from ament_index_python.packages import get_package_share_directory
import time
import unicodedata
import difflib
from text_to_num import text2num

def remove_accents(input_str):
    """Elimina acentos y caracteres diacríticos de la cadena."""
    nfkd_form = unicodedata.normalize('NFD', input_str)
    return "".join([c for c in nfkd_form if not unicodedata.combining(c)])

def convert_number(text):
    """
    Convierte el texto a un número (soporta dígitos o palabras en español).
    Primero intenta convertir directamente a float; si falla, usa text2num.
    """
    try:
        return float(text)
    except ValueError:
        try:
            return float(text2num(text, "es"))
        except Exception:
            return None

class OrionChatNode(Node):
    def __init__(self):
        super().__init__("orion_chat")
        self.subscription = self.create_subscription(
            String,
            "orion_input",
            self.listener_callback,
            10
        )
        self.response_pub = self.create_publisher(String, "orion_response", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.chat_history = deque(maxlen=10)
        
        self.external_document = "Información predeterminada: No se cargaron documentos externos."
        self.system_prompt = (
            "Eres ORION, un robot avanzado diseñado para la interacción humano-robot en educación e investigación. "
            "Tu personalidad es inteligente, amigable y con un toque de humor robótico. "
            "Respondes de forma precisa, lógica y con curiosidad científica. Tus respuestas siempre comienzan con '[ORION]:' y son concisas. "
            "Si el usuario emite un comando que coincida con alguno de los siguientes patrones (para moverse o girar) y especifica parámetros numéricos (ya sea en dígitos o en palabras, por ejemplo, '5' o 'cinco segundos'), responde únicamente confirmando la ejecución del comando sin extender la conversación."
        )
        self.native_commands = []  # Se llenará desde el JSON
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
                        if "commands" in data:
                            self.native_commands.extend(data["commands"])
                        self.get_logger().info(f"Recurso cargado: {filename}")
                    except Exception as e:
                        self.get_logger().error(f"Error cargando {filename}: {e}")
            if external_docs:
                self.external_document = "\n".join(external_docs)
            if system_prompt_override:
                self.system_prompt = system_prompt_override
        else:
            self.get_logger().warn("No se encontró la carpeta 'resource' en el paquete.")

    def listener_callback(self, msg):
        if msg.data == self.last_message or self.is_processing:
            return
        self.is_processing = True
        self.last_message = msg.data

        user_message = msg.data
        self.get_logger().info(f"Recibido en ROS: {user_message}")
        self.chat_history.append(f"Usuario: {user_message}")

        # Clasifica la intención usando el modelo de Ollama
        intent = self.classify_intent(user_message)
        self.current_intent = intent  # Almacenamos la intención actual
        self.get_logger().info(f"Intención detectada: {intent}")

        if intent == "comando":
            # Procesa el comando nativo directamente si se detecta
            if self.check_native_command(user_message):
                self.is_processing = False
                return
        # Si es conversación o no se detecta comando, se genera el prompt aumentado
        augmented_prompt = self.generate_augmented_prompt(user_message)
        self.get_logger().info("Llamando al LLM en modo streaming...")
        self.process_stream(augmented_prompt)
        self.is_processing = False



    def check_native_command(self, message: str) -> bool:
        parsed = self.parse_command_from_text(message)
        if parsed:
            cmd_name, value1, duration = parsed
            self.get_logger().info(f"Comando nativo detectado en input: {cmd_name}")
            self.process_native_command(cmd_name, value1, duration)
            return True
        else:
            self.get_logger().info("No se detectó comando nativo en el input.")
        return False

    def fuzzy_match_command(self, text: str, command_name: str) -> bool:
        """
        Compara el texto con palabras clave requeridas para cada comando usando similitud.
        """
        mapping = {
            "move_forward": ["muevete", "mueve", "avanza", "ve", "adelante", "delante", "forward"],
            "move_backward": ["retrocede", "muevete", "mueve", "avanza", "ve", "atras", "backward"],
            "turn_left": ["gira", "voltea", "izquierda", "left"],
            "turn_right": ["gira", "voltea", "derecha", "right"]
        }
        if command_name not in mapping:
            return False
        words = text.split()
        for keyword in mapping[command_name]:
            for word in words:
                ratio = difflib.SequenceMatcher(None, word, keyword).ratio()
                if ratio >= 0.8:
                    return True
        return False

    def parse_command_from_text(self, text: str):
        normalized_text = remove_accents(text.lower())
        # Primero intenta encontrar coincidencias usando regex
        for command in self.native_commands:
            for pattern in command.get("patterns", []):
                if re.search(pattern, normalized_text):
                    return self.extract_parameters(normalized_text, command)
        # Si no se encuentra con regex, intenta con fuzzy matching
        for command in self.native_commands:
            if self.fuzzy_match_command(normalized_text, command["name"]):
                return self.extract_parameters(normalized_text, command)
        return None
    
    def classify_intent(self, user_message: str) -> str:
        prompt = (
            f"Dado el siguiente mensaje: '{user_message}', determina si se trata de un comando para mover o girar al robot "
            "o si es una conversación/pregunta normal. Responde solo con 'comando' o 'conversacion'."
        )
        # Configura el payload para Ollama
        payload = {
            "model": "llama3.2:3b",  # o el modelo que utilices
            "messages": [
                {"role": "system", "content": "Eres un clasificador de intenciones. Solo responde 'comando' o 'conversacion'."},
                {"role": "user", "content": prompt}
            ],
            "stream": False  # Para una respuesta inmediata y completa.
        }
        url = "http://localhost:11434/api/chat"
        try:
            response = requests.post(url, json=payload)
            response.raise_for_status()
            data = response.json()
            # Supongamos que la respuesta se encuentra en data['response'] o en data['message']['content']
            classification = data.get("response") or data.get("message", {}).get("content", "")
            classification = classification.strip().lower()
            # Aseguramos que solo devuelva 'comando' o 'conversacion'
            if "comando" in classification:
                return "comando"
            else:
                return "conversacion"
        except Exception as e:
            self.get_logger().error(f"Error al clasificar intención: {e}")
            # En caso de error, tratamos de manera conservadora como conversación.
            return "conversacion"


    def extract_parameters(self, text: str, command: dict):
        # Extraer duración (por ejemplo "tres segundos")
        duration_pattern = r"(?P<duration>(?:\d+(?:\.\d+)?|\w+))\s*(?:segundo|segundos)"
        duration_match = re.search(duration_pattern, text)
        duration = None
        if duration_match:
            duration_str = duration_match.group("duration")
            duration = convert_number(duration_str)
        # Buscar velocidad explícita ("velocidad de 4")
        speed_pattern = r"(?:velocidad\s*(?:de)?\s*)(?P<speed>(?:\d+(?:\.\d+)?|\w+))"
        speed_match = re.search(speed_pattern, text)
        speed = None
        if speed_match:
            speed_str = speed_match.group("speed")
            speed = convert_number(speed_str)
        # Si no se detecta velocidad por palabra clave, buscar de forma genérica en el texto restante
        if speed is None:
            text_without_duration = re.sub(duration_pattern, "", text)
            matches = re.findall(r"((?:\d+(?:\.\d+)?)|\w+)", text_without_duration)
            for candidate in matches:
                candidate_value = convert_number(candidate)
                if candidate_value is not None:
                    speed = candidate_value
                    break
        # Asignar valores por defecto si no se detectan
        if command["name"] in ["move_forward", "move_backward"]:
            if speed is None:
                speed = command["default_params"].get("speed", 0.5)
            if duration is None:
                duration = command["default_params"].get("duration", 2.0)
            if command["name"] == "move_backward" and speed > 0:
                speed = -speed
        elif command["name"] in ["turn_left", "turn_right"]:
            if speed is None:
                speed = command["default_params"].get("angular_z", 0.5)
            if duration is None:
                duration = command["default_params"].get("duration", 2.0)
            if command["name"] == "turn_right" and speed > 0:
                speed = -speed
        return command["name"], speed, duration

    def process_native_command(self, cmd_name: str, value1: float, duration: float):
        twist = Twist()
        response_text = ""
        if cmd_name == "move_forward":
            twist.linear.x = value1
            response_text = f"Bien, me muevo hacia adelante a {value1} m/s durante {duration} segundos."
        elif cmd_name == "move_backward":
            twist.linear.x = value1
            response_text = f"Perfecto, retrocediendo a {abs(value1)} m/s durante {duration} segundos."
        elif cmd_name == "turn_left":
            twist.angular.z = value1
            response_text = f"Ok, girando a la izquierda a {value1} rad/s durante {duration} segundos."
        elif cmd_name == "turn_right":
            twist.angular.z = value1
            response_text = f"Ok, girando a la derecha a {abs(value1)} rad/s durante {duration} segundos."
        else:
            self.get_logger().warn("Comando nativo desconocido.")
            return

        # Primero, se envía el mensaje de respuesta al TTS
        ros_msg = String()
        ros_msg.data = response_text
        self.response_pub.publish(ros_msg)
        self.chat_history.append(f"ORION: {response_text}")
        self.get_logger().info(f"Enviando respuesta TTS: {response_text}")

        # Luego se ejecuta el comando de movimiento
        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Comando de movimiento finalizado.")

    def generate_augmented_prompt(self, user_message):
        history_text = "\n".join(list(self.chat_history))
        prompt = (
            f"{self.system_prompt}\n\n"
            f"Contexto reciente:\n{history_text}\n\n"
            f"Documentos adicionales:\n{self.external_document}\n\n"
            f"Pregunta del usuario: {user_message}\n\n"
            f"Responde de forma natural y conversacional."
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
                    # Si se detecta signo de puntuación, consideramos que la frase puede haber terminado
                    if token and token[-1] in punctuation_marks:
                        phrase = buffer.strip()
                        if self.current_intent == "comando":
                            # En intención de comando, verificamos si se detecta un comando
                            parsed = self.parse_command_from_text(phrase)
                            if parsed:
                                self.get_logger().info(f"\n[Detectado comando en respuesta LLM, ejecutando acción nativa]")
                                cmd_name, value1, duration = parsed
                                self.process_native_command(cmd_name, value1, duration)
                            else:
                                self.get_logger().info(f"\n[Publicando a TTS]: {phrase}")
                                ros_msg = String()
                                ros_msg.data = phrase
                                self.response_pub.publish(ros_msg)
                                self.chat_history.append(f"ORION: {phrase}")
                        else:
                            # Para conversación, se publica el mensaje tal cual sin intentar extraer comandos.
                            self.get_logger().info(f"\n[Publicando a TTS]: {phrase}")
                            ros_msg = String()
                            ros_msg.data = phrase
                            self.response_pub.publish(ros_msg)
                            self.chat_history.append(f"ORION: {phrase}")
                        buffer = ""
                except Exception as e:
                    self.get_logger().error(f"Error procesando stream: {e}")

        if not token_found:
            self.get_logger().warn("No se recibió token del LLM.")
        if buffer:
            phrase = buffer.strip()
            if self.current_intent == "comando":
                parsed = self.parse_command_from_text(phrase)
                if parsed:
                    self.get_logger().info(f"\n[Detectado comando en respuesta final, ejecutando acción nativa]")
                    cmd_name, value1, duration = parsed
                    self.process_native_command(cmd_name, value1, duration)
                else:
                    self.get_logger().info(f"\n[Publicando a TTS]: {phrase}")
                    ros_msg = String()
                    ros_msg.data = phrase
                    self.response_pub.publish(ros_msg)
                    self.chat_history.append(f"ORION: {phrase}")
            else:
                self.get_logger().info(f"\n[Publicando a TTS]: {phrase}")
                ros_msg = String()
                ros_msg.data = phrase
                self.response_pub.publish(ros_msg)
                self.chat_history.append(f"ORION: {phrase}")


def main(args=None):
    rclpy.init(args=args)
    node = OrionChatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
