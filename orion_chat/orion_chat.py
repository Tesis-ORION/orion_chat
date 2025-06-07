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
from geometry_msgs.msg import TwistStamped
import requests
from text_to_num import text2num
import difflib
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def remove_accents(input_str):
    nfkd_form = unicodedata.normalize('NFD', input_str)
    return "".join([c for c in nfkd_form if not unicodedata.combining(c)])

def convert_number(text):
    try:
        return float(text)
    except ValueError:
        try:
            return float(text2num(text, "es"))
        except Exception:
            return None

class MovementLayer:
    def __init__(self, config, cmd_vel_publisher, node_logger, parent_node):
        self.cmd_vel_pub = cmd_vel_publisher
        self.config = config
        self.node = parent_node
        self.logger = node_logger
        self.stop_timer = None

        # --- variables para “último comando” y estado de publicación continua ---
        self._current_linear_x  = 0.0
        self._current_angular_z = 0.0
        self._publish_until     = 0.0  # timestamp en epoch hasta cuando publicar a 30 Hz

        # Timer para publicar a 50 Hz: 1/50 = 0.05 s
        timer_period = 1.0 / 50.0  # 50 Hz
        # Cada 0.0333 s llamamos a _timer_publish, que publicará mientras no haya expirado la duración.
        self._timer = self.node.create_timer(1.0 / 50.0, self._timer_publish)

    def process_movement_command(self, message: str):
        system_prompt = self.config.get("system_prompt", "")
        messages_payload = [
            {"role": "system", "content": system_prompt},
            {"role": "user",   "content": message}
        ]
        payload = {
            "model": self.config.get("model", "gemma3:12b"),
            "messages": messages_payload,
            "top_p": 1.0,
            "stream": False,
            "temperature": 0.0,
            "stop": ["}", "]"]
        }
        url = "http://localhost:11434/api/chat"
        try:
            t0 = time.perf_counter()
            response = requests.post(url, json=payload)
            response.raise_for_status()
            data = response.json()
            t1 = time.perf_counter()
            llm_latency = (t1 - t0) * 1000
            self.logger.info(f"[METRIC][LLM] Latencia LLM movimiento: {llm_latency:.1f} ms")
            if "message" in data and isinstance(data["message"], dict):
                content = data["message"].get("content", "").strip()
            elif "response" in data:
                content = data["response"].strip()
            else:
                content = ""
            self.logger.info(f"Respuesta LLM (movimiento): {content}")
            command = json.loads(content)  # Se asume JSON puro
        except Exception as e:
            err_msg = f"Error procesando comando de movimiento: {e}"
            self.logger.error(err_msg)
            return None, err_msg

        # Arrancamos ejecución en hilo separado
        threading.Thread(target=self.execute_commands, args=(command,), daemon=True).start()
        return command, "Comando recibido y en ejecución."

    def execute_commands(self, command):
        """
        1) Si es lista, ejecuta iterativamente cada subcomando (esperando su duration).
        2) Si es dict, ejecuta un solo comando.
        En cada subcomando, actualiza _current_linear_x, _current_angular_z y calcula _publish_until = now + duration.
        Luego, el timer _timer_publish se encargará de publicar continuamente a 30 Hz hasta que expire _publish_until.
        """
        def execute_single(cmd):
            # --- PARSEO DE PARÁMETROS ---
            try:
                linear_x = float(cmd.get("linear_x", 
                    self.config.get("default_forward", {}).get("linear_x", 0.5)))
                angular_z = float(cmd.get("angular_z", 
                    self.config.get("default_left", {}).get("angular_z", 0.5)))
                duration = float(cmd.get("duration", 
                    self.config.get("default_forward", {}).get("duration", 2.0)))
            except Exception as e:
                self.logger.error(f"Error en parámetros: {e}")
                return

            # Guardamos valores para el timer
            self._current_linear_x = linear_x
            self._current_angular_z = angular_z
            # Definimos hasta cuándo publicar (según duración)
            self._publish_until = time.time() + duration

            # Publicación inicial inmediata
            twist = TwistStamped()
            twist.header.stamp = self.pub_clock().now().to_msg()
            twist.twist.linear.x = linear_x
            twist.twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
            self.logger.info(f"Ejecutando: lx={linear_x}, az={angular_z}, dur={duration}")

            # Si duration = 0, hacemos un “stop” inmediato (silencioso)
            if duration <= 0:
                self._publish_until = 0.0
                stop_msg = TwistStamped()
                stop_msg.header.stamp = self.pub_clock().now().to_msg()
                stop_msg.twist.linear.x = 0.0
                stop_msg.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(stop_msg)
                self.logger.info("Duración 0: robot detenido inmediatamente.")

        # Ejecución secuencial de lista o único dict
        if isinstance(command, list):
            for subcmd in command:
                execute_single(subcmd)
                # Esperamos la duración completa + pequeño margen antes de pasar al siguiente
                try:
                    d = float(subcmd.get("duration", 
                        self.config.get("default_forward", {}).get("duration", 2.0)))
                except Exception:
                    d = self.config.get("default_forward", {}).get("duration", 2.0)
                time.sleep(d + 0.1)
        else:
            execute_single(command)

    def _timer_publish(self):
        """
        Este callback se dispara a 30 Hz. Mientras el timestamp actual <= _publish_until,
        publica continuamente el último TwistStamped con _current_linear_x y _current_angular_z.
        """
        if time.time() <= self._publish_until:
            twist = TwistStamped()
            twist.header.stamp = self.pub_clock().now().to_msg()
            twist.twist.linear.x = self._current_linear_x
            twist.twist.angular.z = self._current_angular_z
            self.cmd_vel_pub.publish(twist)
        # Si ya expiró la duración, nada que hacer; el último stop se envió manualmente.

    def pub_clock(self):
        return rclpy.clock.Clock()


class OrionChatMovementNode(Node):
    def __init__(self):
        super().__init__("orion_chat_movement")
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Suscripción de entrada y publicación de respuestas (TTS / conversación)
        self.create_subscription(
            String, 'orion_input', self.listener_callback, qos_profile=self.qos
        )
        self.response_pub = self.create_publisher(
            String, 'orion_response', qos_profile=self.qos
        )
        # NUEVO: TwistStamped en /mobile_base_controller/cmd_vel 
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/mobile_base_controller/cmd_vel', qos_profile=self.qos
        )

        # Carga de configuración 
        self.movement_config = self.load_resource("movement_config.json")
        self.conversation_config = self.load_resource("conversation_config.json")

        # Capa de movimiento con puntero a publisher de cmd_vel
        self.movement_layer = MovementLayer(
            self.movement_config,
            self.cmd_vel_pub,
            self.get_logger(),
            self
        )

        # Estados y llaves para cambiar modo
        self.chat_history = deque(maxlen=100)
        self.last_message = None
        self.is_processing = False
        self.current_mode = "conversation"
        self.current_emotion = 'neutral'

        # Subscripción de emociones (unchanged)
        self.create_subscription(
            String,
            self.conversation_config.get("emotion_topic", "/emotion"),
            self.emotion_callback,
            qos_profile=self.qos
        )

        # Setup de palabras clave para modos
        bases_mov = ["preparate", "prepárate"]
        bases_conv = ["hablemos", "quiero hablar"]

        self.movement_keys = []
        for p in bases_mov:
            norm = remove_accents(p)
            self.movement_keys += [norm, f"orion {norm}", f"{norm} orion"]

        self.conversation_keys = []
        for p in bases_conv:
            norm = remove_accents(p)
            self.conversation_keys += [norm, f"orion {norm}", f"{norm} orion"]

        # Construir system_prompt para conversación
        desc = self.conversation_config.get("description", "")
        ctx = self.conversation_config.get("context", "")
        instr = "\n".join(self.conversation_config.get("instructions", []))
        ext_doc = self.conversation_config.get("external_document", "")
        self.system_prompt = f"{desc}\nContexto: {ctx}\n{ext_doc}\nInstrucciones:\n{instr}\n"

    def emotion_callback(self, msg: String):
        self.current_emotion = msg.data

    def load_resource(self, filename):
        try:
            pkg_share     = get_package_share_directory("orion_chat")
            resources_dir = os.path.join(pkg_share, "resource")
        except Exception as e:
            self.get_logger().error(f"Error al obtener share dir: {e}")
            return {}
        if not os.path.isdir(resources_dir):
            self.get_logger().warning("No se encontró carpeta 'resource'.")
            return {}
        path = os.path.join(resources_dir, filename)
        if not os.path.isfile(path):
            return {}
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            self.get_logger().info(f"Recurso cargado: {filename}")
            return data
        except Exception as e:
            self.get_logger().error(f"Error cargando {filename}: {e}")
            return {}

    def listener_callback(self, msg: String):
        if msg.data == self.last_message or self.is_processing:
            return
        self.is_processing = True
        self.last_message = msg.data
        user_message = msg.data.strip()
        self.get_logger().info(f"Recibido: {user_message}")
        self.chat_history.append(f"Usuario: {user_message}")
        normalized = remove_accents(user_message.lower().replace(",", "").strip())

        # Modo movimiento
        if difflib.get_close_matches(normalized, self.movement_keys, n=1, cutoff=0.8):
            resp = ("Cambiando a modo movimiento."
                    if self.current_mode != "movement"
                    else "Ya estoy en modo movimiento.")
            self.current_mode = "movement"
            self.publish_response(resp)
            self.is_processing = False
            return

        # Modo conversación
        if difflib.get_close_matches(normalized, self.conversation_keys, n=1, cutoff=0.8):
            resp = ("Cambiando a modo conversación. Podemos seguir dialogando normalmente."
                    if self.current_mode != "conversation"
                    else "Ya estoy en modo conversación.")
            self.current_mode = "conversation"
            self.publish_response(resp)
            self.is_processing = False
            return

        # Procesamiento según modo
        if self.current_mode == "movement":
            # Llamamos a process_movement_command, que iniciará execute_commands en fondo
            threading.Thread(target=self.handle_movement_command,
                            args=(user_message,), daemon=True).start()
            self.is_processing = False
        else:
            # Modo conversación (igual que antes)
            augmented = self.generate_augmented_prompt(user_message)
            threading.Thread(target=self.process_stream,
                            args=(augmented,), daemon=True).start()
            self.is_processing = False

    def handle_movement_command(self, user_message):
        command, _ = self.movement_layer.process_movement_command(user_message)
        self.get_logger().info(f"Comando JSON recibido: {command}")
        threading.Thread(target=self.send_natural_confirmation,
                         args=(user_message,), daemon=True).start()


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
            "model": "gemma3:12b",
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
            f"Responde de forma natural y breve, y a demas que la respuesta sea reactiva a {self.current_emotion} es decir que esta emocion se vea reflejada en tu forma de responder."
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
            "model": "gemma3:12b",
            "messages": messages,
            "stream": True
        }
        url = "http://localhost:11434/api/chat"
        try:
            t0 = time.perf_counter()
            response = requests.post(url, json=payload, stream=True)
            response.raise_for_status()
            t1 = time.perf_counter()
            self.get_logger().info(f"[METRIC][LLM] Conversational LLM latency (setup): {(t1 - t0)*1000:.1f} ms")
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