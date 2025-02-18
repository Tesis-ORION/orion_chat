import rclpy
from rclpy.node import Node
import requests
import re
from std_msgs.msg import String

class OrionChatNode(Node):
    def __init__(self):
        super().__init__("orion_chat")

        # Suscribirse al tópico de entrada (mensajes de usuario)
        self.subscription = self.create_subscription(
            String,
            "orion_input",
            self.listener_callback,
            10
        )

        # Publicador de respuestas de ORION
        self.publisher = self.create_publisher(String, "orion_response", 10)

        # Variables para evitar duplicados
        self.last_message = None
        self.is_processing = False  # Bandera para evitar múltiples peticiones

    def listener_callback(self, msg):
        """Maneja los mensajes del tópico, asegurando que se procese solo una vez"""
        if msg.data == self.last_message or self.is_processing:
            return
        
        self.is_processing = True
        self.last_message = msg.data

        user_message = msg.data
        self.get_logger().info(f"Recibido en ROS: {user_message}")

        # Se obtiene la respuesta local del modelo
        orion_response = self.get_orion_response(user_message)

        # Publicar la respuesta en el tópico de ROS
        ros_msg = String()
        ros_msg.data = orion_response
        self.publisher.publish(ros_msg)

        self.is_processing = False

    def get_orion_response(self, user_message):
        """Envía el mensaje al modelo local de Ollama (llama3.1)"""
        system_prompt = (
            "Eres ORION, un robot avanzado diseñado para interacción humano-robot en educación e investigación. "
            "Tu personalidad es inteligente, amigable y con un toque de humor robótico. "
            "Siempre respondes con precisión, lógica y curiosidad científica. "
            "Tus respuestas comienzan con '[ORION]:' y son muy cortas y concretas a menos de que te pidan lo contrario."
        )

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]

        try:
            # Preparar payload para la llamada HTTP al endpoint local de Ollama
            payload = {
                "model": "llama3.1",
                "messages": messages,
                "stream": False
            }
            url = "http://localhost:11434/api/chat"
            response = requests.post(url, json=payload)
            response.raise_for_status()
            data = response.json()

            full_response = data["message"]["content"]
            clean_response = self.clean_ollama_response(full_response)
            return clean_response

        except Exception as e:
            self.get_logger().error(f"Error al comunicarse con Ollama: {e}")
            return "Error en ORION"

    def clean_ollama_response(self, response):
        """Elimina cualquier sección '<think>...</think>' y extrae la respuesta limpia"""
        response = re.sub(r"<think>.*?</think>", "", response, flags=re.DOTALL).strip()
        if "[ORION]:" in response:
            response = response.split("[ORION]:", 1)[-1].strip()
        return f"[ORION]: {response}"

def main(args=None):
    rclpy.init(args=args)
    node = OrionChatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
