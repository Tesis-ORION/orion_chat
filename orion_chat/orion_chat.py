import rclpy
from rclpy.node import Node
import os
from groq import Groq
import re
from std_msgs.msg import String

# Configuración del Cliente de Groq
client = Groq(api_key=os.environ.get("GROQ_API_KEY"))

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

        # Cola para manejar mensajes y evitar duplicados
        self.last_message = None
        self.is_processing = False  # Bandera para evitar múltiples peticiones

    def listener_callback(self, msg):
        """ Maneja los mensajes del tópico, asegurando que solo se procese una vez """
        
        # Evitar procesamiento repetido del mismo mensaje
        if msg.data == self.last_message or self.is_processing:
            return
        
        self.is_processing = True  # Indicar que estamos procesando
        self.last_message = msg.data  # Guardar el último mensaje recibido

        user_message = msg.data
        self.get_logger().info(f"Recibido en ROS: {user_message}")

        # Enviar el mensaje a Groq
        orion_response = self.get_orion_response(user_message)

        # Publicar la respuesta en el tópico de ROS
        ros_msg = String()
        ros_msg.data = orion_response
        self.publisher.publish(ros_msg)

        self.get_logger().info(f"Respuesta publicada: {orion_response}")

        self.is_processing = False  # Indicar que ya terminamos

    def get_orion_response(self, user_message):
        """ Envía el mensaje a Groq con la personalidad de ORION """

        # Definir la personalidad de ORION
        system_prompt = (
            "Eres ORION, un robot avanzado diseñado para interacción humano-robot en educación e investigación. "
            "Tu personalidad es inteligente, amigable y con un toque de humor robótico. "
            "Siempre respondes con precisión, lógica y curiosidad científica. "
            "Tus respuestas comienzan con '[ORION]:'."
        )

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]

        try:
            chat_completion = client.chat.completions.create(
                messages=messages,
                model="deepseek-r1-distill-llama-70b"
            )

            # Extraer la respuesta y eliminar cualquier "<think>...</think>"
            full_response = chat_completion.choices[0].message.content
            clean_response = self.clean_groq_response(full_response)

            return clean_response

        except Exception as e:
            self.get_logger().error(f"Error al comunicarse con Groq: {e}")
            return "Error en ORION"

    def clean_groq_response(self, response):
        """ Elimina la sección '<think>...</think>' y devuelve solo la respuesta limpia """
        
        # Remover <think>...</think>
        response = re.sub(r"<think>.*?</think>", "", response, flags=re.DOTALL).strip()

        # Asegurar que solo devolvemos el texto después de "[ORION]:"
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
