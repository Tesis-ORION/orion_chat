import os
import sys
import json
import time
import threading
import tempfile
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify
import whisper
from flask_cors import CORS  # Importa Flask-CORS

class OrionSTTNode(Node):
    def __init__(self):
        super().__init__('orion_stt')
        self.publisher_ = self.create_publisher(String, 'orion_input', 10)
        
        # Cargar el modelo Whisper de OpenAI
        self.get_logger().info("Cargando modelo Whisper en modo CPU...")
        self.whisper_model = whisper.load_model("small", device="cpu")
        self.get_logger().info("Modelo Whisper cargado correctamente en modo CPU.")

        # Configurar la aplicación Flask
        self.app = Flask(__name__)
        CORS(self.app)  # Habilita CORS para todas las rutas

        self.setup_routes()

        # Ejecutar Flask en un hilo separado
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()
        self.get_logger().info("Servidor Flask iniciado en hilo separado.")

    def setup_routes(self):
        @self.app.route('/upload', methods=['POST'])
        def upload_audio():
            if 'file' not in request.files:
                return jsonify({"error": "No se encontró el archivo en la solicitud"}), 400
            file = request.files['file']
            if file.filename == '':
                return jsonify({"error": "No se seleccionó ningún archivo"}), 400
            
            # Guardar el archivo de audio de forma temporal
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp:
                file.save(tmp)
                tmp_path = tmp.name

            self.get_logger().info(f"Archivo recibido: {tmp_path}")

            try:
                # Transcribir el audio usando Whisper
                result = self.whisper_model.transcribe(tmp_path)
                transcript = result.get("text", "").strip()
                self.get_logger().info(f"Transcripción: {transcript}")

                # Publicar el comando transcrito en el tópico ROS
                if transcript:
                    msg = String()
                    msg.data = transcript
                    self.publisher_.publish(msg)
                    self.get_logger().info("Mensaje publicado en 'orion_input'.")
                else:
                    self.get_logger().info("No se obtuvo transcripción del audio.")

                # Eliminar el archivo temporal
                os.remove(tmp_path)
                return jsonify({"transcript": transcript}), 200
            except Exception as e:
                self.get_logger().error(f"Error al procesar el audio: {e}")
                return jsonify({"error": str(e)}), 500

    def run_flask(self):
        # Ejecutar la aplicación Flask en el puerto 5000
        self.app.run(host='0.0.0.0', port=5000)

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = OrionSTTNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Terminando nodo por interrupción manual.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
