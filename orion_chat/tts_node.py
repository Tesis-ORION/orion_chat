import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
import threading
import queue
import time
import random
import asyncio
import subprocess
import re
import edge_tts
import pyaudio

class OrionTTS(Node):
    def __init__(self):
        super().__init__('orion_tts')
        # Publishers para brazos y base
        self.pub_left_arm = self.create_publisher(Float64, '/servo_conn1_joint/cmd_pos', 10)
        self.pub_right_arm = self.create_publisher(Float64, '/servo_conn2_joint/cmd_pos', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscriber para las respuestas de texto
        self.create_subscription(String, 'orion_response', self.on_response, 10)
        # Flags de estado interno
        self.gestures_enabled = True            # gestos habilitados durante el habla
        self.autonomous_life_enabled = True     # movimientos de vida autónoma habilitados
        self.speaking = False                  # True mientras se está reproduciendo audio
        # Lock para asegurar que solo un audio se reproduce a la vez
        self.audio_lock = threading.Lock()
        # Cola de textos pendientes de sintetizar
        self.tts_queue = queue.Queue()
        # Voz para edge-tts (voz en español, se puede cambiar según preferencia)
        self.tts_voice = "es-VE-SebastianNeural"
        self._pa = pyaudio.PyAudio()
        self._audio_stream = self._pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=24000,             
            output=True
        )
        # Iniciar hilos de trabajo para TTS (2 procesos paralelos)
        self.worker_threads = []
        for i in range(2):
            t = threading.Thread(target=self.tts_worker, args=(i,), daemon=True)
            t.start()
            self.worker_threads.append(t)
        # Iniciar hilo de vida autónoma
        self.autonomous_thread = threading.Thread(target=self.autonomous_life_loop, daemon=True)
        self.autonomous_thread.start()
        self.get_logger().info("Nodo OrionTTS inicializado y en ejecución.")
    
    def on_response(self, msg):
        """Callback de suscripción a 'orion_response'. Añade el texto a la cola de TTS."""
        text = msg.data.strip()
        if not text:
            return  # ignora mensajes vacíos
        # Encolar el texto para procesamiento por los hilos de TTS
        self.tts_queue.put(text)
        self.get_logger().info(f"Texto recibido para locución: '{text}'")
    
    def tts_worker(self, index):
        """Hilo trabajador para procesar tareas de texto a voz en paralelo."""
        while rclpy.ok():
            try:
                raw_text = self.tts_queue.get(timeout=1.0)
            except queue.Empty:
                continue  # no hay tarea, revisar de nuevo (permite salida suave)
            # Elimina cualquier "[ORION]:" o "[orion]:" al inicio
            text = re.sub(r'^\s*\[?[Oo][Rr][Ii][Oo][Nn]\]?:?\s*', '', raw_text)
            # Sintetizar el texto a archivo de audio
            output_file = f"/tmp/orion_tts_{index}.mp3"
            try:
                # Usar edge-tts de forma asíncrona para generar el audio&#8203;:contentReference[oaicite:1]{index=1}
                asyncio.run(self.synthesize_to_file(text, output_file))
            except Exception as e:
                self.get_logger().error(f"Error en síntesis TTS: {e}")
                continue
            # Reproducir el audio y realizar gestos si corresponden (un discurso a la vez)
            with self.audio_lock:
                # Marcar estado "hablando"
                self.speaking = True
                 # Iniciar reproducción de audio con mpg123 (silencioso)
                proc = subprocess.Popen(["mpg123", "-q", output_file])

                # Arrancar hilo de gestos si está habilitado
                if self.gestures_enabled:
                    threading.Thread(
                        target=self._gestures_during_speech,
                        args=(proc, text),
                        daemon=True
                    ).start()

                # Esperar únicamente a que termine el audio
                proc.wait()
                # Marcar fin de locución
                self.speaking = False
            # (Audio lock liberado aquí, permite al siguiente audio reproducirse)
            # Verificar comandos especiales de cambio de modo tras hablar
            if text == "[ORION]: Cambiando a modo movimiento.":
                # Desactivar vida autónoma y gestos (modo movimiento)
                self.autonomous_life_enabled = False
                self.gestures_enabled = False
                # Bajar los brazos a neutro
                self.publish_arm_positions(left_angle=0.0, right_angle=0.0)
                self.get_logger().info("Cambiado a modo movimiento (gestos y vida autónoma desactivados).")
            elif text == "[ORION]: Cambiando a modo conversación. Podemos seguir dialogando normalmente.":
                # Reactivar vida autónoma y gestos (modo conversación)
                self.gestures_enabled = True
                self.autonomous_life_enabled = True
                self.get_logger().info("Cambiado a modo conversación (gestos y vida autónoma activados).")
            # Continuar con el siguiente texto en la cola
    
    async def synthesize_to_file(self, text, file_path):
        """Genera asincrónicamente el audio de `text` y lo guarda en file_path usando edge-tts."""
        communicate = edge_tts.Communicate(text=text, voice=self.tts_voice)
        await communicate.save(file_path)

    def _gestures_during_speech(self, proc, text):
        """Saludo y gestos independientes de brazos con inversión de la base."""
        # 1) Saludo si contiene "Hola"
        if re.search(r'\bhola\b', text, flags=re.IGNORECASE):
            self._wave_salute()

        # Patrones de giro base
        base_speeds = [0.05, 0.08, 0.1, -0.05, -0.08, -0.1]
        # Límites de movimiento de brazos
        left_limits  = [-0.8, -0.6, -0.4, -0.2]
        right_limits = [ 0.2,  0.4,  0.6,  0.8]

        # Mientras el audio está activo y gestos habilitados
        while proc.poll() is None and self.gestures_enabled:
            # Elige tiempos independientes para cada brazo
            left_time  = random.uniform(0.2, 0.5)
            right_time = random.uniform(0.2, 0.5)

            # Elige un ángulo distinto para cada brazo
            left_angle  = random.choice(left_limits)
            right_angle = random.choice(right_limits)

            # Lanzar hilos para mover cada brazo de forma concurrente
            threading.Thread(
                target=lambda: (
                    self.publish_arm_positions(left_angle, None),
                    time.sleep(left_time),
                    self.publish_arm_positions(0.0, None)
                ),
                daemon=True
            ).start()

            threading.Thread(
                target=lambda: (
                    self.publish_arm_positions(None, right_angle),
                    time.sleep(right_time),
                    self.publish_arm_positions(None, 0.0)
                ),
                daemon=True
            ).start()

            # Giro de base con inversión
            speed = random.choice(base_speeds)
            self.publish_base_turn(speed)
            time.sleep(0.3)
            self.publish_base_turn(-speed)
            time.sleep(0.3)
            self.publish_base_turn(0.0)

            # Pausa antes del siguiente ciclo
            time.sleep(random.uniform(0.1, 0.3))

        # Al terminar, volver a posición neutra
        self.publish_arm_positions(0.0, 0.0)
        self.publish_base_turn(0.0)




    def _wave_salute(self):
        """Levanta y baja el brazo derecho para simular un saludo."""
        steps, angle = 5, 1.57
        for i in range(steps+1):
            self.publish_arm_positions(None, angle * (i/steps))
            time.sleep(0.1)
        for i in range(steps, -1, -1):
            self.publish_arm_positions(None, angle * (i/steps))
            time.sleep(0.1)
        self.publish_arm_positions(None, 0.0)

    
    def publish_arm_positions(self, left_angle=None, right_angle=None):
        """Publica comandos de posición para los brazos. `None` significa sin cambio."""
        if left_angle is not None:
            msg = Float64()
            msg.data = float(left_angle)
            self.pub_left_arm.publish(msg)
        if right_angle is not None:
            msg = Float64()
            msg.data = float(right_angle)
            self.pub_right_arm.publish(msg)
    
    def publish_base_turn(self, speed=0.0):
        """Publica un comando de giro de base en el tópico /cmd_vel (solo angular.z)."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = float(speed)
        self.pub_cmd_vel.publish(twist)
    
    def autonomous_life_loop(self):
        """Hilo en segundo plano para movimientos sutiles de vida autónoma periódicos."""
        # Posición inicial neutra (brazos abajo, base sin giro)
        self.publish_arm_positions(left_angle=0.0, right_angle=0.0)
        self.publish_base_turn(speed=0.0)
        while rclpy.ok():
            # Esperar un intervalo aleatorio entre 15 y 25 segundos
            delay = random.uniform(7.0, 12.0)
            time.sleep(delay)
            # Si la vida autónoma está desactivada o el robot está hablando, omitir movimiento
            if not self.autonomous_life_enabled or self.speaking:
                continue
            # Calcular un movimiento sutil aleatorio
            left_target = random.uniform(-0.3, 0.0)   # ligero movimiento brazo izquierdo
            right_target = random.uniform(0.0, 0.3)   # ligero movimiento brazo derecho
            base_speed = random.choice([-0.1, 0.1])   # pequeño giro de base (izq o der)
            # Mover brazos gradualmente hacia la posición objetivo
            steps = 3
            current_left = 0.0
            current_right = 0.0
            for i in range(1, steps+1):
                new_left = current_left + (left_target - current_left) * (i/steps)
                new_right = current_right + (right_target - current_right) * (i/steps)
                self.publish_arm_positions(left_angle=new_left, right_angle=new_right)
                time.sleep(0.2)
            # Pequeño giro de base con inversión para volver al punto inicial
            self.publish_base_turn(speed=base_speed)
            time.sleep(0.5)
            # Giro inverso de la misma magnitud
            self.publish_base_turn(speed=-base_speed)
            time.sleep(0.5)
            # Detener base
            self.publish_base_turn(speed=0.0)
            for i in range(1, steps+1):
                new_left = left_target + (0.0 - left_target) * (i/steps)
                new_right = right_target + (0.0 - right_target) * (i/steps)
                self.publish_arm_positions(left_angle=new_left, right_angle=new_right)
                time.sleep(0.2)
            # Asegurar postura neutra al final del ciclo
            self.publish_arm_positions(left_angle=0.0, right_angle=0.0)
            self.publish_base_turn(speed=0.0)
    
    def send_stop_signals(self):
        """Envía comandos de parada (brazos a 0, base sin giro) al terminar."""
        self.publish_arm_positions(left_angle=0.0, right_angle=0.0)
        self.publish_base_turn(speed=0.0)
        # Cerrar PyAudio al apagar el nodo
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self._pa.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = OrionTTS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado - cerrando nodo.")
    # Al salir, detener movimientos y destruir nodo
    node.send_stop_signals()
    node.destroy_node()
    rclpy.shutdown()
