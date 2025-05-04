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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OrionTTS(Node):
    def __init__(self):
        super().__init__('orion_tts')
        # QoS para asegurar entrega fiable
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Publishers para brazos y base
        self.pub_left_arm  = self.create_publisher(Float64, '/servo_conn1_joint/cmd_pos', 10)
        self.pub_right_arm = self.create_publisher(Float64, '/servo_conn2_joint/cmd_pos', 10)
        self.pub_cmd_vel   = self.create_publisher(Twist,      '/cmd_vel',            10)
        # Subscriber para las respuestas de texto
        self.create_subscription(
            String, 'orion_response', self.on_response,
            qos_profile=self.qos
        )
        # Estado interno
        self.gestures_enabled        = True
        self.autonomous_life_enabled = True
        self.speaking                = False
        self.audio_lock              = threading.Lock()
        self.tts_queue               = queue.Queue()
        self.tts_voice               = "es-VE-SebastianNeural"
        # PyAudio / FFmpeg params
        self._pa         = pyaudio.PyAudio()
        self._audio_rate = 48000
        self._stream     = self._pa.open(
            format   = pyaudio.paInt16,
            channels = 1,
            rate     = self._audio_rate,
            output   = True
        )
        # Lanzar 2 hilos TTS
        for i in range(2):
            t = threading.Thread(target=self.tts_worker, args=(i,), daemon=True)
            t.start()
        # Hilo de vida autónoma
        threading.Thread(target=self.autonomous_life_loop, daemon=True).start()
        self.get_logger().info("Nodo OrionTTS inicializado y en ejecución.")

    def on_response(self, msg):
        text = msg.data.strip()
        if text:
            self.tts_queue.put(text)
            self.get_logger().info(f"Texto recibido para locución: '{text}'")

    def tts_worker(self, index):
        while rclpy.ok():
            try:
                raw = self.tts_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            # 1) Preparar el texto limpio
            text = re.sub(r'^\s*\[?[Oo][Rr][Ii][Oo][Nn]\]?:?\s*', '', raw)

            # 2) Lanzar FFmpeg fuera del lock para arrancar decodificación
            ff = subprocess.Popen(
                ["ffmpeg", "-i", "pipe:0",
                 "-f", "s16le", "-ar", str(self._audio_rate),
                 "-ac", "1", "pipe:1"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL
            )
            # 3) Iniciar síntesis asíncrona hacia ffmpeg.stdin
            threading.Thread(
                target=lambda: asyncio.run(self._stream_synthesis_to(ff.stdin, text)),
                daemon=True
            ).start()
            # 4) Arrancar el hilo de gestos, si está habilitado
            if self.gestures_enabled:
                threading.Thread(
                    target=self._gestures_during_speech,
                    args=(ff, text),
                    daemon=True
                ).start()

            # 5) Reproducir PCM **solo** dentro del lock
            with self.audio_lock:
                self.speaking = True
                while True:
                    chunk = ff.stdout.read(1024)
                    if not chunk:
                        break
                    self._stream.write(chunk)
                self.speaking = False
            # esperar a que ffmpeg termine de limpiar
            ff.wait()

            # Detectar comandos especiales
            if text == "Cambiando a modo movimiento.":
                self.autonomous_life_enabled = False
                self.gestures_enabled        = False
                self.publish_arm_positions(0.0, 0.0)
                self.get_logger().info("Modo movimiento activado.")
            elif text == "Cambiando a modo conversación. Podemos seguir dialogando normalmente.":
                self.autonomous_life_enabled = True
                self.gestures_enabled        = True
                self.get_logger().info("Modo conversación activado.")

    async def _stream_synthesis_to(self, stdin_pipe, text):
        """
        Toma los paquetes de audio de edge-tts y los escribe como bytes
        en stdin de FFmpeg para decodificarlos sobre la marcha.
        """
        communicate = edge_tts.Communicate(text=text, voice=self.tts_voice)
        async for message in communicate.stream():
            # message es un dict; solo el tipo "audio" contiene datos binarios
            if message.get("type") == "audio":
                stdin_pipe.write(message["data"])
        stdin_pipe.close()

    def _gestures_during_speech(self, proc, text):
        """Gestos de brazos y base de forma independiente."""
        # Saludo si toca
        if re.search(r'\bhola\b', text, flags=re.IGNORECASE):
            self._wave_salute()
        base_speeds  = [0.05, 0.08, 0.1, -0.05, -0.08, -0.1]
        left_limits  = [-0.8, -0.6, -0.4, -0.2]
        right_limits = [ 0.2,  0.4,  0.6,  0.8]

        while proc.poll() is None and self.gestures_enabled:
            # Movimientos independientes de cada brazo
            lt, rt = random.uniform(0.2,0.5), random.uniform(0.2,0.5)
            la, ra = random.choice(left_limits), random.choice(right_limits)
            threading.Thread(
                target=lambda: (
                    self.publish_arm_positions(la, None),
                    time.sleep(lt),
                    self.publish_arm_positions(0.0, None)
                ),
                daemon=True
            ).start()
            threading.Thread(
                target=lambda: (
                    self.publish_arm_positions(None, ra),
                    time.sleep(rt),
                    self.publish_arm_positions(None, 0.0)
                ),
                daemon=True
            ).start()
            # Giro de base con inversión
            spd = random.choice(base_speeds)
            self.publish_base_turn(spd); time.sleep(0.3)
            self.publish_base_turn(-spd); time.sleep(0.3)
            self.publish_base_turn(0.0)
            time.sleep(random.uniform(0.1,0.3))

        self.publish_arm_positions(0.0, 0.0)
        self.publish_base_turn(0.0)

    def _wave_salute(self):
        steps, angle = 5, 1.57
        for i in range(steps+1):
            self.publish_arm_positions(None, angle*(i/steps))
            time.sleep(0.1)
        for i in range(steps,-1,-1):
            self.publish_arm_positions(None, angle*(i/steps))
            time.sleep(0.1)
        self.publish_arm_positions(None, 0.0)

    def publish_arm_positions(self, left_angle=None, right_angle=None):
        if left_angle is not None:
            m = Float64(); m.data = left_angle;  self.pub_left_arm.publish(m)
        if right_angle is not None:
            m = Float64(); m.data = right_angle; self.pub_right_arm.publish(m)

    def publish_base_turn(self, speed=0.0):
        t = Twist(); t.linear.x = 0.0; t.angular.z = speed
        self.pub_cmd_vel.publish(t)

    def autonomous_life_loop(self):
        self.publish_arm_positions(0.0,0.0); self.publish_base_turn(0.0)
        while rclpy.ok():
            time.sleep(random.uniform(5.0,12.0))
            if not self.autonomous_life_enabled or self.speaking:
                continue
            lt = random.uniform(-0.3,0.0); rt = random.uniform(0.0,0.3)
            bs = random.choice([-0.1,0.1])
            steps = 3
            for i in range(1,steps+1):
                nl = lt*(i/steps); nr = rt*(i/steps)
                self.publish_arm_positions(nl,nr); time.sleep(0.2)
            self.publish_base_turn(bs); time.sleep(0.5)
            self.publish_base_turn(-bs); time.sleep(0.5)
            self.publish_base_turn(0.0)
            for i in range(1,steps+1):
                nl = lt + (0.0-lt)*(i/steps); nr = rt + (0.0-rt)*(i/steps)
                self.publish_arm_positions(nl,nr); time.sleep(0.2)
            self.publish_arm_positions(0.0,0.0); self.publish_base_turn(0.0)

    def send_stop_signals(self):
        self.publish_arm_positions(0.0,0.0)
        self.publish_base_turn(0.0)
        self._stream.stop_stream(); self._stream.close()
        self._pa.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = OrionTTS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado - cerrando nodo.")
    node.send_stop_signals()
    node.destroy_node()
    rclpy.shutdown()
