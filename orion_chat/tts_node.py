import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import TwistStamped
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
        # Histórico de movimientos de base
        self._base_history = []
        # QoS para asegurar entrega fiable
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Publishers actualizados a los topics y tipos nuevos
        self.pub_left_arm  = self.create_publisher(
            Float64MultiArray,
            '/simple_left_arm_controller/commands',
            self.qos
        )
        self.pub_right_arm = self.create_publisher(
            Float64MultiArray,
            '/simple_right_arm_controller/commands',
            self.qos
        )
        self.pub_cmd_vel   = self.create_publisher(
            TwistStamped,
            '/mobile_base_controller/cmd_vel',
            self.qos
        )

        # Variables para llevar el “último comando” de brazos y base
        self._current_left_angle  = 0.0
        self._current_right_angle = 0.0
        self._current_speed       = 0.0

        # Timers para republicar continuamente a 50 Hz
        timer_period = 1.0 / 50.0  # 50 Hz
        self.create_timer(timer_period, self._timer_publish_arms)    # publica brazos a 50 Hz
        self.create_timer(timer_period, self._timer_publish_vel)     # publica cmd_vel a 50 Hz

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
            frames_per_buffer= 4096,
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

            text = re.sub(r'^\s*\[?[Oo][Rr][Ii][Oo][Nn]\]?:?\s*', '', raw)
            t0 = time.perf_counter()
            ff = subprocess.Popen(
                ["ffmpeg", "-i", "pipe:0",
                 "-f", "s16le", "-ar", str(self._audio_rate),
                 "-ac", "1", "pipe:1"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL
            )
            threading.Thread(
                target=lambda: asyncio.run(self._stream_synthesis_to(ff.stdin, text)),
                daemon=True
            ).start()

            if self.gestures_enabled:
                threading.Thread(
                    target=self._gestures_during_speech,
                    args=(ff, text),
                    daemon=True
                ).start()

            with self.audio_lock:
                self.speaking = True
                while True:
                    chunk = ff.stdout.read(1024)
                    if not chunk:
                        break
                    self._stream.write(chunk)
                self.speaking = False
            ff.wait()
            t1 = time.perf_counter()
            synth_ms = (t1 - t0) * 1000
            self.get_logger().info(f"[METRIC][TTS] Synthesis latency for “{text[:20]}…”: {synth_ms:.1f} ms")

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
        communicate = edge_tts.Communicate(text=text, voice=self.tts_voice)
        async for message in communicate.stream():
            if message.get("type") == "audio":
                stdin_pipe.write(message["data"])
        stdin_pipe.close()

    def _gestures_during_speech(self, proc, text):
        if re.search(r'\bhola\b', text, flags=re.IGNORECASE):
            self._wave_salute()
        # base_speeds ahora en [0.5, 1.0] (o su negativo)
        left_limits  = [-1.0, -1.2, -1.3, -1.4]
        right_limits = [1.0, 1.2, 1.3, 1.4]

        while proc.poll() is None and self.gestures_enabled:
            lt = random.uniform(0.2, 0.5)
            rt = random.uniform(0.2, 0.5)
            la = random.choice(left_limits)
            ra = random.choice(right_limits)

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

            # velocidad base entre 1.0 y 1.3 (o negativa)
            spd = random.choice([
                random.uniform(1.0, 1.3),
                -random.uniform(1.0, 1.3)
            ])
            self.publish_base_turn(spd); time.sleep(0.3)
            self.publish_base_turn(-spd); time.sleep(0.3)
            self.publish_base_turn(0.0)
            time.sleep(random.uniform(0.1, 0.3))
        self.publish_arm_positions(0.0, 0.0)
        self.publish_base_turn(0.0)

    def _wave_salute(self):
        steps, angle = 5, 1.57
        for i in range(steps+1):
            self.publish_arm_positions(None, angle*(i/steps))
            time.sleep(0.1)
        for i in range(steps, -1, -1):
            self.publish_arm_positions(None, angle*(i/steps))
            time.sleep(0.1)
        self.publish_arm_positions(None, 0.0)

    def publish_arm_positions(self, left_angle=None, right_angle=None):
        """Guarda la última posición en las variables y publica inmediatamente."""
        if left_angle is not None:
            self._current_left_angle = left_angle
            msg = Float64MultiArray()
            msg.data = [left_angle]
            self.pub_left_arm.publish(msg)
        if right_angle is not None:
            self._current_right_angle = right_angle
            msg = Float64MultiArray()
            msg.data = [right_angle]
            self.pub_right_arm.publish(msg)

    def publish_base_turn(self, speed=0.0):
        """Guarda la última velocidad y publica inmediatamente."""
        self._current_speed = speed
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = speed
        self.pub_cmd_vel.publish(msg)

    # --- callbacks periódicos a 50 Hz para brazos y cmd_vel ---
    def _timer_publish_arms(self):
        # Publica la última posición guardada de ambos brazos a 50 Hz
        # (aunque no haya cambiado)
        msg_left = Float64MultiArray()
        msg_left.data = [self._current_left_angle]
        self.pub_left_arm.publish(msg_left)

        msg_right = Float64MultiArray()
        msg_right.data = [self._current_right_angle]
        self.pub_right_arm.publish(msg_right)

    def _timer_publish_vel(self):
        # Publica la última velocidad guardada a 50 Hz
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = self._current_speed
        self.pub_cmd_vel.publish(msg)

    def _recorded_base_turn(self, speed: float, duration: float):
        """Publica cmd_vel, guarda en el historial y espera la duración."""
        # Publicar
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = speed
        self.pub_cmd_vel.publish(msg)

        # Registrar para luego invertir
        self._base_history.append((speed, duration))

        # Esperar
        time.sleep(duration)


    def autonomous_life_loop(self):
        self.publish_arm_positions(0.0, 0.0)
        self.publish_base_turn(0.0)
        while rclpy.ok():
            time.sleep(random.uniform(5.0, 10.0))
            if not self.autonomous_life_enabled or self.speaking:
                continue
            lt = random.uniform(-0.5, 0.0)
            rt = random.uniform(0.0, 0.5)
            # --- velocidad base en [0.5, 0.7] (o negativo) ---
            bs = random.choice([
                random.uniform(0.5, 0.7),
                -random.uniform(0.5, 0.7)
            ])
            steps = 3
            for i in range(1, steps+1):
                nl = lt * (i / steps)
                nr = rt * (i / steps)
                self.publish_arm_positions(nl, nr)
                time.sleep(0.2)

            self.publish_base_turn(bs)
            time.sleep(1.0)
            self.publish_base_turn(-bs)
            time.sleep(1.0)
            self.publish_base_turn(0.0)

            for i in range(1, steps+1):
                nl = lt + (0.0 - lt) * (i / steps)
                nr = rt + (0.0 - rt) * (i / steps)
                self.publish_arm_positions(nl, nr)
                time.sleep(0.2)

            self.publish_arm_positions(0.0, 0.0)
            self.publish_base_turn(0.0)

    def send_stop_signals(self):
        self.publish_arm_positions(0.0, 0.0)
        self.publish_base_turn(0.0)
        self._stream.stop_stream()
        self._stream.close()
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
