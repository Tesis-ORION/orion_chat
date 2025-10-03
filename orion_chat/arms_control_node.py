import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import threading
import queue
import time
import random
import re
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OrionArmsControl(Node):
    def __init__(self):
        super().__init__('orion_arms_control')
        
        # QoS para asegurar entrega fiable
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers solo para brazos
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

        # Variables para llevar el "último comando" de brazos
        self._current_left_angle  = 0.0
        self._current_right_angle = 0.0

        # Timer para republicar continuamente a 50 Hz solo brazos
        timer_period = 1.0 / 50.0  # 50 Hz
        self.create_timer(timer_period, self._timer_publish_arms)
        
        # Subscriber para las respuestas de texto (para gestos)
        self.create_subscription(
            String, 'orion_response', self.on_response,
            qos_profile=self.qos
        )
        
        # Estado interno
        self.gestures_enabled        = True
        self.autonomous_life_enabled = True
        self.speaking                = False
        self.gesture_queue           = queue.Queue()
        
        # Hilo para gestión de gestos
        threading.Thread(target=self.gesture_worker, daemon=True).start()
        
        # Hilo de vida autónoma (solo brazos)
        threading.Thread(target=self.autonomous_life_loop, daemon=True).start()
        
        self.get_logger().info("Nodo OrionArmsControl inicializado - Solo control de brazos.")

    def on_response(self, msg):
        """Recibe respuestas de texto y programa gestos apropiados."""
        text = msg.data.strip()
        if text and self.gestures_enabled:
            self.gesture_queue.put(text)
            self.get_logger().info(f"Texto recibido para gestos: '{text[:30]}...'")

    def gesture_worker(self):
        """Hilo que procesa los gestos basados en el texto."""
        while rclpy.ok():
            try:
                text = self.gesture_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            # Procesar texto específico para gestos
            text_clean = re.sub(r'^\s*\[?[Oo][Rr][Ii][Oo][Nn]\]?:?\s*', '', text)
            
            # Cambio de modo
            if text == "Cambiando a modo movimiento.":
                self.autonomous_life_enabled = False
                self.gestures_enabled        = False
                self.publish_arm_positions(0.0, 0.0)
                self.get_logger().info("Modo movimiento activado - Brazos en reposo.")
                continue
            elif text == "Cambiando a modo conversación. Podemos seguir dialogando normalmente.":
                self.autonomous_life_enabled = True
                self.gestures_enabled        = True
                self.get_logger().info("Modo conversación activado - Gestos habilitados.")
                continue

            # Gestos durante conversación
            if self.gestures_enabled:
                self._execute_conversation_gestures(text_clean)

    def _execute_conversation_gestures(self, text):
        """Ejecuta gestos apropiados durante la conversación."""
        self.speaking = True
        
        # Saludo especial
        if re.search(r'\bhola\b', text, flags=re.IGNORECASE):
            self._wave_salute()
        
        # Gestos aleatorios durante conversación
        left_limits  = [-1.0, -1.2, -1.3, -1.4]
        right_limits = [1.0, 1.2, 1.3, 1.4]

        # Simular duración aproximada de speech (basado en longitud del texto)
        estimated_duration = max(2.0, len(text) * 0.08)  # ~80ms por carácter
        end_time = time.time() + estimated_duration

        while time.time() < end_time and self.gestures_enabled:
            lt = random.uniform(0.2, 0.5)
            rt = random.uniform(0.2, 0.5)
            la = random.choice(left_limits)
            ra = random.choice(right_limits)

            # Brazo izquierdo
            threading.Thread(
                target=lambda: (
                    self.publish_arm_positions(la, None),
                    time.sleep(lt),
                    self.publish_arm_positions(0.0, None)
                ),
                daemon=True
            ).start()
            
            # Brazo derecho
            threading.Thread(
                target=lambda: (
                    self.publish_arm_positions(None, ra),
                    time.sleep(rt),
                    self.publish_arm_positions(None, 0.0)
                ),
                daemon=True
            ).start()

            time.sleep(random.uniform(0.1, 0.3))
        
        self.publish_arm_positions(0.0, 0.0)
        self.speaking = False

    def _wave_salute(self):
        """Gesto de saludo con el brazo derecho."""
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

    def _timer_publish_arms(self):
        """Publica la última posición guardada de ambos brazos a 50 Hz."""
        msg_left = Float64MultiArray()
        msg_left.data = [self._current_left_angle]
        self.pub_left_arm.publish(msg_left)

        msg_right = Float64MultiArray()
        msg_right.data = [self._current_right_angle]
        self.pub_right_arm.publish(msg_right)

    def autonomous_life_loop(self):
        """Vida autónoma solo con movimientos de brazos."""
        self.publish_arm_positions(0.0, 0.0)
        
        while rclpy.ok():
            time.sleep(random.uniform(8.0, 15.0))  # Menos frecuente que antes
            if not self.autonomous_life_enabled or self.speaking:
                continue
            
            # Movimientos suaves de brazos
            lt = random.uniform(-0.3, 0.0)
            rt = random.uniform(0.0, 0.3)
            
            steps = 4
            # Movimiento hacia posición objetivo
            for i in range(1, steps+1):
                nl = lt * (i / steps)
                nr = rt * (i / steps)
                self.publish_arm_positions(nl, nr)
                time.sleep(0.25)

            # Mantener posición
            time.sleep(random.uniform(1.0, 3.0))

            # Regreso a posición neutral
            for i in range(1, steps+1):
                nl = lt * (1.0 - i / steps)
                nr = rt * (1.0 - i / steps)
                self.publish_arm_positions(nl, nr)
                time.sleep(0.25)

            self.publish_arm_positions(0.0, 0.0)

    def send_stop_signals(self):
        """Para todos los movimientos de brazos."""
        self.publish_arm_positions(0.0, 0.0)
        self.get_logger().info("Señales de parada enviadas a los brazos.")

def main(args=None):
    rclpy.init(args=args)
    node = OrionArmsControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado - cerrando nodo.")
    finally:
        node.send_stop_signals()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
