# Importamos las librerías necesarias
import rclpy  # Librería principal de ROS2 en Python
from rclpy.node import Node  # Clase base para crear nodos
from geometry_msgs.msg import Twist  # Mensaje para controlar movimiento
import sys
import tty
import termios
import time

# Función para leer una tecla presionada del teclado sin necesidad de dar "Enter"
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)  # Modo de lectura en crudo (sin buffers)
        ch1 = sys.stdin.read(1)  # Leer primer carácter
        if ch1 == '\x1b':  # Si es una tecla especial (como flechas)
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return ch1 + ch2 + ch3  # Devuelve la combinación
        else:
            return ch1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Restaurar configuración

# Definición del nodo de teleoperación de la tortuga
class TurtleTeleop(Node):
    def _init_(self):
        super()._init_('turtle_teleop')
        # Creamos un publicador que enviará mensajes de velocidad
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Controles: flechas para moverse, '.' para salir, teclas A/H/P/M/J/B para dibujar letras")

    # Método principal de ejecución
    def run(self):
        while rclpy.ok():
            key = get_key()  # Esperar una tecla
            msg = Twist()  # Crear mensaje vacío de movimiento

            # Control manual con flechas
            if key == '\x1b[A':  # Flecha arriba
                msg.linear.x = 2.0
            elif key == '\x1b[B':  # Flecha abajo
                msg.linear.x = -2.0
            elif key == '\x1b[D':  # Flecha izquierda
                msg.angular.z = 2.0
            elif key == '\x1b[C':  # Flecha derecha
                msg.angular.z = -2.0

            # Tecla de salida
            elif key == '.':
                self.get_logger().info("Finalizando...")
                break

            # Teclas para dibujar letras
            elif key.upper() == 'A':
                self.draw_A()
            elif key.upper() == 'H':
                self.draw_H()
            elif key.upper() == 'P':
                self.draw_P()
            elif key.upper() == 'M':
                self.draw_M()
            elif key.upper() == 'J':
                self.draw_J()
            elif key.upper() == 'B':
                self.draw_B()
            else:
                # Si no es una tecla válida, detener la tortuga
                msg.linear.x = 0.0
                msg.angular.z = 0.0

            # Publicar el mensaje
            self.publisher_.publish(msg)

        # Al salir del ciclo, detener la tortuga
        self.publisher_.publish(Twist())

    # Método auxiliar para enviar movimientos de velocidad por un tiempo determinado
    def send_move(self, linear=0.0, angular=0.0, duration=1.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        time.sleep(duration)
        self.publisher_.publish(Twist())
        time.sleep(0.2)

    # Métodos para dibujar letras específicas
    def draw_A(self):
        self.get_logger().info("Dibujando la letra A")
        self.send_move(2.0, 0.0, 1.0)
        self.send_move(0.0, 2.36, 2.0)
        self.send_move(2.0, 0.0, 2.0)
        self.send_move(0.0, 3.14, 2.0)
        self.send_move(2.0, 0.0, 0.5)
        self.send_move(0.0, -1.5, 2.0)
        self.send_move(2.0, 0.0, 1.0)

    def draw_H(self):
        self.get_logger().info("Dibujando la letra H")
        self.send_move(2.0, 0.0, 1.0)
        self.send_move(-1.0, 0.0, 1.0)
        self.send_move(0.0, 1.57, 2.0)
        self.send_move(1.0, 0.0, 1.0)
        self.send_move(0.0, 1.57, 2.0)
        self.send_move(1.0, 0.0, 1.0)
        self.send_move(-2.0, 0.0, 1.0)

    def draw_P(self):
        self.get_logger().info("Dibujando la letra P")
        self.send_move(2.0, 0.0, 1.0)
        self.send_move(0.0, -1.57, 1.0)
        self.send_move(1.0, -3.14, 3.0)

    def draw_M(self):
        self.get_logger().info("Dibujando la letra M")
        self.send_move(2.0, 0.0, 1.0)
        self.send_move(0.0, -2.35, 2.0)
        self.send_move(1.0, 0.0, 1.0)
        self.send_move(0.0, 1.57, 2.0)
        self.send_move(1.0, 0.0, 1.0)
        self.send_move(0.0, -2.36, 2.0)
        self.send_move(2.0, 0.0, 1.0)

    def draw_J(self):
        self.get_logger().info("Dibujando la letra J")
        self.send_move(1.0, 1.5, 1.0)
        self.send_move(2.0, 0.0, 1.0)
        self.send_move(0.0, 1.57, 2.0)
        self.send_move(1.0, 0.0, 1.0)
        self.send_move(-2.0, 0.0, 1.0)

    def draw_B(self):
        self.get_logger().info("Dibujando la letra B")
        self.send_move(2.0, 0.0, 1.0)
        self.send_move(0.0, -1.57, 1.0)
        self.send_move(1.0, -3.14, 3.0)
        self.send_move(-1.0, -3.14, 3.0)

# Función main de ROS 2
def main(args=None):
    rclpy.init(args=args)  # Inicializar ROS
    teleop_node = TurtleTeleop()  # Crear instancia del nodo
    teleop_node.run()  # Ejecutar
    teleop_node.destroy_node()  # Destruir nodo al terminar
    rclpy.shutdown()  # Cerrar ROS
