import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import time

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch1 = sys.stdin.read(1)
        if ch1 == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return ch1 + ch2 + ch3
        else:
            return ch1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class TurtleTeleop(Node):
    def __init__(self):
        super().__init__('turtle_teleop')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Controles: flechas para moverse, '.' para salir, teclas A/H/P/M/J/B para dibujar letras")

    def run(self):
        while rclpy.ok():
            key = get_key()
            msg = Twist()

            if key == '\x1b[A':
                msg.linear.x = 2.0
            elif key == '\x1b[B':
                msg.linear.x = -2.0
            elif key == '\x1b[D':
                msg.angular.z = 2.0
            elif key == '\x1b[C':
                msg.angular.z = -2.0
            elif key == '.':
                self.get_logger().info("Finalizando...")
                break
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
                msg.linear.x = 0.0
                msg.angular.z = 0.0

            self.publisher_.publish(msg)

        self.publisher_.publish(Twist())  # Stop turtle

    def send_move(self, linear=0.0, angular=0.0, duration=1.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        time.sleep(duration)
        self.publisher_.publish(Twist())
        time.sleep(0.2)

    # Simulación de letras básicas:

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
        
def main(args=None):
    rclpy.init(args=args)
    teleop_node = TurtleTeleop()
    teleop_node.run()
    teleop_node.destroy_node()
    rclpy.shutdown()

