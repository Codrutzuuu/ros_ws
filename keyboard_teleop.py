import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('SAgeti, Q-iesire')

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()
            twist = Twist()

            if key == 'q':
                self.get_logger().info('Iesire..')
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                rclpy.shutdown()
                return

            if key  == 'w':
                twist.linear.x = 0.5
            elif key == 'a':
                twist.linear.x = -0.5
            elif key  == 's':
                twist.angular.z = 0.5
            elif key == 'd':
                twist.angular.z = -0.5
            else:
            	 twist.linear.x = 0
            	 twist.angular.z = 0

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
