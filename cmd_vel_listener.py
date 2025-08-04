import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("conectat la /dev/ttyACM0")

    def cmd_vel_callback(self, msg):
    
        pwmR = int(abs(msg.linear.x + msg.angular.z) * 255 / 0.5) 
        pwmL = int(abs(msg.linear.x - msg.angular.z) * 255 / 0.5)
        revR = 1 if (msg.linear.x + msg.angular.z) < 0 else 0
        revL = 1 if (msg.linear.x - msg.angular.z) < 0 else 0
        cmd = f"<0,2,{pwmR},{pwmL},{revR},{revL}>"
        self.serial_port.write(cmd.encode())

    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()