import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.serial_port = serial.Serial('/dev/ttyACMa1', 115200, timeout=0.1)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info("conectat la /dev/ttyACM0")

    def send_command(self, pwmR, pwmL, revR, revL):
        # Construim string-ul comenzii in formatul Arduino
        cmd_str = f"<0,2,{pwmR},{pwmL},{int(revR)},{int(revL)}>"
        self.serial_port.write(cmd_str.encode())
        self.get_logger().info(f"Sent command to Arduino: {cmd_str}")

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        pwm = 15

        if linear > 0.1:
            # Mers înainte
            self.send_command(pwm, pwm, False, True)
        elif linear < -0.1:
            self.send_command(pwm+10, pwm, True, True)
        elif angular > 0.1:
            self.send_command(pwm, pwm,  True, False)
        elif angular < -0.1:
            self.send_command(pwm, pwm+10, False, False)
        else:

            self.send_command(0, 0, False, False)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
