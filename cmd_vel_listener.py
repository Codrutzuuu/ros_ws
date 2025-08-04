import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
from tf_transformations import quaternion_from_euler

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.read_odometry)
        self.get_logger().info("conectat la /dev/ttyACM0")

    def cmd_vel_callback(self, msg):
        pwmR = int(abs(msg.linear.x + msg.angular.z) * 255 / 0.5)  # Ajusteaza scalarea
        pwmL = int(abs(msg.linear.x - msg.angular.z) * 255 / 0.5)
        revR = 1 if (msg.linear.x + msg.angular.z) < 0 else 0
        revL = 1 if (msg.linear.x - msg.angular.z) < 0 else 0
        cmd = f"<0,2,{pwmR},{pwmL},{revR},{revL}>"
        self.serial_port.write(cmd.encode())

    def read_odometry(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode().strip()
            if line.startswith("<odom"):
                try:
                    _, x, y, theta = map(float, line.strip("<>").split(",")[1:])
                    odom = Odometry()
                    odom.header.stamp = self.get_clock().now().to_msg()
                    odom.header.frame_id = 'odom'
                    odom.child_frame_id = 'base_link'
                    odom.pose.pose.position.x = x
                    odom.pose.pose.position.y = y
                    q = quaternion_from_euler(0, 0, theta)
                    odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                    self.odom_pub.publish(odom)

                    t = TransformStamped()
                    t.header.stamp = odom.header.stamp
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = x
                    t.transform.translation.y = y
                    t.transform.rotation = odom.pose.pose.orientation
                    self.tf_broadcaster.sendTransform(t)
                except Exception as e:
                    self.get_logger().error(f"Failed to parse odometry: {e}")

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