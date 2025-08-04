#Cmd_vel
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
from tf_transformations import quaternion_from_euler
import math

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.update_odometry)
        self.get_logger().info("Connected to /dev/ttyACM0")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_vel = Twist()
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        # Trimite comenzi catre Arduino: <0,2,pwmR,pwmL,revR,revL>
        pwmR = int(abs(msg.linear.x + msg.angular.z) * 255 / 0.5)  # Ajusteaza scalarea
        pwmL = int(abs(msg.linear.x - msg.angular.z) * 255 / 0.5)
        revR = 1 if (msg.linear.x + msg.angular.z) < 0 else 0
        revL = 1 if (msg.linear.x - msg.angular.z) < 0 else 0
        cmd = f"<0,2,{pwmR},{pwmL},{revR},{revL}>"
        self.serial_port.write(cmd.encode())
        self.last_vel = msg

    def update_odometry(self):
        # Estimeaza odometria din cmd_vel
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        v_x = self.last_vel.linear.x
        v_th = self.last_vel.angular.z
        self.x += v_x * math.cos(self.theta) * dt
        self.y += v_x * math.sin(self.theta) * dt
        self.theta += v_th * dt

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = v_th
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

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

#Imu
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import adafruit_mpu6050
import board
import busio
from tf_transformations import quaternion_from_euler

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_mpu6050.MPU6050(self.i2c)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("IMU node started")

    def timer_callback(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        accel = self.imu.acceleration
        gyro = self.imu.gyro
        # Presupunem roll, pitch, yaw calculate din gyro (simplificat)
        roll, pitch, yaw = gyro[0], gyro[1], gyro[2]
        q = quaternion_from_euler(roll, pitch, yaw)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        self.imu_publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


#URDF_ROBOT

<robot name="toycar">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/> <!-- Lungime, latime, inaltime -->
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>
  <link name="laser_frame"/>
  <joint name="base_to_laser" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  <link name="imu_link"/>
  <joint name="base_to_imu" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>

#ekf_config

ekf_filter_node:
  ros__parameters:
    transform_time_offset: 0.1
    transform_timeout: 0.0
    two_d_mode: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: /odom
    odom0_config: [true, true, false, false, false, true, false, false, false, false, false, false, false, false, false]
    odom0_differential: false
    imu0: /imu/data
    imu0_config: [false, false, false, true, true, true, false, false, false, true, true, true, false, false, false]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true