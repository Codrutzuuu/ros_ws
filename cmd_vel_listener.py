import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    package_dir = get_package_share_directory('keyboard_teleop_pkg')
    config_dir = os.path.join(package_dir, 'config')
    urdf_dir = os.path.join(package_dir, 'urdf')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(config_dir, 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='true',
            description='Whether to set the map subscriber QoS to transient local'),

        # RPLIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser_frame', 'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Existing nodes
        Node(
            package='keyboard_teleop_pkg',
            executable='cmd_vel_listener',
            name='cmd_vel_listener',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='keyboard_teleop_pkg',
            executable='imu_node',
            name='imu_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(config_dir, 'slam_toolbox_config.yaml'), {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(config_dir, 'ekf_config.yaml'), {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(os.path.join(urdf_dir, 'robot.urdf')).read()}, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_link_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_link_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Nav2 nodes
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower']}]),
        
        # RViz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])


    ########################

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