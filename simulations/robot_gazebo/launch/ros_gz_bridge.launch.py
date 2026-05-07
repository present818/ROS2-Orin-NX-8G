from launch import LaunchDescription,LaunchService
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true').perform(context)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value=use_sim_time)

    nav = LaunchConfiguration('nav', default='false').perform(context)
    nav_arg = DeclareLaunchArgument('nav',default_value=nav)

    remappings_default = [("/odom/tf", "tf")]
    if nav == 'true':
        remappings_default += [("/controller/cmd_vel", "/cmd_vel")]

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                # Velocity command (ROS2 -> Gazebo)
                '/controller/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                # Odometry (Gazebo -> ROS2)
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # TF (Gazebo -> ROS2)
                '/odom/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                # Clock (Gazebo -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Joint states (Gazebo -> ROS2)
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                # Lidar (Gazebo -> ROS2)
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                # IMU (Gazebo -> ROS2)
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                # Camera (Gazebo -> ROS2)
                '/depth_cam/depth_cam@sensor_msgs/msg/Image[gz.msgs.Image',
                '/depth_cam/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
        remappings=remappings_default,
        output='screen'
    )


    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='screen',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'])
    return [
        use_sim_time_arg,
        bridge,
        map_static_tf
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])



if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
