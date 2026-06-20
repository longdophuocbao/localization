import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Share directory paths
    pkg_share = get_package_share_directory('localization_system')
    
    # Launch arguments
    imu_port_arg = DeclareLaunchArgument('imu_port', default_value='/dev/ttyAMA0', description='Serial port of the IMU')
    gps_port_arg = DeclareLaunchArgument('gps_port', default_value='/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00', description='Serial port of the GPS')
    use_sim_gps_arg = DeclareLaunchArgument('use_sim_gps', default_value='false', description='Use simulated GPS instead of real gps_driver')
    mag_dec_arg = DeclareLaunchArgument('magnetic_declination', default_value='0.0', description='Magnetic declination in radians at your location')
    
    # UKF Configuration file path
    ukf_config_path = os.path.join(pkg_share, 'config', 'ukf.yaml')
    
    # Parameters config file path
    params_config_path = os.path.join(pkg_share, 'config', 'localization_params.yaml')
    
    # Nodes
    imu_node = Node(
        package='localization_system',
        executable='imu_driver',
        name='imu_driver',
        output='screen',
        parameters=[
            params_config_path,
            {
                'port': LaunchConfiguration('imu_port')
            }
        ]
    )
    
    gps_node = Node(
        package='localization_system',
        executable='gps_driver',
        name='gps_driver',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_sim_gps')),
        parameters=[
            params_config_path,
            {
                'port': LaunchConfiguration('gps_port')
            }
        ]
    )
    
    sim_gps_node = ExecuteProcess(
        cmd=['python3', '/home/tractor/gps_publisher.py'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim_gps'))
    )
    
    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        parameters=[ukf_config_path],
        arguments=['--ros-args', '--log-level', 'rclcpp:=ERROR']
    )
    
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[{
            'frequency': 30.0,
            'delay': 0.0,
            'magnetic_declination_radians': LaunchConfiguration('magnetic_declination'),
            'yaw_offset': 0.0, # Handled in our custom imu_driver
            'zero_altitude': True,
            'broadcast_utm_transform': True,
            'publish_filtered_gps': True,
            'use_odometry_yaw': False,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom'
        }],
        remappings=[
            ('/gps/fix', '/gps/fix'),
            ('imu', '/imu/data'),
            ('/odometry/filtered', '/odometry/filtered'),
            ('/odometry/gps', '/odometry/gps')
        ]
    )
    
    # Static TF transform publishers (base_link to sensor links)
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    tf_base_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link']
    )

    return LaunchDescription([
        imu_port_arg,
        gps_port_arg,
        use_sim_gps_arg,
        mag_dec_arg,
        imu_node,
        gps_node,
        sim_gps_node,
        ukf_node,
        navsat_transform_node,
        tf_base_to_imu,
        tf_base_to_gps
    ])
