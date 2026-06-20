import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Share directory path for localization_system
    pkg_share = get_package_share_directory('localization_system')
    
    # Launch arguments for CAN motor
    can_device_arg = DeclareLaunchArgument(
        'can_device',
        default_value='can0',
        description='CAN interface device name'
    )
    motor_id_arg = DeclareLaunchArgument(
        'motor_id',
        default_value='1',
        description='CAN ID of the motor (1-32)'
    )
    torque_limit_arg = DeclareLaunchArgument(
        'torque_limit',
        default_value='1500',
        description='Torque/current limit for the motor'
    )

    # Include the main localization launch file from the installed package
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        )
    )
    
    # CAN Motor control node
    motor_node = Node(
        package='motor_control',
        executable='can_motor_node',
        name='can_motor_node',
        output='screen',
        parameters=[{
            'can_device': LaunchConfiguration('can_device'),
            'motor_id': LaunchConfiguration('motor_id'),
            'torque_limit': LaunchConfiguration('torque_limit'),
            'poll_rate': 50.0
        }]
    )
    
    # Foxglove Bridge node for data visualization
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000
        }]
    )
    
    return LaunchDescription([
        can_device_arg,
        motor_id_arg,
        torque_limit_arg,
        localization_launch,
        motor_node,
        foxglove_bridge_node
    ])
