from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_imu',
            executable='robot_imu_node',
            name='robot_imu',
            parameters=[{
                'can_id': 0,
                'canbus': 'canivore',  # or your CANivore's name
                'publish_rate': 100.0
            }]
        )
    ])