from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='allblue2', executable='motor_controller', output='screen'),
        Node(package='allblue2', executable='motor_gui', output='screen'),
    ])
