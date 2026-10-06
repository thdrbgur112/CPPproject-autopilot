from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # webots_node는 Webots 월드의 extern controller로 붙으므로 Webots를 먼저 실행해야 함
    return LaunchDescription([
        Node(package='auto_pkg', executable='webots_node', output='screen'),
        Node(package='auto_pkg', executable='cv_node', output='screen'),
        Node(package='auto_pkg', executable='cnn_node.py', output='screen'),
        Node(package='auto_pkg', executable='mpc_node', output='screen'),
    ])
