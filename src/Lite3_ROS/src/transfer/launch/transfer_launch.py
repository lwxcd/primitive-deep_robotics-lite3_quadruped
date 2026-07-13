
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    jetson2app = Node(
        package="transfer",
        executable="jetson2app",
        parameters=[
            {'local_port': int(os.environ.get('TRANSFER_APP_PORT', '43899'))}
        ],
        output='screen'
    )
    jetson2motion = Node(
        package="transfer",
        executable="jetson2motion",
        parameters=[
            {'target_ip': os.environ.get('TRANSFER_TARGET_IP', '192.168.1.120')},
            {'target_port': int(os.environ.get('TRANSFER_TARGET_PORT', '43893'))},
            {'local_port': int(os.environ.get('TRANSFER_LOCAL_PORT', '43897'))}
        ],
        output='screen'
    )
    sensor_checker = Node(
        package="transfer",
        executable="sensor_checker",
        parameters=[
            {'is_debug': True}
        ],
        output='screen'
    )
    launch_description = LaunchDescription(
        [jetson2app, jetson2motion, sensor_checker])
    return launch_description
