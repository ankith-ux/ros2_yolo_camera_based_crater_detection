from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_crater_detector',
            executable='detector_node',
            name='yolo_crater_detector',
            parameters=[{'model_path': 'models/best.pt'}],
            output='screen'
        )
    ])

