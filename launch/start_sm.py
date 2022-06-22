from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smach_viewer',
            namespace='smach',
            executable='smach_viewer_gui.py',
            output='screen',
            name='smach_viewer_gui'
        ),
        Node(
            package='smach_controller',
            namespace='smach',
            executable='start',
            name='smach_controller'
        )
    ])
