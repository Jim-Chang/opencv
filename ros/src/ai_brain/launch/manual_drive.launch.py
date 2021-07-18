from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_deep_learning',
            node_executable='video_source',
            name='video_src_node',
            arguments=['--ros-args', '-p', 'resource:="csi://0"', '-p', 'width:=640', '-p', 'height:=360', '-p', 'framerate:=30.']
        ),
        Node(
            package='ai_brain',
            node_executable='motor',
            name='motor_node'
        ),
        Node(
            package='ai_brain',
            node_executable='web_server',
            name='joystick_node'
        ),
    ])