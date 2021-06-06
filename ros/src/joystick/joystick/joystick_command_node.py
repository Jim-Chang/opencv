import json

import rclpy
from rclpy.node import Node
from jbot_msgs.msg import Motor as MotorMsg

class WebJoystickCommandNode(Node):
    
    def __init__(self):
        super().__init__('web_joystick_cmd_node')
        self._motor_ctrl_pub = self.create_publisher(MotorMsg, 'motor_ctrl', 10)
        
    def pub_cmd(self, speed, diff):
        msg = MotorMsg()
        msg.speed = speed
        msg.diff = diff
        self._motor_ctrl_pub.publish(msg)
        
        
        