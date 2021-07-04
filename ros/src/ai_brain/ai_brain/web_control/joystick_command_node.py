import json

import rclpy
from rclpy.node import Node
from jbot_msgs.msg import Motor as MotorMsg
from std_msgs.msg import String

class WebJoystickCommandNode(Node):
    
    def __init__(self):
        super().__init__('web_joystick_cmd_node')
        self._motor_ctrl_pub = self.create_publisher(MotorMsg, 'motor_ctrl', 10)
        self._rec_ctrl_pub = self.create_publisher(String, 'data_collector_ctrl', 10)
        self._auto_drive_ctrl_pub = self.create_publisher(String, 'auto_drive_ctrl', 10)
        
    def pub_motor_cmd(self, speed, diff):
        msg = MotorMsg()
        msg.speed = speed
        msg.diff = diff
        self._motor_ctrl_pub.publish(msg)

    def pub_rec_cmd(self, is_rec):
        msg = String()
        msg.data = str(is_rec)
        self._rec_ctrl_pub.publish(msg)

    def pub_auto_drive_cmd(self, is_enable):
        msg = String()
        msg.data = str(is_enable)
        self._auto_drive_ctrl_pub.publish(msg)
        
        
        