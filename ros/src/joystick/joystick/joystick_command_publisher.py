import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WebJoystickCommandPublisher(Node):
    
    def __init__(self):
        super().__init__('web_joystick_cmd_publisher')
        self._publisher = self.create_publisher(String, 'joystick', 10)
        
    def pub_cmd(self, speed, diff):
        msg = String()
        msg.data = json.dumps({
            'speed': speed,
            'diff': diff,
        })
        self._publisher.publish(msg)
        
        
        