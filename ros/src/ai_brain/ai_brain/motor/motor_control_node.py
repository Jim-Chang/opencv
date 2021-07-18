from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ai_brain.utils import logging

from jbot_msgs.msg import Motor as MotorMsg


class MotorControlNode(Node):

    def __init__(self, motor):
        super().__init__('motor_control')
        self.subscription = self.create_subscription(
            MotorMsg,
            'motor/ctrl',
            self._sub_cb,
            qos_profile_sensor_data)
        self.motor = motor
        
        logging.info('Motor Control Node start')

    def _sub_cb(self, msg):
        logging.info(f'[Receive ctrl] speed: {msg.speed}, diff: {msg.diff}')
        self.motor.go(msg.speed, msg.diff)
        
