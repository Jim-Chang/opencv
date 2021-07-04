import rclpy
from rclpy.node import Node
import numpy as np
from PIL import Image
import cv2
from io import BytesIO

from sensor_msgs.msg import Image as ImageMsg
from jbot_msgs.msg import Motor as MotorMsg
from std_msgs.msg import String

from ai_brain.utils import im_msg_2_im_np, save_img_2_bytes
from .nvidia_model import SteeringPredictor


class DriveNode(Node):

    def __init__(self):
        super().__init__('auto_drive__drive')
        
        self.is_enable = False
        # self.np_img = None
        
        self._img_sub = self.create_subscription(ImageMsg, 'video_source/raw', self.img_cb, 10)
        self._auto_drive_ctrl_sub = self.create_subscription(String, 'auto_drive_ctrl', self.auto_drive_cb, 10)
        
        self._motor_ctrl_pub = self.create_publisher(MotorMsg, 'motor_ctrl', 10)
        self._auto_drive_predict_pub = self.create_publisher(MotorMsg, 'auto_drive_predict', 10)

        print('init perdictor...')
        self.predictor = SteeringPredictor('steering_ep_99.pth')
        print('successfully!')

    def auto_drive_cb(self, msg):
        print(f'Receive auto drive ctrl: {msg.data}')
        self.is_enable = msg.data == 'true'

    def img_cb(self, msg):
        np_img = im_msg_2_im_np(msg)
        steering = self.predictor.predict(Image.fromarray(np_img))

        self._pub_auto_drive_perdict(int(steering))
        if self.is_enable:
            self._pub_motor_cmd(80, int(steering))

    def _pub_motor_cmd(self, speed, diff):
        msg = MotorMsg()
        msg.speed = speed
        msg.diff = diff
        self._motor_ctrl_pub.publish(msg)

    def _pub_auto_drive_perdict(self, diff):
        msg = MotorMsg()
        msg.diff = diff
        self._auto_drive_predict_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = DriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('force stop...')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()