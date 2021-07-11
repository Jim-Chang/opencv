from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from PIL import Image
from io import BytesIO

from sensor_msgs.msg import Image as ImageMsg
from jbot_msgs.msg import Motor as MotorMsg
from ai_brain.utils import im_msg_2_im_np, save_img_2_bytes, write_text_on_im


class ImageReceiveNode(Node):

    def __init__(self):
        super().__init__('joystick__image_receive')
        
        self.img_msg = None
        self.predict_motor_msg = None
        self._img_sub = self.create_subscription(ImageMsg, 'video_source/raw', self.img_cb, qos_profile_sensor_data)
        self._auto_drive_predict_sub = self.create_subscription(MotorMsg, 'auto_drive/predict', self.auto_drive_predict, qos_profile_sensor_data)

    def img_cb(self, msg):
        self.img_msg = msg

    def auto_drive_predict(self, msg):
        self.predict_motor_msg = msg
        
    def gen_frames(self):
        while True:
            if self.img_msg != None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + self._get_img_bytes() + b'\r\n')

    def _get_img_bytes(self):
        img = im_msg_2_im_np(self.img_msg)

        if self.predict_motor_msg:
            write_text_on_im(img, f'AI diff: {self.predict_motor_msg.diff}')

        return save_img_2_bytes(img, resize=0.5)
