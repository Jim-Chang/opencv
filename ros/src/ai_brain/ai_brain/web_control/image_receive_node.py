from rclpy.node import Node
import numpy as np
from PIL import Image
from io import BytesIO

from sensor_msgs.msg import Image as ImageMsg
from ai_brain.utils import im_msg_2_im_np, save_img_2_bytes


class ImageReceiveNode(Node):

    def __init__(self):
        super().__init__('joystick__image_receive')
        
        self.img_msg = None
        self.sub = self.create_subscription(ImageMsg, 'video_source/raw', self.img_cb, 10)

    def img_cb(self, msg):
        self.img_msg = msg
        
    def gen_frames(self):
        while True:
            if self.img_msg != None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + self._get_img_bytes() + b'\r\n')

    def _get_img_bytes(self):
        img = im_msg_2_im_np(self.img_msg)
        return save_img_2_bytes(img, resize=0.5)
