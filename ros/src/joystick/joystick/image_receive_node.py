from rclpy.node import Node
import numpy as np
from PIL import Image
from io import BytesIO

from sensor_msgs.msg import Image as ImageMsg


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
        data = np.asarray(self.img_msg.data)
        img = np.reshape(data, (self.img_msg.height, self.img_msg.width, 3))
        
        bytes_io = BytesIO()
        Image.fromarray(img).save(bytes_io, 'jpeg')
        return bytes_io.getvalue()
