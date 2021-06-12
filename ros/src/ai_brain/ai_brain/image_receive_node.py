import rclpy
from rclpy.node import Node
import numpy as np
from PIL import Image
from uuid import uuid1

from sensor_msgs.msg import Image as ImageMsg

from .utils import prepare_folders, get_key, get_im_file_path, save_image, im_msg_2_im_np, load_motor_im_map, save_motor_im_map



class ImageReceiveNode(Node):

    data_folder_path = '/data'
    im_folder_path = f'{data_folder_path}/img'

    def __init__(self):
        super().__init__('image_receive')
        prepare_folders(self.im_folder_path)
        
        # Image subscriber from cam2image
        self.sub = self.create_subscription(ImageMsg, 'video_source/raw', self.read_cam_callback, 10)

    def read_cam_callback(self, msg):
        key = get_key()
        im_file_path = get_im_file_path(self.im_folder_path, key)

        img = im_msg_2_im_np(msg)
        save_image(img, im_file_path)
        print(f'[ImageReceiveNode]: receive image {key}')


def main(args=None):
    rclpy.init(args=args)

    node = ImageReceiveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('force stop...')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


