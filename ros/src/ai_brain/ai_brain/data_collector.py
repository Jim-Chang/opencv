import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from PIL import Image
from uuid import uuid1

from ai_brain.utils import logging
from sensor_msgs.msg import Image as ImageMsg
from jbot_msgs.msg import Motor as MotorMsg
from std_msgs.msg import String

from PIL import Image
from uuid import uuid1

from .utils import prepare_folders, get_key, get_im_file_path, save_image, im_msg_2_im_np, load_motor_im_map, save_motor_im_map


class DataCollectorNode(Node):

    data_folder_path = '/data'
    im_folder_path = f'{data_folder_path}/img'
    motor_im_map_fpath = f'{data_folder_path}/motor_im_map.json'

    def __init__(self):
        super().__init__('data_collector')

        logging.info('create folders')
        prepare_folders(self.im_folder_path)

        self.last_motor_speed = 0
        self.last_motor_diff = 0
        self.is_rec = False

        self.motor_im_map = load_motor_im_map(self.motor_im_map_fpath)

        logging.info('initial subscribe')
        # Image subscriber from cam2image
        self.im_sub = self.create_subscription(ImageMsg, 'video_source/raw', self.im_cb, qos_profile_sensor_data)
        self.motor_ctrl_sub = self.create_subscription(MotorMsg, 'motor_ctrl', self.motor_ctrl_cb, qos_profile_sensor_data)
        self.collector_ctrl_sub = self.create_subscription(String, 'data_collector_ctrl', self.collector_ctrl_cb, qos_profile_sensor_data)

        logging.info('data collector init, wait ctrl cmd to start rec...')

    def im_cb(self, msg):
        if self.is_rec:
            key = get_key()
            im_file_path = get_im_file_path(self.im_folder_path, key)

            img = im_msg_2_im_np(msg)
            save_image(img, im_file_path)

            self.motor_im_map[key] = [self.last_motor_speed, self.last_motor_diff]
            save_motor_im_map(self.motor_im_map_fpath, self.motor_im_map)

    def motor_ctrl_cb(self, msg):
        self.last_motor_speed = msg.speed
        self.last_motor_diff = msg.diff

    def collector_ctrl_cb(self, msg):
        logging.info(f'Receive collector ctrl: {msg.data}')
        self.is_rec = msg.data == 'true'


def main(args=None):
    rclpy.init(args=args)

    node = DataCollectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logging.info('force stop...')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()