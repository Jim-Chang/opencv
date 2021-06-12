from uuid import uuid1
from PIL import Image
import numpy as np
import os
import json

from sensor_msgs.msg import Image as ImageMsg

def prepare_folders(folders):
    if type(folders) is list:
        for path in folders:
            os.makedirs(path, exist_ok=True)
    else:
        os.makedirs(folders, exist_ok=True)


def get_key():
    return str(uuid1())


def get_im_file_path(folder_path, key):
    return f'{folder_path}/{key}.jpg'


def save_image(image, file_path):
    with open(file_path, 'wb') as f:
        Image.fromarray(image).save(f, 'jpeg')


def im_np_2_im_msg(im_np):
    im_msg = ImageMsg()
    im_msg.height = im_np.shape[0]
    im_msg.width = im_np.shape[1]
    im_msg.encoding = 'bgr8'
    im_msg.data = im_np.tostring()
    im_msg.step = len(im_msg.data) // im_msg.height
    im_msg.header.frame_id = 'map'
    return im_msg


def im_msg_2_im_np(msg):
    data = np.asarray(msg.data)
    return np.reshape(data, (msg.height, msg.width, 3))


def load_motor_im_map(file_path):
    try:
        with open(file_path, 'r') as f:
            data = f.read()
            return json.loads(data)
    except:
        print('motor im map json file not found, create a new file')
        return dict()


def save_motor_im_map(file_path, data_map):
    with open(file_path, 'w') as f:
        data = json.dumps(data_map)
        f.write(data)