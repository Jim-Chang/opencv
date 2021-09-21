import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from PIL import Image
import cv2
from io import BytesIO

from sensor_msgs.msg import Image as ImageMsg
from jbot_msgs.msg import Motor as MotorMsg, Distance as DistanceMsg
from std_msgs.msg import String

from ai_brain.utils import logging, im_msg_2_im_np, save_img_2_bytes
from .nvidia_model import SteeringPredictor
from .nvidia_model_2 import SteeringSpeedPredictor


class DriveNode(Node):

    def __init__(self, motor):
        super().__init__('auto_drive__drive')
        self.motor = motor

        self.is_enable = False
        self.manual_ctrl = False
        self.interfer_with_sensor = False

        self.img = None
        self.front_distance = 1000

        self._img_sub = self.create_subscription(ImageMsg, 'video_source/raw', self.img_cb, qos_profile_sensor_data)
        self._auto_drive_ctrl_sub = self.create_subscription(String, 'auto_drive/ctrl', self.auto_drive_cb, qos_profile_sensor_data)
        self._joystick_drive_ctrl_sub = self.create_subscription(MotorMsg, 'motor/ctrl', self.joystick_drive_cb, qos_profile_sensor_data)
        self._distance_sub = self.create_subscription(MotorMsg, 'sensor/distance', self.distance_cb, qos_profile_sensor_data)

        self._auto_drive_predict_pub = self.create_publisher(MotorMsg, 'auto_drive/predict', qos_profile_sensor_data)

        logging.info('init predictor...')
        # self.predictor = SteeringPredictor('0703.pth')
        self.predictor = SteeringSpeedPredictor('0713_4.pth')
        logging.info('init predictor successfully!')

        logging.info('Drive Node Start')

    def auto_drive_cb(self, msg):
        logging.info(f'Receive auto drive ctrl: {msg.data}')
        self.is_enable = msg.data == 'true'
        self.stop_motor()

    def img_cb(self, msg):
        if self.is_enable:
            self.img = Image.fromarray(im_msg_2_im_np(msg))
            self._do_driving()

        else:
            logging.info('not enable auto drive')

    def joystick_drive_cb(self, msg):
        logging.info(f'[Receive joystick ctrl] speed: {msg.speed}, diff: {msg.diff}')
        self.motor.go(msg.speed, msg.diff)

        self.manual_ctrl = not (msg.speed == 0 and msg.diff == 0)

    def distance_cb(self, msg):
        self.front_distance = msg.front

    def stop_motor(self):
        self.motor.stop()

    def _pub_auto_drive_perdict(self, speed, diff):
        msg = MotorMsg()
        msg.speed = speed
        msg.diff = diff
        self._auto_drive_predict_pub.publish(msg)

    def _do_driving(self):
        steering, speed = self._do_predict(self.img)

        # pub predict first
        self._pub_auto_drive_perdict(int(speed), int(steering))

        # if user has sent control msg then override auto drive
        if not self.manual_ctrl:
            if speed > 0 and self.front_distance < 100:
                self.interfer_with_sensor = True

            if self.interfer_with_sensor and self.front_distance > 200:
                self.interfer_with_sensor = False

            if self.interfer_with_sensor:
                if steering == 0:
                    steering = 50
                else:
                    steering += 50 * (steering / abs(steering))

                logging.info(f'Interfer with sensor, front distance = {self.front_distance}, steering = {steering}')

            self.motor.go(int(speed), int(steering))

    def _do_predict(self, pil_img):
        if isinstance(self.predictor, SteeringPredictor):
            steering = self.predictor.predict(pil_img)
            return steering, 75

        elif isinstance(self.predictor, SteeringSpeedPredictor):
            return self.predictor.predict(pil_img)

        else:
            raise Exception('Predictor is not support')


def main(args=None):
    from ai_brain.motor.pico_motor import PicoMotor
    from ai_brain.i2c_helper import I2CHelper

    rclpy.init(args=args)

    # prepare motor
    i2c = I2CHelper()
    motor = PicoMotor(i2c)
    
    # init drive node
    node = DriveNode(motor)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logging.info('force stop...')

    node.stop_motor()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
