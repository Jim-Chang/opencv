import rclpy

from .motor_control_node import MotorControlNode
from .pico_motor import PicoMotor
from ai_brain.i2c_helper import I2CHelper

def main(args=None):
    rclpy.init(args=args)
    
    i2c = I2CHelper()
    motor = PicoMotor(i2c)
    motor_ctrl_node = MotorControlNode(motor)
    
    try:
        rclpy.spin(motor_ctrl_node)
    except KeyboardInterrupt:
        print('stop...')
    
    motor.stop()
    motor_ctrl_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()