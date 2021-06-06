import rclpy

from .motor_control_node import MotorControlNode
from .pico_motor import PicoMotor
from .i2c_helper import I2CHelper

def main(args=None):
    print("Motor Node start serve")
    rclpy.init(args=args)
    
    i2c = I2CHelper()
    motor = PicoMotor(i2c)
    motor_ctrl_node = MotorControlNode(motor)
    
    rclpy.spin(motor_ctrl_node)
    
    motor_ctrl_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()