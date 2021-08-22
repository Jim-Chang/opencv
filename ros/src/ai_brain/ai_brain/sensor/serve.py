import rclpy

from .distance_sensor_node import DistanceSensorNode
from .distance_sensor import DistanceSensor
from ai_brain.i2c_helper import I2CHelper

def main(args=None):
    rclpy.init(args=args)
    
    i2c = I2CHelper()
    dsensor = DistanceSensor(i2c)
    dsensor_node = DistanceSensorNode(dsensor)
    
    try:
        dsensor_node.run()
    except KeyboardInterrupt:
        print('stop...')
    
    dsensor_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()