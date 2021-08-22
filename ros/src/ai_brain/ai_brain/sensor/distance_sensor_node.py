from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ai_brain.utils import logging
import time

from jbot_msgs.msg import Distance as DistanceMsg


class DistanceSensorNode(Node):

    def __init__(self, sensor):
        super().__init__('distance_sensor_node')
        self._sensor = sensor
        self._distance_pub = self.create_publisher(DistanceMsg, 'sensor/distance', qos_profile_sensor_data)
        
        logging.info('Distance Sensor Node start')
        
    def run(self):
        while True:
            msg = DistanceMsg()
            
            front = self._sensor.get_front_distance()
            if front:
                msg.front = front

            self._distance_pub.publish(msg)
            time.sleep(0.1)


    

        
