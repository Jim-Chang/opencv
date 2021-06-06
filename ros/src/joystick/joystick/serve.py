import rclpy

from .web_server import app
from .joystick_command_node import WebJoystickCommandNode

def main(args=None):
    rclpy.init(args=args)
    publisher = WebJoystickCommandNode()
    
    app.joystick_cmd_publisher = publisher
    app.run(host='0.0.0.0', port=1234)
    
    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()