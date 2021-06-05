import rclpy

from .web_server import app
from .joystick_command_publisher import WebJoystickCommandPublisher

def main(args=None):
    rclpy.init(args=args)
    
    publisher = WebJoystickCommandPublisher()
    app.joystick_cmd_publisher = publisher
    
    app.run(host='0.0.0.0', port=1234)
    
if __name__ == '__main__':
    main()