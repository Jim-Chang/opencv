import rclpy
import threading

from .web_server import app
from .joystick_command_node import WebJoystickCommandNode
from .image_receive_node import ImageReceiveNode

def web_server():
    app.run(host='0.0.0.0', port=1234)

def main(args=None):
    rclpy.init(args=args)
    cmd_pub = WebJoystickCommandNode()
    im_receive = ImageReceiveNode()
    
    app.joystick_cmd_publisher = cmd_pub
    app.image_receiver = im_receive

    t = threading.Thread(target=web_server)
    t.start()
    # app.run(host='0.0.0.0', port=1234)

    rclpy.spin(im_receive)

    cmd_pub.destroy_node()
    im_receive.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()