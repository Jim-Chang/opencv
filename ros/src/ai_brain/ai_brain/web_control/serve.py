import rclpy
import threading

from .web_server import app
from .joystick_command_node import WebJoystickCommandNode
from .image_receive_node import ImageReceiveNode


def main(args=None):
    rclpy.init(args=args)
    cmd_pub = WebJoystickCommandNode()
    im_receive = ImageReceiveNode()
    
    app.joystick_cmd_publisher = cmd_pub
    app.image_receiver = im_receive

    t = threading.Thread(target=app.run, kwargs=dict(host='0.0.0.0', port=1234))
    t.start()

    try:
        rclpy.spin(im_receive)
    except KeyboardInterrupt:
        print('stop ros node..., press ctrl+c again to stop web server')

    cmd_pub.destroy_node()
    im_receive.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()