import rclpy
import threading

from .web_server import app
from .joystick_command_node import WebJoystickCommandNode
from .image_receive_node import ImageReceiveNode


def main(args=None):
    rclpy.init(args=args)

    # prepare nodes
    cmd_pub = WebJoystickCommandNode()
    im_receive = ImageReceiveNode()

    # start ros executor in thread
    executor = rclpy.get_global_executor()
    executor.add_node(cmd_pub)
    executor.add_node(im_receive)
    t = threading.Thread(target=executor.spin)
    t.start()
    
    # register node to flask app
    app.joystick_cmd_publisher = cmd_pub
    app.image_receiver = im_receive

    # start flask app and stuck main thread
    app.run(host='0.0.0.0', port=1234)

    # destroy nodes and stop ros executor
    print('stop ros nodes...')
    cmd_pub.destroy_node()
    im_receive.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()