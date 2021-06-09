import rclpy
from rclpy.node import Node
import numpy as np
from PIL import Image
from uuid import uuid1

from sensor_msgs.msg import Image as ImageMsg


class ImageReceiveNode(Node):

    def __init__(self):
        super().__init__('image_receive')
        
        # Image subscriber from cam2image
        self.sub = self.create_subscription(ImageMsg, 'video_source/raw', self.read_cam_callback, 10)

    def read_cam_callback(self, msg):
        img = np.asarray(msg.data)
        self.image = np.reshape(img, (msg.height, msg.width, 3))
        print('[ImageReceiveNode]: receive image')
        self.save_image(self.image)
#         self.annotated_image = self.execute()

#         image_msg = self.image_np_to_image_msg(self.annotated_image)
#         self.image_pub.publish(image_msg)
#         if self.show_image_param:
#             cv2.imshow('frame', self.annotated_image)
#             cv2.waitKey(1)
            
    def image_np_to_image_msg(self, image_np):
        image_msg = ImageMsg()
        image_msg.height = image_np.shape[0]
        image_msg.width = image_np.shape[1]
        image_msg.encoding = 'bgr8'
        image_msg.data = image_np.tostring()
        image_msg.step = len(image_msg.data) // image_msg.height
        image_msg.header.frame_id = 'map'
        return image_msg
    
    def save_image(self, image):
        image_path = f'/src/img/{str(uuid1())}.jpg'
        with open(image_path, 'wb') as f:
            Image.fromarray(image).save(f, 'jpeg')


def main(args=None):
    rclpy.init(args=args)

    node = ImageReceiveNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


