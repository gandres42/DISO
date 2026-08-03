import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class SonarCompressorNode(Node):
    def __init__(self):
        super().__init__('sonar_compressor_node')

        self.subscription = self.create_subscription(
            Image,
            '/oculus/drawn_sonar',
            self.image_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/son/compressed',
            10
        )

        self.bridge = CvBridge()

        self.get_logger().info('Republishing /oculus/drawn_sonar to /son (BGR -> JPEG)...')

    def image_callback(self, msg):
        try:
            # 1. Force the conversion to BGR8.
            # If the source is already BGR, it acts as a passthrough.
            # If the source was incorrectly tagged as RGB, this safely handles it for OpenCV.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 2. Compress the image to JPEG (OpenCV expects BGR natively for this)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            success, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)

            if success:
                # 3. Create and populate the CompressedImage message
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header

                # Explicitly define the format so decompressores don't swap red and blue
                compressed_msg.format = "bgr8; jpeg compressed bgr8"

                compressed_msg.data = encoded_image.tobytes()

                # 4. Publish
                self.publisher_.publish(compressed_msg)
            else:
                self.get_logger().error('Image compression failed.')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    compressor_node = SonarCompressorNode()

    try:
        rclpy.spin(compressor_node)
    except KeyboardInterrupt:
        pass
    finally:
        compressor_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()