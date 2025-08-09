import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')

        # 声明参数用于设置保存路径
        self.declare_parameter('save_path', '~/code/ros2_ws/images')
        save_path = self.get_parameter('save_path').get_parameter_value().string_value
        # 展开用户目录（~）并确保目录存在
        self.save_path = os.path.expanduser(save_path)
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            self.get_logger().info(f'Created directory: {self.save_path}')

        self.bridge = CvBridge()
        self.image_count = 0

        # 创建订阅者。话题名称需要与AirSim ROS2 Wrapper发布的名称一致
        # 格式: /airsim_node/<vehicle_name>/<camera_name>/<image_type>
        self.subscription = self.create_subscription(
            Image,
            '/airsim_node/PX4/CameraImage/Scene', # <-- 确认这个话题名称!
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Image saver node started. Subscribing to /airsim_node/PX4/CameraImage/Scene')
        self.get_logger().info(f'Images will be saved to: {self.save_path}')

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # 构建文件名并保存
        file_name = f'image_{self.image_count:05d}.png'
        full_path = os.path.join(self.save_path, file_name)

        try:
            cv2.imwrite(full_path, cv_image)
            self.get_logger().info(f'Saved {file_name}')
            self.image_count += 1
        except Exception as e:
            self.get_logger().error(f'Failed to save image: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaverNode()
    rclpy.spin(image_saver_node)
    image_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()