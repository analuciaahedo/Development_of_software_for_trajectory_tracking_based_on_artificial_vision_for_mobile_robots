import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

class CVExample(Node):
    def __init__(self):
        super().__init__('cv_node')
        self.img = None  # Inicializa la imagen como None
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        
        # Publicadores para el color rojo
        self.pub_red = self.create_publisher(Image, 'img_properties/red/msk', 10)
        self.pubper_red = self.create_publisher(Float32, '/img_properties/red/percentage', 10)
        self.pub_red_alert = self.create_publisher(Float32, '/img_properties/red/alert', 10)

        # Publicadores para el color verde
        self.pub_green = self.create_publisher(Image, 'img_properties/green/msk', 10)
        self.pubper_green = self.create_publisher(Float32, '/img_properties/green/percentage', 10)
        self.pub_green_alert = self.create_publisher(Float32, '/img_properties/green/alert', 10)

        self.timer = self.create_timer(0.1, self.process_image)
        self.get_logger().info('CV Node started')

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().info(f'Failed to get an image: {e}')
            self.img = None  # Asegura que img se establece como None si la conversión falla

    def process_image(self):
        if self.img is not None and self.img.size != 0:  # Verifica que img no esté vacía
            try:
                hsv_image = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
                width = self.img.shape[1]

                # Procesamiento de color rojo
                red_mask = self.create_red_mask(hsv_image)
                self.process_red_mask(red_mask[:, width // 2:], self.img[:, width // 2:])

                # Procesamiento de color verde
                green_mask = self.create_green_mask(hsv_image)
                self.process_green_mask(green_mask[:, width // 2:], self.img[:, width // 2:])

            except Exception as e:
                self.get_logger().info(f'Error processing image: {e}')

    def create_red_mask(self, hsv_image):
        lower_red1 = np.array([0, 100, 100], np.uint8)
        upper_red1 = np.array([10, 255, 255], np.uint8)
        lower_red2 = np.array([160, 100, 100], np.uint8)
        upper_red2 = np.array([180, 255, 255], np.uint8)
        red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        return cv2.bitwise_or(red_mask1, red_mask2)

    def create_green_mask(self, hsv_image):
        lower_green = np.array([45, 40, 40], np.uint8)
        upper_green = np.array([75, 255, 255], np.uint8)
        return cv2.inRange(hsv_image, lower_green, upper_green)

    def process_red_mask(self, red_mask, img_segment):
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.dilate(red_mask, kernel, iterations=1)
        red_mask = cv2.erode(red_mask, kernel, iterations=1)
        highlighted = cv2.bitwise_and(img_segment, img_segment, mask=red_mask)
        self.pub_red.publish(self.bridge.cv2_to_imgmsg(highlighted, "bgr8"))
        self.calculate_and_publish_red_percentage(red_mask)

    def process_green_mask(self, green_mask, img_segment):
        kernel = np.ones((5, 5), np.uint8)
        green_mask = cv2.dilate(green_mask, kernel, iterations=1)
        green_mask = cv2.erode(green_mask, kernel, iterations=1)
        highlighted = cv2.bitwise_and(img_segment, img_segment, mask=green_mask)
        self.pub_green.publish(self.bridge.cv2_to_imgmsg(highlighted, "bgr8"))
        self.calculate_and_publish_green_percentage(green_mask)

    def calculate_and_publish_red_percentage(self, red_mask):
        red_pixel_count = np.sum(red_mask > 0)
        total_pixels = red_mask.shape[0] * red_mask.shape[1]
        red_percentage = red_pixel_count / total_pixels
        self.pubper_red.publish(Float32(data=red_percentage))
        red_alert_value = 1.0 if red_percentage >= 0.001 else 0.0
        self.pub_red_alert.publish(Float32(data=red_alert_value))
        self.get_logger().info(f'Red Alert: {red_alert_value}')

    def calculate_and_publish_green_percentage(self, green_mask):
        green_pixel_count = np.sum(green_mask > 0)
        total_pixels = green_mask.shape[0] * green_mask.shape[1]
        green_percentage = green_pixel_count / total_pixels
        self.pubper_green.publish(Float32(data=green_percentage))
        green_alert_value = 1.0 if green_percentage >= 0.001 else 0.0
        self.pub_green_alert.publish(Float32(data=green_alert_value))
        self.get_logger().info(f'Green Alert: {green_alert_value}')

def main(args=None):
    rclpy.init(args=args)
    cv_example = CVExample()
    rclpy.spin(cv_example)
    cv_example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


