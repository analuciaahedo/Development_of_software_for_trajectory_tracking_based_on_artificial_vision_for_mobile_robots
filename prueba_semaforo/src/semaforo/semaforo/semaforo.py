import rclpy
from rclpy.node import Node
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import cv2

class MyLine(Node):
    def __init__(self):
        super().__init__('line_detection')
        self.image = np.ndarray((720, 1280, 3))
        self.valid_img = False
        self.bridge = CvBridge()
        self.twist_msg = Twist()
        self.sub_camera = self.create_subscription(Image, 'video_source/raw', self.video_callback, 10)
        self.pub_cropped = self.create_publisher(Image, 'cropped_image', 10)
        self.pub_line = self.create_publisher(Image, 'line', 10)
        self.velPublisher = self.create_publisher(Twist, 'cmd_vel', rclpy.qos.qos_profile_sensor_data)
        self.pub_canny = self.create_publisher(Image, 'canny', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publisher_callback)
        self.get_logger().info('Line Detection Mode started!')
        
        # Subscribe to color alerts
        self.sub_red_alert = self.create_subscription(Float32, '/img_properties/red/alert', self.red_alert_callback, 10)
        self.sub_green_alert = self.create_subscription(Float32, '/img_properties/green/alert', self.green_alert_callback, 10)
        self.red_alert = False
        self.green_alert = False
        self.alert_timeout = 0  # Time since last alert

    def video_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.valid_img = True
        except Exception as e:
            self.get_logger().error(f'Failed to get an image: {str(e)}')

    def red_alert_callback(self, msg):
        if msg.data == 1.0:
            self.red_alert = True
            self.alert_timeout = self.get_clock().now().seconds_nanoseconds()[0]
        elif msg.data == 0.0:
            self.red_alert = False

    def green_alert_callback(self, msg):
        if msg.data == 1.0:
            self.green_alert = True
            self.alert_timeout = self.get_clock().now().seconds_nanoseconds()[0]
        elif msg.data == 0.0:
            self.green_alert = False

    def publisher_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.alert_timeout > 5:  # Reset alert state after 5 seconds
            self.red_alert = False
            self.green_alert = False

        if self.valid_img:
            try:
                self.get_logger().info("Processing line detector")

                if self.red_alert:
                    self.twist_msg.linear.x = 0.0
                    self.twist_msg.angular.z = 0.0
                    self.velPublisher.publish(self.twist_msg)
                    self.get_logger().info('Red light,STOP.')
                    return
                elif self.green_alert:
                    self.get_logger().info('Green light, GO!.')
                    
                    cropped_image = self.image[400:, 150:1130]
                    self.pub_cropped.publish(self.bridge.cv2_to_imgmsg(cropped_image,"bgr8"))
                    gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
                    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                    gamma = 1.5
                    correcion = np.uint8(cv2.pow(blurred /200.0 , gamma)*200.0)
                    edges = cv2.Canny(correcion, 5, 25)
                    self.pub_canny.publish(self.bridge.cv2_to_imgmsg(edges, "mono8"))
     
                    color_edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # Convert edges image to color

                    lines_p = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=5, maxLineGap=100)
                    if lines_p is not None:
                        num_lines_p = len(lines_p)
                        self.get_logger().info(f"lines detected: {num_lines_p}")
                        contador = 0.0
                        max_lenght = 0.0
                      
                        if lines_p is not None:
                            num_lines_p = len(lines_p)
                            for line in lines_p:
                                x1,y1,x2,y2 = line [0]
                                length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                                cv2.line(color_edges, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)
                                self.pub_line.publish(self.bridge.cv2_to_imgmsg(color_edges, "bgr8"))
                                if length > max_lenght:
                                        max_lenght = length
                                        longest_line = line
                              
                            x1,y1,x2,y2 = longest_line[0]
                           

                        referencia = 490.0
                        a= np.absolute(referencia-x2)
                        b= np.absolute(referencia-x1)

                        if (a < b ):
                            error = referencia-x2
                            if error <= 30:
                                self.twist_msg.angular.z=0.0
                            else: 
                                angular_speed = error *0.00020
                                self.twist_msg.angular.z = angular_speed
                        else: 
                            error = referencia -x1
                            if -30 <= error:
                                self.twist_msg.angular.z =0.0
                            else: 
                                angular_speed = error *0.00020
                                self.twist_msg.angular.z = angular_speed
                        self.twist_msg.linear.x = 0.03

                        self.velPublisher.publish(self.twist_msg)

                if not self.red_alert and not self.green_alert:
                    cropped_image = self.image[400:, 150:1130]
                    self.pub_cropped.publish(self.bridge.cv2_to_imgmsg(cropped_image,"bgr8"))
                    gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
                    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                    gamma = 1.5
                    correcion = np.uint8(cv2.pow(blurred /200.0 , gamma)*200.0)
                    edges = cv2.Canny(correcion, 5, 25)
                    self.pub_canny.publish(self.bridge.cv2_to_imgmsg(edges, "mono8"))
     
                    color_edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # Convert edges image to color

                    lines_p = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=5, maxLineGap=100)
                    if lines_p is not None:
                        num_lines_p = len(lines_p)
                        self.get_logger().info(f"lines detected: {num_lines_p}")
                        contador = 0.0
                        max_lenght = 0.0
                      
                        if lines_p is not None:
                            num_lines_p = len(lines_p)
                            for line in lines_p:
                                x1,y1,x2,y2 = line [0]
                                length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                                cv2.line(color_edges, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)
                                self.pub_line.publish(self.bridge.cv2_to_imgmsg(color_edges, "bgr8"))
                                if length > max_lenght:
                                        max_lenght = length
                                        longest_line = line
                              
                            x1,y1,x2,y2 = longest_line[0]
                           

                        referencia = 490.0
                        a= np.absolute(referencia-x2)
                        b= np.absolute(referencia-x1)

                        if (a < b ):
                            error = referencia-x2
                            if error <= 30:
                                self.twist_msg.angular.z=0.0
                            else: 
                                angular_speed = error *0.00020
                                self.twist_msg.angular.z = angular_speed
                        else: 
                            error = referencia -x1
                            if -30 <= error:
                                self.twist_msg.angular.z =0.0
                            else: 
                                angular_speed = error *0.00020
                                self.twist_msg.angular.z = angular_speed
                        self.twist_msg.linear.x = 0.03

                        self.velPublisher.publish(self.twist_msg)
            
            except Exception as e:
                self.get_logger().error(f"Failed to process image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MyLine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
