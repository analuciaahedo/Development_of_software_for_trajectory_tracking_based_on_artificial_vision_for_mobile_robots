import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from rclpy import qos
from cv_bridge import CvBridge
import cv2

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # Subscriptions
        self.subscription = self.create_subscription(Int32, 'traffic_sign', self.traffic_sign_callback, 10)
        self.image_subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 1)
        
        # Publisher
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', qos.qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.turn_timer = self.create_timer(0.1, self.turn_timer_callback)
        
        self.bridge = CvBridge()
        self.twist_msg = Twist()
        
        self.stop_robot_flag = False
        self.turning = False
        self.turn_left = False
        self.turn_right = False
        
        self.linear_vel = 0.02
        self.linear_vel_slow = 0.01
        self.angular_vel_right = -0.3
        self.angular_vel_left = 0.3
        self.linear_stop = 0.0
        
        self.get_logger().info('robot_control_node started')

    def timer_callback(self):
        if self.stop_robot_flag:
            self.twist_msg.linear.x = self.linear_stop
            self.twist_msg.angular.z = 0.0
        else:
            self.twist_msg.linear.x = self.linear_vel
            self.twist_msg.angular.z = 0.0
        
        self.pub_cmd_vel.publish(self.twist_msg)

    def turn_timer_callback(self):
        if self.turning:
            if self.turn_left:
                self.twist_msg.angular.z = self.angular_vel_left
                self.twist_msg.linear.x = self.linear_stop
            elif self.turn_right:
                self.twist_msg.angular.z = self.angular_vel_right
                self.twist_msg.linear.x = self.linear_stop
        else:
            self.twist_msg.angular.z = 0.0
            self.twist_msg.linear.x = self.linear_stop
        
        self.pub_cmd_vel.publish(self.twist_msg)

    def traffic_sign_callback(self, msg):
        sign = msg.data
        if sign == 5:  # stop sign
            self.stop_robot()
        elif sign == 3:  # turn left sign
            self.start_turn('left')
        elif sign == 2:  # turn right sign
            self.start_turn('right')
        elif sign == 4:  # go straight sign
            self.go_straight()
        elif sign == 0:  # give way sign
            self.give_way()
        elif sign == 1:  # work in progress sign
            self.work_in_progress()
        else:
            self.get_logger().warn(f'Unknown traffic sign: {sign}')

    def start_turn(self, direction):
        self.stop_robot_flag = False
        self.turning = True
        if direction == 'left':
            self.turn_left = True
            self.turn_right = False
        elif direction == 'right':
            self.turn_left = False
            self.turn_right = True
        self.get_logger().info(f'Starting to turn {direction}')

    def stop_turning(self):
        self.turning = False
        self.turn_left = False
        self.turn_right = False
        self.stop_robot_flag = True  # Ensure the robot stops after turning
        self.twist_msg.angular.z = 0.0
        self.twist_msg.linear.x = self.linear_stop  # Stop movement after turn
        self.pub_cmd_vel.publish(self.twist_msg)
        self.get_logger().info('Turn completed and robot stopped')

    def stop_robot(self):
        self.get_logger().info('Stopping the robot completely')
        self.stop_robot_flag = True
        self.turning = False
        self.turn_left = False
        self.turn_right = False
        self.twist_msg.linear.x = self.linear_stop
        self.twist_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(self.twist_msg)

    def go_straight(self):
        self.get_logger().info(f'Going straight with constant speed {self.linear_vel}')
        self.stop_robot_flag = False
        self.turning = False
        self.turn_left = False
        self.turn_right = False

    def give_way(self):
        self.get_logger().info(f'Giving way, reducing speed to {self.linear_vel_slow}')
        self.stop_robot_flag = False
        self.turning = False
        self.turn_left = False
        self.turn_right = False
        self.twist_msg.linear.x = self.linear_vel_slow
        self.twist_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(self.twist_msg)

    def work_in_progress(self):
        self.get_logger().info(f'Work in progress, reducing speed to {self.linear_vel_slow}')
        self.stop_robot_flag = False
        self.turning = False
        self.turn_left = False
        self.turn_right = False
        self.twist_msg.linear.x = self.linear_vel_slow
        self.twist_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(self.twist_msg)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Here you can add additional image processing if needed
        # If the signal is no longer detected, stop turning
        if not self.detect_signal(cv_image):
            self.stop_turning()

    def detect_signal(self, image):
        # Dummy implementation: replace with actual signal detection
        # Return False if the signal is not detected
        return False

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
