import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.subscription = self.create_subscription(Int32, 'traffic_sign', self.traffic_sign_callback, 10)
        self.image_subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 1)
        self.pubcmdvel = self.create_publisher(Twist, 'cmd_vel', rclpy.qos.qos_profile_sensor_data)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('robot_control_node started')
        self.bridge = CvBridge()
        self.twist_msg = Twist()
        self.stop_robot_flag = False
        
        self.ang_vel = 0.0
        self.lineal_vel = 0.03
        self.lineal_vel_men = 0.01
        self.ang_vel_right = 0.05
        self.ang_vel_left = -0.05
        self.lineal_stop = 0.0

    def timer_callback(self):
        twist_msg = Twist()
        if self.stop_robot_flag:
            twist_msg.linear.x = self.lineal_stop
            twist_msg.angular.z = self.ang_vel
        else:
            twist_msg.linear.x = self.lineal_vel
            twist_msg.angular.z = self.ang_vel
        self.pubcmdvel.publish(twist_msg)

    def traffic_sign_callback(self, msg):
        sign = msg.data
        if sign == 5:  # stop sign
            self.stop_robot()
        elif sign == 3:  # turn left sign
            self.turn_left()
        elif sign == 2:  # turn right sign
            self.turn_right()
        elif sign == 4:  # go straight sign
            self.go_straight()
        elif sign == 0:  # giveaway sign
            self.give_way()
        elif sign == 1:  # work in progress sign
            self.work_in_progress()

    def stop_robot(self):
        self.get_logger().info('Deteniendo el robot completamente')
        self.stop_robot_flag = True

    def turn_left(self):
        self.get_logger().info(f'Girando a la izquierda con una velocidad de {self.ang_vel_left}')
        self.stop_robot_flag = True
        #self.twist_msg.linear.x = self.lineal_stop
        self.twist_msg.angular.z = self.ang_vel_left
        self.pubcmdvel.publish(self.twist_msg)

    def turn_right(self):
        self.get_logger().info(f'Girando a la derecha con una velocidad de {self.ang_vel_right}')
        self.stop_robot_flag = False
        #self.twist_msg.linear.x = self.lineal_stop
        self.twist_msg.angular.z = self.ang_vel_right
        self.pubcmdvel.publish(self.twist_msg)

    def go_straight(self):
        self.get_logger().info(f'Avanzando recto, velocidad constante de {self.lineal_vel}')
        self.stop_robot_flag = False
        self.twist_msg.angular.z = self.ang_vel
        self.twist_msg.linear.x = self.lineal_vel
        self.pubcmdvel.publish(self.twist_msg)

    def give_way(self):
        self.get_logger().info(f'Cediendo el paso, reduciendo velocidad a {self.lineal_vel_men}')
        self.stop_robot_flag = False
        self.twist_msg.linear.x = self.lineal_vel_men
        self.pubcmdvel.publish(self.twist_msg)

    def work_in_progress(self):
        self.get_logger().info(f'Trabajos en progreso, reduciendo velocidad a {self.lineal_vel_men}')
        self.stop_robot_flag = False
        self.twist_msg.linear.x = self.lineal_vel_men
        self.pubcmdvel.publish(self.twist_msg)
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

