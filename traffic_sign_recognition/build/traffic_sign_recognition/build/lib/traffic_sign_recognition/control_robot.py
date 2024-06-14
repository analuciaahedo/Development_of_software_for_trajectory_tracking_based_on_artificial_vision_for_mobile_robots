import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.subscription = self.create_subscription(Int32, 'traffic_sign', self.traffic_sign_callback, 10)
        self.pubcmdvel = self.create_publisher(Twist, 'cmd_vel', rclpy.qos.qos_profile_sensor_data)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('robot_control_node started')
        self.stop_robot_flag = False
        self.lineal_vel = 0.050
        self.ang_vel = 0.0

    def timer_callback(self):
        twist_msg = Twist()
        if self.stop_robot_flag:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
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

    '''
    def stop_robot(self):
        self.get_logger().info('Deteniendo el robot completamente')
        self.stop_robot_flag = True

    def turn_left(self):
        self.get_logger().info('Girando a la izquierda')
        self.stop_robot_flag = True
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.5
        self.pubcmdvel.publish(twist_msg)

    def turn_right(self):
        self.get_logger().info('Girando a la derecha')
        self.stop_robot_flag = False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = -0.5
        self.pubcmdvel.publish(twist_msg)
    '''

    def go_straight(self):
        self.get_logger().info('Avanzando recto')
        self.stop_robot_flag = False
        self.lineal_vel = 0.1

    '''
    def give_way(self):
        self.get_logger().info('Cediendo el paso')
        self.stop_robot_flag = True
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pubcmdvel.publish(twist_msg)

    def work_in_progress(self):
        self.get_logger().info('Trabajos en progreso, reduciendo velocidad')
        self.stop_robot_flag = False
        self.lineal_vel = 0.025
    '''

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
