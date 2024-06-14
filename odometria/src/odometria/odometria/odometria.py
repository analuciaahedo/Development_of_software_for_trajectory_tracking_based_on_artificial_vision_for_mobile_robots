import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class My_Process(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.last_velL = 0.0
        self.last_velR = 0.0

        # interrupcion por suscripcion a los topicos de velocidad angular del motor izquierdo y del motor derecho
        self.velL_s = self.create_subscription(Float32, 'VelocityEncL', self.process_callback_velL, rclpy.qos.qos_profile_sensor_data)
        self.velR_s = self.create_subscription(Float32, 'VelocityEncR', self.process_callback_velR, rclpy.qos.qos_profile_sensor_data)

        # publishers para distancia, velocidad, orientacion, velocidad angular, posicion global (x,y)
        self.distance_p = self.create_publisher(Float32, 'distance', rclpy.qos.qos_profile_sensor_data)
        self.linear_p =  self.create_publisher(Float32, 'linear_speed', rclpy.qos.qos_profile_sensor_data)
        self.orientation_p = self.create_publisher(Float32, 'odometria_a', rclpy.qos.qos_profile_sensor_data)
        self.angular_p = self.create_publisher(Float32, 'angular_speed', rclpy.qos.qos_profile_sensor_data)
        self.x_p = self.create_publisher(Float32, 'positionX', rclpy.qos.qos_profile_sensor_data)
        self.y_p = self.create_publisher(Float32, 'positionY', rclpy.qos.qos_profile_sensor_data)

        # periodo de muestreo
        self.timer_period = 0.1

        # callback ejecutado por timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Process node successfully initialized!')

        # mensajes a calcular
        self.msg_x = Float32()  # Posición en el eje x
        self.msg_y = Float32()  # Posición en el eje y
        self.msg_theta = Float32()  # Orientación del robot
        self.msg_distance = Float32() # Distancia del robot
        self.msg_speed = Float32()
        self.msg_angular = Float32()

        self.radius = 0.05
        self.axis = 0.18

        self.old_distance = 0.0
        self.old_theta = 0.0
        self.old_x = 0.0
        self.old_y = 0.0

    def process_callback_velL(self, msg):
        self.last_velL = msg.data

    def process_callback_velR(self, msg):
        self.last_velR = msg.data

    def timer_callback(self):
        speed = ((self.last_velL + self.last_velR) * self.radius) / 2.0
        self.msg_speed.data = speed

        angular_speed = (self.radius * (self.last_velR - self.last_velL)) / self.axis
        self.msg_angular.data = angular_speed

        speed_for_dist = abs(speed)

        distance = self.old_distance + (speed_for_dist * self.timer_period)

        self.msg_distance.data = distance

        theta = (self.old_theta + (angular_speed * self.timer_period)) % (2 * math.pi)

        self.msg_theta.data = theta

        speed_for_x = speed * np.cos(theta)

        speed_for_y = speed * np.sin(theta)

        x = self.old_x + (speed_for_x * self.timer_period)
        self.msg_x.data = x

        y = self.old_y + (speed_for_y * self.timer_period)
        self.msg_y.data = y

        self.old_distance = distance
        self.old_theta = theta
        self.old_x = x
        self.old_y = y

        self.distance_p.publish(self.msg_distance)
        self.linear_p.publish(self.msg_speed)
        self.orientation_p.publish(self.msg_theta)
        self.angular_p.publish(self.msg_angular)
        self.x_p.publish(self.msg_x)
        self.y_p.publish(self.msg_y)

def main(args=None):
    rclpy.init(args=args)
    m_p = My_Process()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

