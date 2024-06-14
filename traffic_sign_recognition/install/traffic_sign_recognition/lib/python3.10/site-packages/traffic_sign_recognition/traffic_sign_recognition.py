import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32
import cv2
import numpy as np
import os
from collections import deque

class TrafficSignRecognitionNode(Node):
    def __init__(self):
        super().__init__('traffic_sign_recognition_node')
        
        # Ruta a la carpeta que contiene las plantillas de las señales de tránsito
        template_folder = '/home/redes/traffic_sign_recognition/src/traffic_sign_recognition'  # Actualiza esta ruta según corresponda
        
        # Nombres de las imágenes de las plantillas
        template_names = [
            ('giveaway.jpg', 0),
            ('workinprogress.jpg', 1),
            ('turnright.jpg', 2),
            ('turnleft.jpg', 3),
            ('straigth.jpg', 4),
            ('stop.jpg', 5)
        ]

        # Cargar las plantillas de señales de tránsito
        self.templates = {}
        self.template_values = {}
        for name, value in template_names:
            path = os.path.join(template_folder, name)
            template = cv2.imread(path, 0)
            self.templates[name.split('.')[0]] = template
            self.template_values[name.split('.')[0]] = value
        
        # Inicializar SIFT
        self.sift = cv2.SIFT_create()
        
        # Detectar características clave y descriptores en las plantillas
        self.template_keypoints_descriptors = {}
        for name, template in self.templates.items():
            keypoints, descriptors = self.sift.detectAndCompute(template, None)
            self.template_keypoints_descriptors[name] = (keypoints, descriptors)

        # Inicializar el matcher de características
        self.bf = cv2.BFMatcher(cv2.NORM_L2)
        
        # Configurar el suscriptor de imágenes
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camara_callback, 10)
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Int32, 'traffic_sign', 10)

        # Configurar la media móvil
        self.detections = deque(maxlen=5)  # Puedes ajustar el tamaño de la ventana de la media móvil

    def camara_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        match = self.match_sift(frame, self.template_keypoints_descriptors)
        
        if match is not None:
            value = self.template_values[match]
            self.detections.append(value)
        else:
            self.detections.append(-1)  # Añadir un valor que indique que no se detectó ninguna señal
        
        # Calcular la media móvil
        if len(self.detections) == self.detections.maxlen:
            values = [d for d in self.detections if d != -1]
            if values:
                most_common_value = max(set(values), key=values.count)
                self.publisher_.publish(Int32(data=most_common_value))
                self.get_logger().info(f'Señal detectada: {most_common_value}')
            else:
                self.get_logger().info('No se detectó ninguna señal en la ventana de la media móvil')
        else:
            self.get_logger().info('Acumulando detecciones para la media móvil')

    def match_sift(self, frame, template_keypoints_descriptors):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints_frame, descriptors_frame = self.sift.detectAndCompute(gray_frame, None)
        
        best_match = None
        best_matches_count = 0
        min_good_matches = 15
        
        for name, (keypoints_template, descriptors_template) in template_keypoints_descriptors.items():
            if descriptors_template is not None and descriptors_frame is not None:
                matches = self.bf.knnMatch(descriptors_template, descriptors_frame, k=2)
                
                good_matches = []
                for match in matches:
                    if len(match) == 2:  # Ensure there are two matches to unpack
                        m, n = match
                        if m.distance < 0.75 * n.distance:
                            good_matches.append(m)
                
                if len(good_matches) > min_good_matches:
                    src_pts = np.float32([keypoints_template[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                    dst_pts = np.float32([keypoints_frame[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                    matches_mask = mask.ravel().tolist()
                    
                    if sum(matches_mask) > best_matches_count:
                        best_matches_count = sum(matches_mask)
                        best_match = name
        
        if best_matches_count >= min_good_matches:
            return best_match
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

