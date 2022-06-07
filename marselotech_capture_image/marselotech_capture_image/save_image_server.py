# Importar mensajes
from distutils.command.upload import upload
from geometry_msgs.msg import Twist
from marselotech_custom_interface.srv import DetectionMsg
from cv2 import imread, imshow, imwrite
import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import boto3
import pyrebase
import firebase_admin
from firebase_admin import credentials, storage, firestore
import datetime



class Service(Node):
    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('save_server') 
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio

        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/image',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))      



    def camera_callback(self,data):
        """ Esta función devuelve un callback, muestra en una ventana una imagen cuando
            detecta una cara (en simulación)

        Args: 
            data : Imagen que recibe del robot en forma de matriz (píxeles)

        """

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            cv2.imwrite("/home/john/image.jpg", cv_image)
            self.funciona=True
        except CvBridgeError as e:
            print(e)

    

def main(args=None):
    # inicializa la comunicacion ROS2
    rclpy.init(args=args)
    # creamos el nodo
    service = Service()
    try:
        #dejamos abierto el servicio
        rclpy.spin(service)
    except KeyboardInterrupt:
        service.get_logger().info('Cerrando el nodo service')
    finally:
        #destruimos el nodo
        service.destroy_node()
        #cerramos la comunicacion
        rclpy.shutdown()

#definimos el ejecutable
if __name__=='__main__':
    main()