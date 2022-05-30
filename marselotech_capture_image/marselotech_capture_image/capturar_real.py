import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Ros2OpenCVImageConverter(Node):   

    """
  Clase para hacer una captura de una imagen

  Attributes:
    bridge_object (CvBridge): tranforma la imagen a csv
    image_sub (subscriber): suscriptor del topic /image
  
  Methods:
    camera_callback(): metodo para detectar la imagen y hacer la captura


  """ 

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/image',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
    def camera_callback(self,data):

        """ Función que se activa al detectar una imagen por el topic /camre/image_raw
        Permite mostrar por pantalla la imagen resultante con un rectángulo

        Args: 
        data(imagen)

        """

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            print("haciendo foto")
        except CvBridgeError as e:
            print(e)
            

        cv2.imshow("Imagen capturada por el robot", cv_image)
        cv2.imwrite("/home/belen/Imagen_capturada.jpg",cv_image)
                
        cv2.waitKey(1)    

def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = Ros2OpenCVImageConverter()    
       
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")


    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()

