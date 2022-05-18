import rclpy
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Ros2OpenCVImageConverter(Node):   

    """
    Detección de personas en simulación

    Attributes:
        bridge_subject (CvBridge): Transforma la imagen a csv
        image_sub (subscriber): Suscriptor del topic

    
    Methods:
        camera_callback(): Recibe la imagen y detecta la persona
    """ 

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
    def camera_callback(self,data):

        """ Función que se activa al detectar una imagen por el topic /camera/image_raw
            Permite motrar por pantalla y guardar una imagen cada vez que se detecta una persona

        Args: 
            data (Imagen): La imagen recibida mediante el topic

        """


        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            


        

        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Cargamos las librerías
        people_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_fullbody.xml')
        people_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

        # Detecta las personas de la imagen en escala de grises
        personas = people_cascade.detectMultiScale(img_gray, 1.1, 2)

        # Por cada persona detectada se dibuja un rectángulo a su al rededor
        npersonas = 0
        for (x,y,w,h) in personas:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = cv_image[y:y+h, x:x+w]
            npersonas=npersonas+1


        # Muestra la imagen y la guarda si detecta una persona, además de avisar por terminal
        if(npersonas>0):
            cv2.imshow("PRUEBA", cv_image)
            
            print("Intruso detectado!!!")

            cv2.imwrite('/home/belen/imagen.jpg', cv_image)

        else:
            print(npersonas)
        
        cv2.waitKey(1)



            

def main(args=None):

    os.system('python --version')
    os.system('pwd')


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