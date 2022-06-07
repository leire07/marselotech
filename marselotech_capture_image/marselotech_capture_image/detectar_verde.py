import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
kernel = np.ones((7,7),np.uint8)

class Ros2OpenCVImageConverter(Node):   

    """
  Clase para detectar el color verde en una persona

  Attributes:
    bridge_object (CvBridge): tranforma la imagen a csv
    image_sub (subscriber): suscriptor del topic /camera/image_raw
  
  Methods:
    camera_callback(): metodo para detectar la imagen y que detecte el color verde por la persona


  """

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
    def camera_callback(self,data):

        """ Función que se activa al detectar una imagen por el topic /camre/image_raw
        Permite mostrar por pantalla la imagen resultante con un rectángulo

        Args: 
        data(imagen)

        """
        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
                    
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        #Detectar persona

        # Cargamos las librerías
        people_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_fullbody.xml')
        people_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

        # Detecta las personas de la imagen en escala de grises
        personas = people_cascade.detectMultiScale(img_gray, 1.1, 2)

        # Por cada persona detectada se dibuja un rectángulo a su al rededor
        npersonas = 0
        for (x,y,w,h) in personas:
            #cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = cv_image[y:y+h, x:x+w]
            npersonas=npersonas+1

        #Termina detectar persona


        lower_green = np.array([36, 25, 25])
        upper_green = np.array([86, 255, 255])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        	
        apertura = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        cv2.imshow('funcionapordios', apertura)

        mask = apertura
        
        res = cv2.bitwise_and(cv_image, cv_image, mask= mask)

        x_inicial = 1000
        x_final = 0
        y_inicial = 1000
        y_final = 0

        for i in range(res.shape[0]):
            for j in range(res.shape[1]):
              pixeles = res[i,j,:]
        
              if pixeles[0] > 0: 
                 if pixeles[1] > 0:
                       if pixeles[2] > 0:
                           if i > x_final:
                                x_final = i
                           if i < x_inicial:
                                x_inicial = i
                           if j > y_final:
                                y_final = j
                           if j < y_inicial:
                                y_inicial = j

        x1 = y_inicial
        y1 = x_final
        x2 = y_final
        y2 = x_inicial

        color = (0,0,255)

        if(npersonas>0):
            rectangulo = cv2.rectangle(cv_image, (x1,y1), (x2,y2), color, 2)
            cv2.imshow('Verde detectado',rectangulo)
            cv2.waitKey(1) #aprieta una tecla 
            print("Se ha detectado una persona de mi equipo")
        else:
            print("No hay personas verdes")

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