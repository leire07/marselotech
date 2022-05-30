import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Ros2OpenCVImageConverter(Node):   

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
    def camera_callback(self,data):
        """ Esta función devuelve un callback, muestra en una ventana una imagen cuando
            detecta una cara (en simulación)

        Args: 
            data : Imagen que recibe del robot en forma de matriz (píxeles)

        """

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            cv2.imwrite("image", cv_image)
        except CvBridgeError as e:
            print(e)
            

        

        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) #ponemos la imagen en blanco y negro

        face_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_frontalface_default.xml') #añadimos los clasificadores
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


        caras = face_cascade.detectMultiScale(img_gray, 1.1, 5) #lo aplicamos a la imagen en blanco y negro

        ncaras = 0 #variable para contar el número de caras
        for (x,y,w,h) in caras:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2) #añadimos un rectángulo a cada cara
            roi_color = cv_image[y:y+h, x:x+w]
            ncaras=ncaras+1 #suma 1 cada vez que encuentra una cara


        if(ncaras>0): #si detecta al menos una cara
            cv2.imshow("Imagen capturada por el robot", cv_image) #muestra la imagen
            
            cv2.imshow("Imagen capturada por el robot", cv_image)
            cv2.imwrite("Imagen_capturada.jpg",cv_image) #la guarda
                
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
