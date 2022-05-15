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

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.cv_imagemsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

# Cargamos los clasificadores
        face_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_frontalface_default.xml')
        eye_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_eye.xml')

# Detectamos caras
        faces = face_cascade.detectMultiScale(cv_image_gray, 1.1, 5)

# Para cada cara detectada, dibujamos un rectángulo
        for (x,y,w,h) in faces:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = cv_image[y:y+h, x:x+w]

        cv2.imshow("Imagen capturada por el robot", cv_image)
                
        cv2.waitKey(1)    

def main(args=None):

    rclpy.init(args=args)    
    cv_image_converter_object = Ros2OpenCVImageConverter()    
       
    try:
        rclpy.spin(cv_image_converter_object)
    except KeyboardInterrupt:
        cv_image_converter_object.destroy_node()
        print("Fin del programa!")
    
    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()
