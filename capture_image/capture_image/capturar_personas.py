import rclpy
import cv2
import numpy as np
import os
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
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            


        

        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        people_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_fullbody.xml')
        people_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')


        personas = people_cascade.detectMultiScale(img_gray, 1.1, 2)

        npersonas = 0
        for (x,y,w,h) in personas:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = cv_image[y:y+h, x:x+w]
            npersonas=npersonas+1


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