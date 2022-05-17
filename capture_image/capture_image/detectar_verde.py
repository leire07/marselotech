import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
kernel = np.ones((7,7),np.uint8)

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
                    
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)


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

        rectangulo = cv2.rectangle(cv_image, (x1,y1), (x2,y2), color, 2)
     
        

        cv2.imshow('Verde detectado',rectangulo)
        

        cv2.waitKey(1) #aprieta una tecla 

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