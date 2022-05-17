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
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
                    
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

        lower_blue = np.array([36, 25, 25])
        upper_blue = np.array([86, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        res = cv2.bitwise_and(cv_image, cv_image, mask= mask)
        min_x = 1000
        max_x = 0
        min_y = 1000
        max_y = 0

        for i in range(res.shape[0]):
            for j in range(res.shape[1]):
                pixel = res[i,j,:]
                if pixel[0] > 0:
                    if pixel[1] > 0:
                        if pixel[2] > 0:
                            if i > max_x:
                                max_x = i
                            if i < min_x:
                                min_x = i
                            if j > max_y:
                                max_y = j
                            if j < min_y:
                                min_y = j   

        color = (0,0,255)
        x1 = min_y
        y1 = max_x
        x2 = max_y
        y2 = min_x

        img_res = cv2.rectangle(img, (x1,y1), (x2,y2), color, 2)

        cv2.imshow('Verde detectado',img_res)
        cv2.waitKey(0) #aprieta una tecla 
        cv2.destroyAllWindows()


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