import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import boto3


class Ros2OpenCVImageConverter(Node):   

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/image',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.bucket="marselotech"
        self.collectionId="caras"
        self.threshold = 1
        self.maxFaces=3
        self.enemy=0
        self.known=False
        self.friendlist = ['Belen', 'Leire', 'Jose']
        self.client=boto3.client('rekognition', 'us-east-1')

    def camera_callback(self,data):

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            cv2.imwrite("/home/belen/image.jpg", cv_image)
            print("rostro detectado")
        except CvBridgeError as e:
            print(e)
            

        

        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        face_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_frontalface_default.xml')
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


        caras = face_cascade.detectMultiScale(img_gray, 1.1, 5)

        ncaras = 0
        for (x,y,w,h) in caras:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = cv_image[y:y+h, x:x+w]
            ncaras=ncaras+1


        if(ncaras>0):
            cv2.imshow("Imagen capturada por el robot", cv_image) #muestra la imagen
            
            cv2.imshow("Imagen capturada por el robot", cv_image)
            cv2.imwrite("Imagen_capturada.jpg",cv_image) #la guarda

            response=self.client.search_faces_by_image(CollectionId=self.collectionId,
                            SourceImage={'Bytes': cv_image},
                            FaceMatchThreshold=self.threshold,
                            MaxFaces=self.maxFaces)

            faceMatches=response['FaceMatches']
            print ('Matching faces')
            for match in faceMatches:
                if(match['Similarity']>90):
                    known=True
                    if(match['Face']['ExternalImageId'] in self.friendlist):
                        print ('Se ha detectado a :'  + match['Face']['ExternalImageId'])
                        print ('Similarity: ' + "{:.2f}".format(match['Similarity']) + "%")
                        
                    else:
                        print ('Se ha detectado a :'  + match['Face']['ExternalImageId'])
                        print ('Similarity: ' + "{:.2f}".format(match['Similarity']) + "%")
                        enemy = 1

            if(known):
                if(enemy):
                    print("Enemigo detectado")
                else:
                    print("Aliado detectado")
            else:
                print("Persona desconocida detectada")
                
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
