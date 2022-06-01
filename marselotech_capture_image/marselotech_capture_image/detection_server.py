# Importar mensajes
from geometry_msgs.msg import Twist
from marselotech_custom_interface.srv import DetectionMsg
from cv2 import imread
import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import boto3



class Service(Node):
    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('movement_server') 
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(DetectionMsg, 'detection', self.marselotech_my_service_callback)

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
        self.cv_image
        self.kernel = np.ones((7,7),np.uint8)



    def camera_callback(self,data):
        """ Esta función devuelve un callback, muestra en una ventana una imagen cuando
            detecta una cara (en simulación)

        Args: 
            data : Imagen que recibe del robot en forma de matriz (píxeles)

        """

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            cv2.imwrite("image", self.cv_image)
        except CvBridgeError as e:
            print(e)

    def marselotech_my_service_callback(self, request, response):
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response

        if request.type == "caras":
            
            self.capturar_caras()
            response.success = True
        elif request.type == "color":

            self.detectar_verde()
            response.success = True
        elif request.type == "personas":
            
            self.capturar_personas()
            response.success = True
        
        else:
            # estado de la respuesta
            # si no se ha dado ningun caso anterior
            response.success = False

        # devuelve la respuesta
        return response

    def capturar_caras(self):
        img_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        face_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_frontalface_default.xml')
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


        caras = face_cascade.detectMultiScale(img_gray, 1.1, 5)

        ncaras = 0
        for (x,y,w,h) in caras:
            cv2.rectangle(self.cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = self.cv_image[y:y+h, x:x+w]
            ncaras=ncaras+1


        if(ncaras>0):
            cv2.imshow("Imagen capturada por el robot", self.cv_image) #muestra la imagen
            
            cv2.imwrite("Imagen_capturada.jpg",self.cv_image) #la guarda
            imageSource=open("Imagen_capturada.jpg",'rb')

            img = imread("Imagen_capturada.jpg")
            cv2.imshow("Imagen capturada por el robot", img)


            response=self.client.search_faces_by_image(CollectionId=self.collectionId,
                            Image={'Bytes': imageSource.read()},
                            FaceMatchThreshold=self.threshold,
                            MaxFaces=self.maxFaces)

            faceMatches=response['FaceMatches']
            print ('Matching faces')
            for match in faceMatches:
                if(match['Similarity']>90):
                    self.known=True
                    if(match['Face']['ExternalImageId'] in self.friendlist):
                        print ('Se ha detectado a :'  + match['Face']['ExternalImageId'])
                        print ('Similarity: ' + "{:.2f}".format(match['Similarity']) + "%")
                        
                    else:
                        print ('Se ha detectado a :'  + match['Face']['ExternalImageId'])
                        print ('Similarity: ' + "{:.2f}".format(match['Similarity']) + "%")
                        self.enemy = 1

            if(self.known):
                if(self.enemy):
                    print("Enemigo detectado")
                else:
                    print("Aliado detectado")
            else:
                print("Intruso detectado")
        
        self.enemy=0
        self.known=False

    def capturar_personas(self):
        img_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        people_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_fullbody.xml')
        people_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

        personas = people_cascade.detectMultiScale(img_gray, 1.1, 2)

        npersonas = 0
        for (x,y,w,h) in personas:
            cv2.rectangle(self.cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = self.cv_image[y:y+h, x:x+w]
            npersonas=npersonas+1
            print("puta")


        if(npersonas>0):
            cv2.imshow("PRUEBA", self.cv_image)
            
            print("Intruso detectado!!!")

            cv2.imwrite('/home/belen/imagen.jpg', self.cv_image)

        else:
            print(npersonas)
        
        cv2.waitKey(1)

    def detectar_verde(self):

    
        hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)


        lower_green = np.array([36, 25, 25])
        upper_green = np.array([86, 255, 255])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        	
        apertura = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        cv2.imshow('funcionapordios', apertura)

        mask = apertura
        
        res = cv2.bitwise_and(self.cv_image, self.cv_image, mask= mask)

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

        rectangulo = cv2.rectangle(self.cv_image, (x1,y1), (x2,y2), color, 2)

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