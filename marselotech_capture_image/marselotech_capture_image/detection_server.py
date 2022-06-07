# Importar mensajes
from distutils.command.upload import upload
from geometry_msgs.msg import Twist
from marselotech_custom_interface.srv import DetectionMsg
from cv2 import imread, imshow, imwrite
import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import boto3
import pyrebase
import firebase_admin
from firebase_admin import credentials, storage, firestore
import datetime





class Service(Node):
    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('detection_server') 
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(DetectionMsg, 'detection', self.marselotech_my_service_callback)

        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/image',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.bucketname="marselotech"
        self.collectionId="caras"
        self.threshold = 1
        self.maxFaces=3
        self.enemy=0
        self.known=False
        self.friendlist = ['Belen', 'Leire', 'Jose']
        self.client=boto3.client('rekognition', 'us-east-1')
        self.kernel = np.ones((7,7),np.uint8)
        self.funciona=False


        #Credenciales para la comunicación FIREBASE
        self.config = {
            "apiKey": "AIzaSyB5H63Brfoe6WKs8t2v-5WXVP70c8X8hm4",
            "authDomain": "marselotech-web.firebaseapp.com",
            "databaseURL": "https://marselotech-web-default-rtdb.firebaseio.com",
            "storageBucket": "marselotech-web.appspot.com"
        }

        jsonObject={
            "type": "service_account",
            "project_id": "marselotech-web",
            "private_key_id": "7d2c358ef0507a9e3d9ddde9905d14e6495d0dae",
            "private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgkqhkiG9w0BAQEFAASCBKYwggSiAgEAAoIBAQC0et+ofrI0H+KG\nNUP+sPn5svHMAqx9yusfNleqyFDeWN2dXv7L1rrMRx7BaJQE/2PXD4czi/FPVNoT\nWoLnc8SnuzzPt7d48KhdQ0qOUzhwfs+dGbKcMQ9jeMmUqiwFnzbTC1yrTCxzHwGv\nwQ9ypHOsD60AUNiKeQf4Dxe9o2xbUVC+KmBr6k7gMa+phIzYKfcYaNQ6IKBN1Xg2\nRN6BztRMWduCmjqtMaYjsKpL/luOtRka5pImvSORWH8ERJG97Q/eI9ujwSalCIyJ\n2KzsjOo0IxGdZol5ayrwAK/uoRoDYK86zx67c/HzrYmFdEEOvI9WNsIpno0a1Hot\nWjO2Ti1TAgMBAAECggEAE7229ia5llMAg56S6+puxV3J7f9C39WQx0654x/bHJ8K\nyppn82Lu9sY2uoAWZL+Mq6rypnRBtmQ5IFHPrMJwecpUugHF61AjkmP4ZT38F+3/\nBpeXflctcDt3jS/Z9dl29Tmybrg7ynHTTOaoqmkLj//88+Jj9S2M7hi0h5U3FnvM\nnbtv91XzLsEVSaHI3E9rJBFetUQViHv/wB4DoIT7pfW9v/HyYuJAK5DM9DYYT3+1\n2El5jnov8e3v4k33InGUayA6xutSsuNsxAhuU/7UL+FcJZvPWeG8WF7nCNOfv3xw\nEsyiNEjWem+KHwdMHrwdcVjqbtB50EmONXrlbqiz0QKBgQDziT4pnvb6HtTyit5M\n4/5kTK1xTaAIvynTwAmbv/MEPyxjzGSNztuFN1hhx0/t8sF33cLVVZadEvBhUaKn\nCi/pRXqg8BsxZ8e6W8SC8dk1rszZYET9rkkl20TGKh+8UIbqNPQP0lz1LFuZ5R+u\n62Mpu0C6s9FLPsiUfRkxnTh40QKBgQC9t3ut2pz/RZfyiXFliX02Wzi5l20ndJ4Z\nae/rAx42eZWvXKfDOuoUpRnSmPbC+ZHEakACUFjhu7XQTTv2OO+rhrFW33+wRbHT\nUwP90wVDrpUNuZslazU5hRw2oQmxcFlCM7zq2DJVUPPjEjfab3p1rU3aL+9W8B8F\nYaYFqs1M4wKBgEvt4Uy2vEgVbs1EELUmbH03DuiBjEDN4Suc9yHxQcJ0M9HVuxAf\ni3/IVqz9qGUGx90obgN3kOLeMcYV3sF3wqJXQDmHQuMveS0YSXeOEevT2Rh5FGmH\nelTsPVAPeB+Nd9LzuZhpPQRP1StxLWSrDRrIwBC12a49H+pz6nP2kdHRAoGAG0Er\nxGsemNGfpZk3MDYTRebO/GKTrNJlEBOXAvUctwi6h0nRVAu3qyWY0xdkg9gkp8n3\nzh1K14sG8JjX32SIkeS0v102U9V/WXOYpDNXk0SWZzVd439GUzAbQIHcCaCxQgB6\nyGYsvPL3RozMd1YCirtN7uVqBdxTKIvBtRi3i68CgYBeER5yGkQDwUjM1CJhpcFk\npACO+KId43apzfzmJzbLjFKIx0aPy/Gye3B9xA8HGMgMHkub8YWGgJio17ojvXm4\nvAsLZ7Lz73rkIoa2cnqZg/VOoIb77WEJ2rAsLkkzHfMajlIQT06Lc/1DUMN60IKy\nn7hd0F5yJOW6JQnvcKc4Wg==\n-----END PRIVATE KEY-----\n",
            "client_email": "firebase-adminsdk-vokzj@marselotech-web.iam.gserviceaccount.com",
            "client_id": "117057869359564983002",
            "auth_uri": "https://accounts.google.com/o/oauth2/auth",
            "token_uri": "https://oauth2.googleapis.com/token",
            "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
            "client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/firebase-adminsdk-vokzj%40marselotech-web.iam.gserviceaccount.com"
        }

        #Inicializa la base de datos

        self.firebase = pyrebase.initialize_app(self.config)
        self.db1 = self.firebase.database()

        

        self.cred = credentials.Certificate(jsonObject)
        firebase_admin.initialize_app(self.cred, { 'storageBucket' : 'marselotech-web.appspot.com' })
        self.db = firestore.client()


        self.bucket = storage.bucket()
        



    def camera_callback(self,data):
        """ Esta función devuelve un callback, muestra en una ventana una imagen cuando
            detecta una cara (en simulación)

        Args: 
            data : Imagen que recibe del robot en forma de matriz (píxeles)

        """


        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            cv2.imwrite("/home/john/image.jpg", cv_image)
            self.funciona=True
        except CvBridgeError as e:
            print(e)

    def marselotech_my_service_callback(self, request, response):
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response
        respuesta = False

        print(request.type)


        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            img=imread("/home/john/image.jpg")
            hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)   
            self.funciona=True
        except CvBridgeError as e:
                print(e)

        
        if(self.funciona):
            if request.type == "caras":
                while(not respuesta):
                    respuesta=self.capturar_caras()
                response.success=True
               
            elif request.type == "color":
                while(not respuesta):
                    respuesta=self.detectar_verde()
                response.success=True
            elif request.type == "personas":
                while(not respuesta):
                    respuesta=self.capturar_personas()
                response.success=True
            
            elif request.type == "armas":
                while(not respuesta):
                    respuesta=self.capturar_armas()
                response.success=True
            
            else:
                # estado de la respuesta
                # si no se ha dado ningun caso anterior
                response.success = False
        else:
            # estado de la respuesta
            # si no se ha dado ningun caso anterior
            response.success = False


        # devuelve la respuesta
        return response




    def capturar_armas(self):

        photo=imread("/home/john/image.jpg")
        imagesrc=open("/home/john/image.jpg", 'rb')
        client=boto3.client('rekognition','us-east-1')

        response = client.detect_labels(Image={'Bytes':imagesrc.read()})

        res=False
        #print('Detected labels for ' + photo) 
        for label in response['Labels']:
            if(label['Name'] == 'Weapon' and (label['Confidence'])>= 50.00):
                cv2.imwrite("armas.jpg",photo)
                #Subir una imagen a Storage
                storage = self.firebase.storage()
                storage.child("arma").put("armas.jpg")
                self.upload_image("armas.jpg", "arma");
                res=True
        
        if(res):
            return True
        return False

    def capturar_caras(self):
        img=imread("/home/john/image.jpg")
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        face_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_frontalface_default.xml')
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


        caras = face_cascade.detectMultiScale(img_gray, 1.1, 5)

        ncaras = 0
        for (x,y,w,h) in caras:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = img[y:y+h, x:x+w]
            ncaras=ncaras+1


        if(ncaras>0):
            cv2.imshow("Imagen capturada por el robot", img) #muestra la imagen
            
            cv2.imwrite("Imagen_capturada.jpg",img) #la guarda
            imageSource=open("Imagen_capturada.jpg",'rb')

            img = imread("Imagen_capturada.jpg")
            cv2.imshow("Imagen capturada por el robot", img)


            response=self.client.search_faces_by_image(CollectionId=self.collectionId,
                            Image={'Bytes': imageSource.read()},
                            FaceMatchThreshold=self.threshold,
                            MaxFaces=self.maxFaces)

            faceMatches=response['FaceMatches']
            print ('Matching faces')
            texto=""
            fuente = cv2.FONT_HERSHEY_SIMPLEX
            color = (0,0,0)
            for match in faceMatches:
                if(match['Similarity']>90):
                    self.known=True
                    if(match['Face']['ExternalImageId'] in self.friendlist):
                        texto = match['Face']
                        print ('Se ha detectado a :'  + match['Face']['ExternalImageId'])
                        print ('Similarity: ' + "{:.2f}".format(match['Similarity']) + "%")
                        
                    else:
                        texto = match['Face']
                        print ('Se ha detectado a :'  + match['Face']['ExternalImageId'])
                        print ('Similarity: ' + "{:.2f}".format(match['Similarity']) + "%")
                        self.enemy = 1

            if(self.known):
                if(self.enemy):
                    print("Enemigo detectado")

                    cv2.imwrite("caras.jpg",img) #la guarda
                    #Subir una imagen a Storage
                    storage = self.firebase.storage()
                    storage.child("caras").put("caras.jpg")
                else:
                    print("Aliado detectado")

                    cv2.imwrite("caras.jpg",img) #la guarda
                    #Subir una imagen a Storage
                    storage = self.firebase.storage()
                    storage.child("caras").put("caras.jpg")
            else:
                cv2.putText(img,"Intruso detectado",0,fuente,1,color,2)
                print("Intruso detectado")

                cv2.imwrite("caras.jpg",img) #la guarda
                #Subir una imagen a Storage
                storage = self.firebase.storage()
                storage.child("caras").put("caras.jpg")     
            
            self.upload_image("caras.jpg", "caras")
            self.enemy=0
            self.known=False

            return True
        else:
            self.enemy=0
            self.known=False
            return False

    def upload_image(self,name, type):

        blob = self.bucket.blob(name) #blob
        storage=self.firebase.storage()
        url =  storage.child(type).get_url(name)

        now = datetime.datetime.now()

        current_time = now.strftime("%H:%M:%S")
        data = {
            u'img': url,
            u'robotid': u'1',
            u'time': current_time,
            u'type': type
        }


        self.db.collection(u'images').add(data)

    def capturar_personas(self):
        photo=imread("/home/john/image.jpg")
        imagesrc=open("/home/john/image.jpg", 'rb')
        client=boto3.client('rekognition','us-east-1')

        response = client.detect_labels(Image={'Bytes':imagesrc.read()})

        res=False
        #print('Detected labels for ' + photo) 
        for label in response['Labels']:
            if(label['Name'] == 'Human' and (label['Confidence'])>= 70.00):
                cv2.imwrite("armas.jpg",photo)
                #Subir una imagen a Storage
                storage = self.firebase.storage()
                storage.child("arma").put("armas.jpg")
                self.upload_image("armas.jpg", "arma");
                res=True
        
        if(res):
            return True
        return False
        
       

    def detectar_verde(self):    
    
        """ Función que se activa al detectar una imagen por el topic /camre/image_raw
        Permite mostrar por pantalla la imagen resultante con un rectángulo

        Args: 
        data(imagen)

        """

        img=imread("/home/john/image.jpg")

                    
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #Detectar persona
        photo=imread("/home/john/image.jpg")
        imagesrc=open("/home/john/image.jpg", 'rb')
        client=boto3.client('rekognition','us-east-1')

        response = client.detect_labels(Image={'Bytes':imagesrc.read()})

        pers=False
        #print('Detected labels for ' + photo) 
        for label in response['Labels']:
            if(label['Name'] == 'Human' and (label['Confidence'])>= 70.00):
                cv2.imwrite("armas.jpg",photo)
                #Subir una imagen a Storage
                storage = self.firebase.storage()
                storage.child("arma").put("armas.jpg")
                self.upload_image("armas.jpg", "arma");
                pers=True
        

        #Termina detectar persona


        lower_green = np.array([36, 25, 25])
        upper_green = np.array([86, 255, 255])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        	
        apertura = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        mask = apertura
        
        res = cv2.bitwise_and(img, img, mask= mask)

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
        print("La x1 es:" + str(x1))
        y1 = x_final
        print("La y1 es:" + str(y1))
        x2 = y_final
        print("La x2 es:" + str(x2))
        y2 = x_inicial
        print("La y2 es:" + str(y2))

        color = (0,0,255)

        if(x1 != 1000 or y1 != 0):
            if(pers):
                rectangulo = cv2.rectangle(img, (x1,y1), (x2,y2), color, 2)

                cv2.imwrite("color.jpg",img) #la guarda
                #Subir una imagen a Storage
                storage = self.firebase.storage()
                storage.child("color").put("color.jpg")     
                
                self.upload_image("color.jpg", "color")
                print("Se ha detectado una persona de mi equipo")
                return True
            else:
                print("No hay personas verdes")
                return False

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