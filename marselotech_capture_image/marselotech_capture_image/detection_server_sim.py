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
from firebase_admin import credentials, storage
import datetime





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
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.bucket="marselotech"
        self.collectionId="caras"
        self.threshold = 1
        self.maxFaces=3
        self.enemy=0
        self.known=False
        self.friendlist = ['Belen', 'Leire', 'Jose']
        self.client=boto3.client('rekognition', 'us-east-1')
        self.kernel = np.ones((7,7),np.uint8)

        #Credenciales para la comunicación FIREBASE
        self.config = {
            "apiKey": "AIzaSyB5H63Brfoe6WKs8t2v-5WXVP70c8X8hm4",
            "authDomain": "marselotech-web.firebaseapp.com",
            "databaseURL": "https://marselotech-web-default-rtdb.firebaseio.com",
            "storageBucket": "marselotech-web.appspot.com"
        }

        #Inicializa la base de datos

        self.firebase = pyrebase.initialize_app(self.config)
        self.db = self.firebase.database()

        self.cred = credentials.Certificate("./key.json")
        app = firebase_admin.initialize_app(self.cred, { 'storageBucket' : 'marselotech-web.appspot.com' })

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
            cv2.imwrite("/home/belen/image.jpg", cv_image)
        except CvBridgeError as e:
            print(e)

    def marselotech_my_service_callback(self, request, response):
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response
        respuesta = False

        print(request.type)

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
        
        else:
            # estado de la respuesta
            # si no se ha dado ningun caso anterior
            response.success = False

        # devuelve la respuesta
        return response

    def capturar_caras(self):
        img=imread("/home/belen/image.jpg")
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
                    cv2.putText(img,"Enemigo detectado: "+texto,0,fuente,1,color,2)
                    print("Enemigo detectado")

                    cv2.imwrite("result.jpg",img) #la guarda
                    #Subir una imagen a Storage
                    storage = self.firebase.storage()
                    storage.child("images/foto.jpg").put("result.jpg")
                else:
                    cv2.putText(img,"Aliado detectado: "+texto,0,fuente,1,color,2)
                    print("Aliado detectado")

                    cv2.imwrite("result.jpg",img) #la guarda
                    #Subir una imagen a Storage
                    storage = self.firebase.storage()
                    storage.child("images/foto.jpg").put("result.jpg")
            else:
                cv2.putText(img,"Intruso detectado",0,fuente,1,color,2)
                print("Intruso detectado")

                cv2.imwrite("result.jpg",img) #la guarda
                #Subir una imagen a Storage
                storage = self.firebase.storage()
                storage.child("images").put("result.jpg")     
            
            self.upload_image()
            self.enemy=0
            self.known=False

            return True
        else:
            self.enemy=0
            self.known=False
            return False

    def upload_image(self):

        blob = self.bucket.get_blob("result.png") #blob

        url = blob.generate_signed_url(
                version="v4",
                # This URL is valid for 15 minutes
                expiration=datetime.timedelta(minutes=15),
                # Allow GET requests using this URL.
                method="GET",
        )
        now = datetime.now()

        current_time = now.strftime("%H:%M:%S")
        data = {
            u'img': url,
            u'robotid': u'1',
            u'time': current_time
        }

        # Add a new doc in collection 'cities' with ID 'LA'

        self.db.collection(u'cities').document(current_time).set(data)

    def capturar_personas(self):
        img=imread("/home/belen/image.jpg")

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        people_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_fullbody.xml')
        people_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

        personas = people_cascade.detectMultiScale(img_gray, 1.1, 2)

        npersonas = 0
        for (x,y,w,h) in personas:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = img[y:y+h, x:x+w]
            npersonas=npersonas+1


        if(npersonas>0):
            cv2.imshow("PRUEBA", img)
            
            print("Intruso detectado!!!")


            cv2.imwrite("result.jpg",img) #la guarda
            #Subir una imagen a Storage
            storage = self.firebase.storage()
            storage.child("images").put("result.jpg")     
            
            self.upload_image()

            return True


        else:
            print(npersonas)
            return False
        
        cv2.waitKey(1)

    def detectar_verde(self):    
    
        """ Función que se activa al detectar una imagen por el topic /camre/image_raw
        Permite mostrar por pantalla la imagen resultante con un rectángulo

        Args: 
        data(imagen)

        """

        img=imread("/home/belen/image.jpg")

                    
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #Detectar persona

        # Cargamos las librerías
        people_cascade = cv2.CascadeClassifier('clasificadores/haarcascade_fullbody.xml')
        people_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

        # Detecta las personas de la imagen en escala de grises
        personas = people_cascade.detectMultiScale(img_gray, 1.1, 2)

        # Por cada persona detectada se dibuja un rectángulo a su al rededor
        npersonas = 0
        for (x,y,w,h) in personas:
            #cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_color = img[y:y+h, x:x+w]
            npersonas=npersonas+1

        #Termina detectar persona


        lower_green = np.array([36, 25, 25])
        upper_green = np.array([86, 255, 255])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        	
        apertura = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        cv2.imshow('funcionapordios', apertura)

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
        y1 = x_final
        x2 = y_final
        y2 = x_inicial

        color = (0,0,255)

        if(npersonas>0):
            rectangulo = cv2.rectangle(img, (x1,y1), (x2,y2), color, 2)
            cv2.imshow('Verde detectado',rectangulo)

            cv2.imwrite("result.jpg",img) #la guarda
            #Subir una imagen a Storage
            storage = self.firebase.storage()
            storage.child("images").put("result.jpg")     
            
            self.upload_image()
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