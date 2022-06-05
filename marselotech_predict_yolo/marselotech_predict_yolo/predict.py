import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import cv2
import rclpy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
kernel = np.ones((7,7),np.uint8)

import time
from absl import app, flags, logging
from absl.flags import FLAGS
import numpy as np
import tensorflow as tf
from .models import (
    YoloV3, YoloV3Tiny
)
from .dataset import transform_images, load_tfrecord_dataset
from .utils import draw_outputs
img_robot = None

########
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
            img_robot = cv_image
            cv2.imwrite("image", cv_image)
        except CvBridgeError as e:
            print(e)
        #########

def main(args=None):

    rclpy.init(args=args)

    classes_path = '/home/leire/turtlebot3_ws/src/marselotech/marselotech_predict_yolo/marselotech_predict_yolo/new_names.names' #path to classes file
    weights = '/home/leire/turtlebot3_ws/src/marselotech/marselotech_predict_yolo/marselotech_predict_yolo/checkpoints/yolov3_train_4.tf' #path to weights file
    tiny = False #yolov3 or yolov3-tiny
    size = 416 #resize images to
    image = '/home/leire/turtlebot3_ws/src/marselotech/marselotech_predict_yolo/marselotech_predict_yolo/weapon2.jpg' #path to input image
    if (img_robot != None):
        image = img_robot
    tfrecord = None #tfrecord instead of image
    output = '/home/leire/turtlebot3_ws/src/marselotech//marselotech_predict_yolo/marselotech_predict_yolo/output.png' #path to output image
    num_classes = 1 #number of classes in the model

    physical_devices = tf.config.experimental.list_physical_devices('GPU')
    
    if len(physical_devices) > 0:
        tf.config.experimental.set_memory_growth(physical_devices[0], True)

    if tiny:
        yolo = YoloV3Tiny(classes=num_classes)
    else:
        yolo = YoloV3(classes=num_classes)

    yolo.load_weights(weights).expect_partial()
    logging.info('weights loaded')

    class_names = [c.strip() for c in open(classes_path).readlines()]
    logging.info('classes loaded')

    if tfrecord:
        dataset = load_tfrecord_dataset(
            tfrecord, classes_path, size)
        dataset = dataset.shuffle(512)
        img_raw, _label = next(iter(dataset.take(1)))
        print("el dataset es: " + str(dataset))
    else:
        img_raw = tf.image.decode_image(
            open(image, 'rb').read(), channels=3)

    img = tf.expand_dims(img_raw, 0)
    img = transform_images(img, size)

    t1 = time.time()
    boxes, scores, classes, nums = yolo(img)
    t2 = time.time()
    logging.info('time: {}'.format(t2 - t1))

    logging.info('detections:')
    for i in range(nums[0]):
        logging.info('\t{}, {}, {}'.format(class_names[int(classes[0][i])],
                                           np.array(scores[0][i]),
                                           np.array(boxes[0][i])))

    img = cv2.cvtColor(img_raw.numpy(), cv2.COLOR_RGB2BGR)

    class_weapon = 'mira_weapon'
    if (class_names[0] == class_weapon):
        #Aquí dentro lo que quieres hacer si se detecta el arma
        print("La clase es: " + str(class_names[0]))
    else:
        print("La clase es: " + str(class_names[0]) + " pero no la detecta")

    img = draw_outputs(img, (boxes, scores, classes, nums), class_names)
    cv2.imwrite(output, img)
    logging.info('output saved to: {}'.format(output))
    
    

if __name__ == '__main__':
    main()
