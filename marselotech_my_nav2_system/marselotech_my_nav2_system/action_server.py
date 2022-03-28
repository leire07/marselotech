# action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time

class MyActionServer(Node):

    def __init__(self):
        super().__init__('my_action_server')
        #creamos el servidor de la accion
        #con parametros : 
        # nodo servidor,
        # tipo de mensaje
        # nombre de la accion
        # funcion a ejecutar
        self.action_server = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute_callback )
        # creamos objeto tipo Twist para enviar la velocidad del robot
        self.cmd = PoseStamped()
        #creamos el publisher para el topic cmd_vel con parametros:
        # tipo de mensaje
        # nombre del topic
        # tamaño de la cola
        self.publisher = self.create_publisher(PoseStamped, 'posestamped', 1)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Recibiendo el Goal...')

        feedback_msg = NavigateToPose.Feedback()
        feedback_msg.feedback = "Moviendo el robot a la izquierda..."
        self.cmd.pose.position.x = 1
        self.cmd.pose.position.y = 1
        self.cmd.pose.position.w = 0
        
        self.publisher.publish(self.cmd)
        feedback_msg.feedback = "¡Accion finalizada!"
        result = NavigateToPose.Result()
        result.status = feedback_msg.feedback
        return result

def main(args=None):
    rclpy.init(args=args)

    my_action_server = MyActionServer()

    rclpy.spin(my_action_server)

if __name__=='__main__':
    main()