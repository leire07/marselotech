#action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, PoseStamped
from marselotech_custom_interface.srv import NavToPoseMsg
from action_msgs.msg import GoalStatus
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time
import sys


"""
Este módulo incluye el código para la navegación automática a un punto

Classes:
  NavToPose

"""
class NavToPose(Node):

    """
    Navegación automática a un punto

    Attributes:
        action_client (action_client): El cliente de la acción

    
    Methods:
        send_goal(): Recibe el PoseStamped y manda el mensaje al topic para la navegación automática
        goal_response_callback(): Responde ante el goal si se ha aceptado o no
        get_result_callback(): Respuesta al resultado
    """

    def __init__(self):
        super().__init__('my_action_client')
        #creamos el objeto cliente de una accion
        #con parametros
        #nodo
        #tipo de mensaje
        #nombre de la accion
        self.srv = self.create_service(NavToPoseMsg, 'navigate', self.marselotech_my_nav_service_callback)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


    def marselotech_my_nav_service_callback(self, request, response):

        
        self.goal_pose.pose.position.x = float(request.x)
        self.goal_pose.pose.position.y = float(request.y)
        self.goal_pose.pose.orientation.w = 0.0  

        future = self.send_goal(self.goal_pose) # se para secs como argumento
        response=True
        return response

    #definimos la funcion de mandar goal
    def send_goal(self, pose):
        """ Función quemanda el goal al topic nav_to_pose

        Args: 
            pose (PoseStamped): Posición final del robot
        
        Returns:
            bool: Si el goal se ha aceptado o no

        """

        # crea el mensaje tipo Goal
        # y lo rellena con el argumento dado
        self.get_logger().info('TEST SEND_GOAL :O')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        #espera a que el servidor este listo
        self.action_client.wait_for_server()
        # envia el goal
        self._send_goal_future = self.action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    #definimos la funcion de respuesta al goal
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    #definimos la funcion de respuesta al resultado
    def get_result_callback(self, future):
        self.status = future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation failed with status code: {0}'.format(self.status))

            self.send_goal(self.goal_pose)
        else:
            self.get_logger().info('Goal success!')
        
        #self.__reset_action()

    #definimos la funcion de respuesta al feedback
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPose()

    try:
        #dejamos abierto el servicio
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Cerrando el nodo service')
    finally:
        #destruimos el nodo
        action_client.destroy_node()
        #cerramos la comunicacion
        rclpy.shutdown()

    

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
