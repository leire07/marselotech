#action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time


class NavToPose(Node):

    def __init__(self):
        super().__init__('my_action_client')
        #creamos el objeto cliente de una accion
        #con parametros
        #nodo
        #tipo de mensaje
        #nombre de la accion
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    #definimos la funcion de mandar goal
    def send_goal(self, pose):
        # crea el mensaje tipo Goal
        # y lo rellena con el argumento dado
        self.get_logger().info('TEST SEND_GOAL :O')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        #espera a que el servidor este listo
        self._action_client.wait_for_server()
        # envia el goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)

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
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = -1.0
            goal_pose.pose.position.y = 2.5
            goal_pose.pose.orientation.w = 0.0
            self.send_goal(goal_pose)
        else:
            self.get_logger().info('Goal success!')
        
        self.__reset_action()

    #definimos la funcion de respuesta al feedback
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPose()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = action_client.get_clock().now().to_msg()
    goal_pose.pose.position.x = -1.0
    goal_pose.pose.position.y = 2.5
    goal_pose.pose.orientation.w = 0.0

    future = action_client.send_goal(goal_pose) # se para secs como argumento

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
