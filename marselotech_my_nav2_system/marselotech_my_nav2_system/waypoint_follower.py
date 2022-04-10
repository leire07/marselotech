import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import Pose, PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.qos import ReliabilityPolicy, QoSProfile
   



class WayPointFollower(Node):

    def _init_(self):
        super()._init_('my_action_client')
        #creamos el objeto cliente de una accion
        #con parametros
        #nodo
        #tipo de mensaje
        #nombre de la accion
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    #definimos la funcion de mandar goal
    def send_goal(self, poses):
        # crea el mensaje tipo Goal
        # y lo rellena con el argumento dado
        self.get_logger().info('TEST SEND_GOAL :O')
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
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
        else:
            self.get_logger().info('Goal success!')
        
        self.__reset_action()

    #definimos la funcion de respuesta al feedback
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    
'''
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
'''
def main(args=None):
 
  rclpy.init(args=args)

  navigator = WayPointFollower()


  goal_poses = []
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -1.0
  goal_pose.pose.position.y = 0.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.23
  goal_pose.pose.orientation.w = 0.97
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -1.5
  goal_pose.pose.position.y = 1.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.707
  goal_pose.pose.orientation.w = -0.707
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -1.5
  goal_pose.pose.position.y = 1.5
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.92
  goal_pose.pose.orientation.w = -0.38
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -2.5
  goal_pose.pose.position.y = 1.5
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.92
  goal_pose.pose.orientation.w = 0.38
  goal_poses.append(goal_pose)
  
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -3.2
  goal_pose.pose.position.y = 1.5
  goal_pose.pose.position.z = 1.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.38
  goal_pose.pose.orientation.w = 0.92
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -3.0
  goal_pose.pose.position.y = 0.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  goal_poses.append(goal_pose)
 
  
  future = navigator.send_goal(goal_poses) # se para secs como argumento

  rclpy.spin(navigator)
  
if _name_ == '_main_':
  main()