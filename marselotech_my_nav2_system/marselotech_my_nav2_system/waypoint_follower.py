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
   


"""
Este módulo incluye el código para la navegación automática a una serie de puntos

Classes:
  WayPointFollower

"""

class WayPointFollower(Node):

    """
    Navegación automática por una ruta

    Attributes:
        action_client (action_client): El cliente de la acción para los waypoints

    
    Methods:
        send_goal(): Recibe los puntos y manda el mensaje al topic para la navegación automática
        goal_response_callback(): Responde ante el goal si se ha aceptado o no
        get_result_callback(): Respuesta al resultado
    """

    def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')



    #definimos la funcion de mandar goal
    def send_goal(self, poses):

        """ Función quemanda el goal al topic /FollowWaypoints

        Args: 
            poses (PoseStamped[]): Array con todas las posiciones por las que debe pasar el robot
        
        Returns:
            bool: Si el goal se ha aceptado o no

        """
        self.get_logger().info("Waiting for 'FollowWaypoints' action server")
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.get_logger().info('Following ' + str(len(goal_msg.poses)) + ' goals.' + '...')
        send_goal_future = self._action_client.send_goal_async(goal_msg,
                                                                        self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Following ' + str(len(poses)) + ' waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        self.status = future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation failed with status code: {0}'.format(self.status))
        else:
            self.get_logger().info('Goal success!')
        
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
 
  
  future = navigator.send_goal(goal_poses)

  rclpy.spin(navigator)
  
if __name__ == '__main__':
    main()