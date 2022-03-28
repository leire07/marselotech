import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
import BasicNavigator # Helper module
  
'''
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
'''
def main():
 
  # Start the ROS 2 Python Client Library
  rclpy.init()
 
  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()
 
  navigator.waitUntilNav2Active()


  goal_poses = []
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 1.3
  goal_pose.pose.position.y = 6.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.23
  goal_pose.pose.orientation.w = 0.97
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 2.0
  goal_pose.pose.position.y = -3.5
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.707
  goal_pose.pose.orientation.w = -0.707
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 1.5
  goal_pose.pose.position.y = -7.7
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.92
  goal_pose.pose.orientation.w = -0.38
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -1.4
  goal_pose.pose.position.y = -7.8
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.92
  goal_pose.pose.orientation.w = 0.38
  goal_poses.append(goal_pose)
  
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -2.6
  goal_pose.pose.position.y = -4.5
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.38
  goal_pose.pose.orientation.w = 0.92
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 0.0
  goal_pose.pose.position.y = 0.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
  goal_poses.append(goal_pose)
 
  # sanity check a valid path exists
  # path = navigator.getPathThroughPoses(initial_pose, goal_poses)
 
  nav_start = navigator.get_clock().now()
  navigator.followWaypoints(goal_poses)
 
  i = 0
  while not navigator.isNavComplete():
    ################################################
    #
    # Implement some code here for your application!
    #
    ################################################
 
    # Do something with the feedback
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Executing current waypoint: ' +
            str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
      now = navigator.get_clock().now()
 
      # Some navigation timeout to demo cancellation
      if now - nav_start > Duration(seconds=100000000.0):
        navigator.cancelNav()
 
      # Some follow waypoints request change to demo preemption
      if now - nav_start > Duration(seconds=500000.0):
        goal_pose_alt = PoseStamped()
        goal_pose_alt.header.frame_id = 'map'
        goal_pose_alt.header.stamp = now.to_msg()
        goal_pose_alt.pose.position.x = -6.5
        goal_pose_alt.pose.position.y = -4.2
        goal_pose_alt.pose.position.z = 0.0
        goal_pose_alt.pose.orientation.x = 0.0
        goal_pose_alt.pose.orientation.y = 0.0  
        goal_pose_alt.pose.orientation.z = 0.0
        goal_pose_alt.pose.orientation.w = 1.0
        goal_poses = [goal_pose_alt]
        nav_start = now
        navigator.followWaypoints(goal_poses)
 
  # Do something depending on the return code
  result = navigator.getResult()
  if result == 0:
    print('Goal succeeded!')
  elif result == 1:
    print('Goal was canceled!')
  elif result == 2:
    print('Goal failed!')
  else:
    print('Goal has an invalid return status!')
 
  navigator.lifecycleShutdown()
 
  exit(0)
 
if __name__ == '__main__':
  main()