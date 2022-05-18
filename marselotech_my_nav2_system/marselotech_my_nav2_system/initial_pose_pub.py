# initial_pose_pub.py
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

class Publisher(Node):


    qos_profile = QoSProfile(depth = 10)
    qos_profile.reliability = QoSReliabilityPolicy.RELIABLE  # RELIABLE o BEST_EFFORT
    qos_profile.durability = QoSDurabilityPolicy.VOLATILE  # VOLATILE o TRANSIENT_LOCAL
    qos_profile.history = QoSHistoryPolicy.KEEP_LAST # KEEP_ALL o KEEP_LAST
    qos_profile.liveliness = QoSLivelinessPolicy.AUTOMATIC # MANUAL BY TOPIC o AUTOMATIC
    qos_profile.deadline = Duration(seconds = 2.0)

    def __init__(self):
        super().__init__('initial_pose_pub_node')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', self.qos_profile )
        timer_period = 0.5  # seconds
        self.timer_ = self.create_timer(timer_period, self.callback)

    def callback(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(sys.argv[1]) #0.7 #0.45
        msg.pose.pose.position.y = float(sys.argv[2]) #2.75 #2.75
        msg.pose.pose.orientation.w = float(sys.argv[3]) #1.0 #1.0
        self.get_logger().info('Publishing  Initial Position')
        self.get_logger().info(str(float(sys.argv[1])))
        self.get_logger().info(str(float(sys.argv[2])))
        self.get_logger().info(str(float(sys.argv[3])))
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    try:
        rclpy.spin_once(publisher)
    except KeyboardInterrupt:
        publisher.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
