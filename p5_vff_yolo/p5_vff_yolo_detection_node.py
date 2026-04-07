import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('vff_yolo_detection_node')

        self.state = 'looking_for'
        self.distance_obstacle = 10.0
        self.angle_obstacle = 0.0

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.turning_time = Duration(seconds = 1.0)

        self.state_ts = self.get_clock().now()

    def control_loop(self):

        twist = Twist()
            
        if self.state == 'looking_for':
            self.state_ts = self.get_clock().now()
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        self.vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

