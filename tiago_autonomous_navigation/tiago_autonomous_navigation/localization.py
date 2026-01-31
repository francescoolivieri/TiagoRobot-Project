import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import time
import numpy as np
from nav_msgs.msg import Odometry

TWO_PI = 6.28
ROTATION_VELOCITY = -0.6

class InitialPositionNode(Node):
    def __init__(self):
        super().__init__('initial_position_node')
        self.odom_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose',
            self.amcl_callback, 10)
        self.amcl_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.odometry_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.covariance_threshold = 0.07
        # Initialize with high uncertainty for both messages
        cov = np.zeros(36, dtype=np.float64)
        cov[0] = 200.0  # high uncertainty in x
        cov[7] = 200.0  # high uncertainty in y
        cov[35] = 200.0   # uncertainty in yaw
        self.covariance_values = cov
        # Initialize covariance_msg with high values too
        self.covariance_msg = PoseWithCovarianceStamped()
        self.covariance_msg.pose.covariance = cov.tolist()
        self.tb3_pose = [0, 0, 0]
        self.tb3_orientation = [0, 0, 0, 0]
        self.get_logger().info('Waiting for the initial position...')
        # Spin once to handle callbacks
        rclpy.spin_once(self, timeout_sec=1)

    def amcl_callback(self, msg):
        self.amcl_position = msg.pose.pose.position
        self.amcl_orientation = msg.pose.pose.orientation
        self.covariance_msg.pose.covariance = msg.pose.covariance

    def odom_callback(self, odom_msg):
        x_o = odom_msg.pose.pose.orientation.x
        y_o = odom_msg.pose.pose.orientation.y
        z_o = odom_msg.pose.pose.orientation.z
        w_o = odom_msg.pose.pose.orientation.w
        cov = np.zeros(36, dtype=np.float64)
        cov[0] = 200
        cov[7] = 200
        cov[35] = 1
        self.covariance_values = cov
        self.tb3_orientation = [x_o, y_o, z_o, w_o]

    def publish_initial_pose(self):
        rclpy.spin_once(self, timeout_sec = 1)
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.pose.pose.position.x = float(self.tb3_pose[0])
        initial_pose_msg.pose.pose.position.y = float(self.tb3_pose[1])
        initial_pose_msg.pose.pose.position.z = float(self.tb3_pose[2])
        initial_pose_msg.pose.pose.orientation.x = \
            float(self.tb3_orientation[0])
        initial_pose_msg.pose.pose.orientation.y = \
            float(self.tb3_orientation[1])
        initial_pose_msg.pose.pose.orientation.z = \
            float(self.tb3_orientation[2])
        initial_pose_msg.pose.pose.orientation.w = \
            float(self.tb3_orientation[3])
        initial_pose_msg.pose.covariance = self.covariance_values.tolist()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.amcl_pose_publisher.publish(initial_pose_msg)

    def localization(self):
        while True:
            print('Localization in progress...')
            self.rotate()
            # update the covariance matrix from the topic amcl
            rclpy.spin_once(self, timeout_sec=1)
            if self.check_covariance():
                break

    def check_covariance(self):
        covariance_values = self.covariance_msg.pose.covariance
        if np.max(covariance_values) < self.covariance_threshold:
            self.get_logger().info("Covariance below the threshold.")
            self.get_logger().info("Robot is localized.")
            return True
        else:
            self.get_logger().warn("Covariance above the threshold.")
            return False

    def rotate(self):
        vel_msg = Twist() #create a velocity message
        vel_msg.angular.z = ROTATION_VELOCITY
        self.publisher.publish(vel_msg)
        self.get_logger().info('Publishing: "%s"' % vel_msg)
        time.sleep(- TWO_PI / vel_msg.angular.z) # rotate for some time
        self.stop() # stop the robot to check the covariance

    def stop(self):
        vel_msg = Twist() #create a velocity message
        vel_msg.angular.z = 0.0
        vel_msg.linear.x = 0.0
        self.publisher.publish(vel_msg)
        self.get_logger().info('Publishing: "%s"' % vel_msg)

def main():
    rclpy.init()
    initial_position_node = InitialPositionNode()
    time.sleep(5)
    initial_position_node.publish_initial_pose()
    initial_position_node.localization()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
