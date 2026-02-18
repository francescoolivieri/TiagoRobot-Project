import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import time
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

PI = 3.14
ROTATION_VELOCITY = -0.5

class InitialPositionNode(Node):
    def __init__(self):
        super().__init__('initial_position_node')
        
        # Initial guess fot the pose
        self.initial_pose_x_guess, self.initial_pose_y_guess, self.initial_pose_z_guess, self.initial_pose_yaw_guess = 0.0, 0.0, 0.0, 0.0
        
        self.odom_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose',
            self.amcl_callback, 10)
        self.amcl_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.odometry_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan_raw', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.latest_scan = None
        self.updated_scan = False
        self.covariance_threshold = 0.07
        # Initialize with high uncertainty for both messages
        cov = np.zeros(36, dtype=np.float64)
        cov[0] = 200.0   
        cov[7] = 200.0   
        cov[35] = 50.0  # very high uncertainty in yaw (full rotation)
        self.covariance_values = cov
        # Initialize covariance_msg with high values too
        self.covariance_msg = PoseWithCovarianceStamped()
        self.covariance_msg.pose.covariance = cov.tolist()
        
        # Get initial pose parameters
        initial_x = self.initial_pose_x_guess
        initial_y = self.initial_pose_y_guess
        initial_z = self.initial_pose_z_guess
        initial_yaw = self.initial_pose_yaw_guess
        
        # Convert yaw to quaternion
        from math import sin, cos
        qz = sin(initial_yaw / 2.0)
        qw = cos(initial_yaw / 2.0)
        
        self.tb3_pose = [initial_x, initial_y, initial_z]
        self.tb3_orientation = [0.0, 0.0, 0.0, 0.0]
        self.odom_received = False
        
        # Initialize AMCL pose attributes
        self.amcl_position = None
        self.amcl_orientation = None
        self.amcl_received = False
        self.rotation = True
        self.get_logger().info('Initialization complete...')
        # Spin once to handle callbacks
        rclpy.spin_once(self, timeout_sec=1)

    def amcl_callback(self, msg):
        self.amcl_position = msg.pose.pose.position
        self.amcl_orientation = msg.pose.pose.orientation
        self.covariance_msg.pose.covariance = msg.pose.covariance
        self.amcl_received = True

    def odom_callback(self, odom_msg):
        x_o = odom_msg.pose.pose.orientation.x
        y_o = odom_msg.pose.pose.orientation.y
        z_o = odom_msg.pose.pose.orientation.z
        w_o = odom_msg.pose.pose.orientation.w
        
        self.tb3_orientation = [x_o, y_o, z_o, w_o]
        
        self.odom_received = True
    
    def scan_callback(self, msg):
        # Store latest laser scan data 
        self.latest_scan = msg
        self.updated_scan = True
        

    def publish_initial_pose(self):
        # Wait for odometry to get proper orientation
        max_wait = 10
        wait_count = 0
        while not self.odom_received and wait_count < max_wait:
            self.get_logger().warn('Waiting for odometry data...')
            rclpy.spin_once(self, timeout_sec=1)
            wait_count += 1
        
        if not self.odom_received:
            self.get_logger().warn('Odometry not received, send default pose...')
        
        # Validate quaternion (should not be all zeros)
        quat_sum = sum([abs(x) for x in self.tb3_orientation])
        if quat_sum < 0.01:  # Nearly zero quaternion is invalid
            self.get_logger().error('Invalid quaternion (near zero)! Setting to default: (0,0,0,1)')
            self.tb3_orientation = [0.0, 0.0, 0.0, 1.0]
        
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
        # Wait for AMCL messages before starting localization
        max_wait_time = 10  # seconds
        wait_start = time.time()
        while not self.amcl_received and (time.time() - wait_start) < max_wait_time:
            self.get_logger().warn('Waiting for AMCL pose messages...')
            rclpy.spin_once(self, timeout_sec=1)
        
        if not self.amcl_received:
            self.get_logger().error('AMCL pose messages not received!')
            return
        
        # Wait for scan data        
        wait_start = time.time()
        while self.latest_scan is None and (time.time() - wait_start) < max_wait_time:
            self.get_logger().warn('Waiting for laser scan data...')
            rclpy.spin_once(self, timeout_sec=1)
    
    
        while True:
            print('Localization in progress...')
            self.rotate(2*PI)
            # update the covariance matrix from the topic amcl
            rclpy.spin_once(self, timeout_sec=1)
            
            if self.check_covariance():
                break
            
            # If not localized, find free direction and move towards it
            free_direction = self.find_free_direction()
            if free_direction:
                angle, distance = free_direction
                self.move_to_obstacle(angle, target_distance=1.0)
                rclpy.spin_once(self, timeout_sec=1) #?
                
                if self.check_covariance():
                    break

    def check_covariance(self):
        covariance_values = self.covariance_msg.pose.covariance
        max_cov = np.max(covariance_values)
        
        
        cov_x = covariance_values[0]  # x position covariance
        cov_y = covariance_values[7]  # y position covariance
        cov_yaw = covariance_values[35]  # yaw covariance
        
        #debug
        self.get_logger().info(f'Covariance - X: {cov_x:.4f}, Y: {cov_y:.4f}, Yaw: {cov_yaw:.4f}, Max: {max_cov:.4f}')
        
        if max_cov < self.covariance_threshold:
            self.get_logger().info("Covariance below the threshold.")
            self.get_logger().info("Robot is localized.")
            if self.amcl_position is not None:
                self.get_logger().info(f'Final AMCL position: x={self.amcl_position.x:.2f}, y={self.amcl_position.y:.2f}')
            return True
        else:
            self.get_logger().warn("Covariance above the threshold.")
            return False
    
    def find_free_direction(self):
        #Find the direction with the most free space
        if self.latest_scan is None:
            self.get_logger().warn('No scan data available')
            return None
        
        ranges = self.latest_scan.ranges
        num_ranges = len(ranges)
        
        # Divide scan into 8 sectors and find which has most space
        sector_size = num_ranges // 8
        best_sector = -1
        max_avg_distance = 0.0
        
        for i in range(8):
            start_idx = i * sector_size
            end_idx = start_idx + sector_size
            sector_ranges = ranges[start_idx:end_idx]
            
            # Filter out invalid readings (inf or nan)
            valid_ranges = [r for r in sector_ranges if self.latest_scan.range_min < ranges[i] < self.latest_scan.range_max]
            
            if valid_ranges:
                avg_distance = sum(valid_ranges) / len(valid_ranges)
                if avg_distance > max_avg_distance:
                    max_avg_distance = avg_distance
                    best_sector = i
                    
        # should add additional rotation if everything is occupied
        if best_sector == -1:
            self.get_logger().info(f'Could NOT find feasible direction. Rotating a bit more...')
            self.rotate(PI/2)
            time.sleep(0.5) # to get new laser measurements
            self.find_free_direction()
        
        # Convert sector to angle (0 = front, counter-clockwise)
        angle = self.latest_scan.angle_min + (best_sector * sector_size + sector_size/2) * self.latest_scan.angle_increment
        
        self.get_logger().info(f'Found free direction at angle {angle:.2f} rad with avg distance {max_avg_distance:.2f}m')
        return angle, max_avg_distance
    
    def move_to_obstacle(self, target_angle, target_distance=1.0):
        """Move in the specified direction until target_distance from obstacle"""
        LINEAR_SPEED = 0.25
        
        # First, rotate to face the target direction
        self.get_logger().info(f'Rotating to target angle {target_angle:.2f} rad')
        
        self.rotate(target_angle)
        # vel_msg = Twist()
        # vel_msg.angular.z = ANGULAR_SPEED if target_angle > 0 else -ANGULAR_SPEED
        
        # rotation_time = abs(target_angle) / ANGULAR_SPEED
        # start_time = time.time()
        
        # while (time.time() - start_time) < rotation_time:
        #     self.publisher.publish(vel_msg)
        #     rclpy.spin_once(self, timeout_sec=0.01)
        
        #self.stop()
        time.sleep(0.5) # to get new laser measurements
        
        # Now move forward until 0.5m from obstacle
        self.get_logger().info('Moving toward obstacle...')
        vel_msg = Twist()
        vel_msg.linear.x = LINEAR_SPEED
        
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.latest_scan is None or self.updated_scan is False: # can also regulate the loop based on msg.scan_time + small delay
                continue
            
            # Check front distance (centered region)
            ranges = self.latest_scan.ranges
            num_ranges = len(ranges)
            print(f"angle_min: {self.latest_scan.angle_min}, angle_max: {self.latest_scan.angle_max}, angle_increment: {self.latest_scan.angle_increment} ")
            center_index = int((-self.latest_scan.angle_min)/self.latest_scan.angle_increment)
            front_indices = list(range(center_index - 40, center_index)) + list(range(center_index, center_index + 40))
            front_ranges = [ranges[i] for i in front_indices if self.latest_scan.range_min < ranges[i] < self.latest_scan.range_max] 
            
            if front_ranges:
                min_front_distance = min(front_ranges)
                self.get_logger().info(f'Distance to obstacle: {min_front_distance:.2f}m')
                
                if min_front_distance <= target_distance:
                    self.get_logger().info(f'Reached target distance: {min_front_distance:.2f}m')
                    break
            
            self.publisher.publish(vel_msg)
            self.updated_scan = False            
        
        self.stop()
        self.get_logger().info('Movement complete!')

    def rotate(self, angle):
                
        start_time = self.get_clock().now()

        if self.rotation:
            vel_msg = Twist() #create a velocity message
            vel_msg.angular.z = ROTATION_VELOCITY
            vel_msg.linear.y = -0.3
            
            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < (- angle / vel_msg.angular.z):
                self.publisher.publish(vel_msg)
                rclpy.spin_once(self, timeout_sec=0.01)
                
        #     self.rotation = False
        # else:
        #     vel_msg = Twist() #create a velocity message
        #     vel_msg.linear.y = 0.6
            
        #     while ((self.get_clock().now() - start_time).nanoseconds / 1e9) < 5:
        #         if ((self.get_clock().now() - start_time).nanoseconds / 1e9) < 3.5:
        #             self.publisher.publish(vel_msg)
        #         else:
        #             vel_msg.linear.x = -0.6
        #             vel_msg.linear.y = 0.3
        #             self.publisher.publish(vel_msg)
        #         rclpy.spin_once(self, timeout_sec=0.01)
            
        #     self.rotation = True
        #time.sleep(- TWO_PI / vel_msg.angular.z) # rotate for some time
        
        
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
    initial_position_node.get_logger().info('Starting localization procedure...')
    initial_position_node.localization()
    initial_position_node.get_logger().info('Localization complete!')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
