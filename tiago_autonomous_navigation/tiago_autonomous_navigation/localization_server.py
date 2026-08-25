"""
localization_server.py

ROS 2 Action Server that wraps the active-localization logic from
localization.py.  The server:

  1. Requests AMCL global localization, with a high-covariance initial pose.
  2. Spins the robot a full rotation and checks the AMCL covariance.
  3. If the covariance is still above the threshold, drives toward
     the direction with the most free space and repeats.
  4. Publishes covariance feedback every iteration.
  5. Succeeds (result.success = True) once the covariance drops below
     the threshold, or aborts/cancels on preemption / timeout.

The active-localization constants are kept close to localization.py.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from tiago_task2_interfaces.action import Localize


PI = 3.14
ROTATION_VELOCITY = -0.6
MIN_DIST_OBSTACLE = 1.0
FRONT_CHECK_SAMPLES = 45


class LocalizationServer(Node):
    """Action Server that actively localizes the robot."""

    def __init__(self):
        super().__init__('localization_server')

        cb_group = ReentrantCallbackGroup()
        
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose',
            self._amcl_callback, 10, callback_group=cb_group,
        )
        self.amcl_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(
            Odometry, '/mobile_base_controller/odom',
            self._odom_callback, 10, callback_group=cb_group,
        )
        self.create_subscription(
            LaserScan, 'scan_raw',
            self._scan_callback, 10, callback_group=cb_group,
        )

        self.covariance_threshold = 0.03
        cov = np.zeros(36, dtype=np.float64)
        cov[0] = 50.0
        cov[7] = 50.0
        cov[35] = math.pi ** 2
        self._initial_covariance = cov

        self.covariance_msg = PoseWithCovarianceStamped()
        self.covariance_msg.pose.covariance = cov.tolist()

        self.tb3_pose = [0.0, 0.0, 0.0]
        self.tb3_orientation = [0.0, 0.0, 0.0, 1.0]
        
        # Variables for algorithm logic
        self.amcl_position = None
        self.amcl_orientation = None
        self.amcl_received = False
        self.amcl_update_count = 0
        self.odom_received = False
        self.latest_scan = None
        self.updated_scan = False
        self.rotation = True  

        self.global_localization_client = self.create_client(
            Empty,
            'reinitialize_global_localization',
            callback_group=cb_group,
        )

        # Action Server
        self._action_server = ActionServer(
            self,
            Localize,
            'localize',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        # Seed AMCL until it answers so Nav2 can activate the map frame.
        self._startup_timer = self.create_timer(
            1.5, self._publish_initial_pose_until_amcl
        )

        self.get_logger().info('LocalizationServer ready.')


    def _publish_initial_pose_until_amcl(self):
        """Seed AMCL at startup, retrying if DDS missed an earlier message."""
        if self.amcl_received:
            self._startup_timer.cancel()
            return

        self.get_logger().info(
            'Seeding AMCL with initial pose (startup) to unblock Nav2 activation.'
        )
        self._publish_initial_pose()

    
    def _amcl_callback(self, msg):
        self.amcl_position = msg.pose.pose.position
        self.amcl_orientation = msg.pose.pose.orientation
        self.covariance_msg.pose.covariance = msg.pose.covariance
        self.amcl_received = True
        self.amcl_update_count += 1

    def _odom_callback(self, msg):
        x_o = msg.pose.pose.orientation.x
        y_o = msg.pose.pose.orientation.y
        z_o = msg.pose.pose.orientation.z
        w_o = msg.pose.pose.orientation.w
        
        self.tb3_orientation = [x_o, y_o, z_o, w_o]
        self.odom_received = True

    def _scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.updated_scan = True

    # Action Server stuff

    def _goal_callback(self, goal_request):
        self.get_logger().info('Localization goal received.')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Localization cancel requested.')
        return CancelResponse.ACCEPT

    # ── Main execute callback 

    def _execute_callback(self, goal_handle):
        """
        Drives the full localization sequence.
        """
        feedback_msg = Localize.Feedback()
        result = Localize.Result()

        if not self._initialize_amcl():
            self.get_logger().error('AMCL pose not received after initialization. Aborting.')
            goal_handle.abort()
            result.success = False
            return result

        # wait for laser scan
        deadline = time.time() + 10.0
        while self.latest_scan is None and time.time() < deadline:
            self.get_logger().warn('Waiting for laser scan...')
            time.sleep(1.0)

        # ---- localization loop ----
        while rclpy.ok():

            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                result.success = False
                return result

            self.get_logger().info('Localization in progress...')
            self._rotate(2 * PI)

            # Publish feedback with current max covariance
            max_cov = float(np.max(self.covariance_msg.pose.covariance))
            feedback_msg.current_covariance = max_cov
            goal_handle.publish_feedback(feedback_msg)

            if self._check_covariance():
                break

            time.sleep(1.0)

            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                result.success = False
                return result

            # If not yet localized, find a direction to move towards
            free_direction = self._find_free_direction(MIN_DIST_OBSTACLE + 0.2)
            if free_direction:
                angle, distance = free_direction
                self._move_to_obstacle(angle, target_distance=MIN_DIST_OBSTACLE)

                max_cov = float(np.max(self.covariance_msg.pose.covariance))
                feedback_msg.current_covariance = max_cov
                goal_handle.publish_feedback(feedback_msg)

                if self._check_covariance():
                    break

        goal_handle.succeed()
        result.success = True
        self.get_logger().info('Localization complete!')
        return result

    # ── Localization helpers ───────────────────────────────────────────────

    def _initialize_amcl(self) -> bool:
        """Reset AMCL globally and wait for a pose produced after the reset."""
        if self.global_localization_client.wait_for_service(timeout_sec=10.0):
            future = self.global_localization_client.call_async(Empty.Request())
            deadline = time.time() + 10.0

            while not future.done() and time.time() < deadline:
                time.sleep(0.05)

            if future.done() and future.exception() is None:
                self.get_logger().info('AMCL global localization requested.')
                updates_after_reset = self.amcl_update_count
                return self._wait_for_amcl_update(updates_after_reset)

            self.get_logger().warn(
                'AMCL global localization service failed. Using initialpose instead.'
            )
        else:
            self.get_logger().warn(
                'AMCL global localization service is unavailable...'
            )
            rclpy.shutdown()

        self.covariance_msg.pose.covariance = self._initial_covariance.tolist()
        self.amcl_received = False
        self._publish_initial_pose()
        updates_after_initial_pose = self.amcl_update_count
        return self._wait_for_amcl_update(updates_after_initial_pose)

    def _wait_for_amcl_update(self, updates_before: int) -> bool:
        """Wait for AMCL to publish a pose newer than the initialization."""
        deadline = time.time() + 10.0
        while self.amcl_update_count <= updates_before and time.time() < deadline:
            self.get_logger().warn('Waiting for AMCL pose...')
            time.sleep(1.0)

        return self.amcl_update_count > updates_before

    def _publish_initial_pose(self):
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.pose.pose.position.x = float(self.tb3_pose[0])
        initial_pose_msg.pose.pose.position.y = float(self.tb3_pose[1])
        initial_pose_msg.pose.pose.position.z = float(self.tb3_pose[2])
        initial_pose_msg.pose.pose.orientation.x = float(self.tb3_orientation[0])
        initial_pose_msg.pose.pose.orientation.y = float(self.tb3_orientation[1])
        initial_pose_msg.pose.pose.orientation.z = float(self.tb3_orientation[2])
        initial_pose_msg.pose.pose.orientation.w = float(self.tb3_orientation[3])
        initial_pose_msg.pose.covariance = self._initial_covariance.tolist()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.amcl_pose_publisher.publish(initial_pose_msg)
        

    def _check_covariance(self) -> bool:
        """Mirrors InitialPositionNode.check_covariance() exactly."""
        covariance_values = self.covariance_msg.pose.covariance
        max_cov = np.max(covariance_values)

        cov_x   = covariance_values[0]   # x position covariance
        cov_y   = covariance_values[7]   # y position covariance
        cov_yaw = covariance_values[35]  # yaw covariance

        self.get_logger().info(
            f'Covariance - X: {cov_x:.4f}, Y: {cov_y:.4f}, '
            f'Yaw: {cov_yaw:.4f}, Max: {max_cov:.4f}'
        )

        if max_cov < self.covariance_threshold:
            self.get_logger().info('Covariance below the threshold.')
            self.get_logger().info('Robot is localized.')
            if self.amcl_position is not None:
                self.get_logger().info(
                    f'Final AMCL position: '
                    f'x={self.amcl_position.x:.2f}, y={self.amcl_position.y:.2f}'
                )
            return True
        else:
            self.get_logger().warn('Covariance above the threshold.')
            return False

    def _find_free_direction(self, min_dist: float):
        ''' Find the direction with the most free space '''
        if self.latest_scan is None:
            self.get_logger().warn('No scan data available.')
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
            
            ## Filter out invalid readings
            valid_ranges = [r for r in sector_ranges if self.latest_scan.range_min < r < self.latest_scan.range_max]
            
            if valid_ranges:
                avg_distance = sum(valid_ranges) / len(valid_ranges)
                if avg_distance > max_avg_distance and min(valid_ranges) > min_dist:
                    max_avg_distance = avg_distance
                    best_sector = i

        # Let the next localization pass try again instead of recursing forever.
        if best_sector == -1:
            self.get_logger().warn(
                'Could not find a safe direction to move toward.'
            )
            return None

        # Convert sector to angle (0 = front, counter-clockwise)
        angle = self.latest_scan.angle_min + (best_sector * sector_size + sector_size/2) * self.latest_scan.angle_increment
        
        self.get_logger().info(
            f'Found free direction at angle {angle:.2f} rad '
            f'with avg distance {max_avg_distance:.2f}m'
        )
        return angle, max_avg_distance

    def _move_to_obstacle(self, target_angle: float, target_distance: float = 1.0):
        """Move in the specified direction until target_distance from obstacle"""
        LINEAR_SPEED = 0.25

        self.get_logger().info(f'Rotating to target angle {target_angle:.2f} rad')
        self._rotate(target_angle)
        time.sleep(0.5)

        self.get_logger().info('Moving toward obstacle...')
        vel_msg = Twist()
        vel_msg.linear.x = LINEAR_SPEED
        self.updated_scan = False
        deadline = time.time() + 20.0

        while rclpy.ok() and time.time() < deadline:
            if self.latest_scan is None or not self.updated_scan:
                time.sleep(0.01)
                continue

            # Check front distance (centered region)
            ranges = self.latest_scan.ranges
            if self.latest_scan.angle_increment == 0.0:
                self.get_logger().warn('Laser scan has no angular resolution.')
                break

            center_index = int(
                -self.latest_scan.angle_min / self.latest_scan.angle_increment
            )
            first_index = max(0, center_index - FRONT_CHECK_SAMPLES)
            last_index = min(len(ranges), center_index + FRONT_CHECK_SAMPLES + 1)
            front_ranges = [
                distance for distance in ranges[first_index:last_index]
                if self.latest_scan.range_min < distance < self.latest_scan.range_max
            ]
            
            if front_ranges:
                min_front_distance = min(front_ranges)
                self.get_logger().info(
                    f'Distance to obstacle: {min_front_distance:.2f}m'
                )
                if min_front_distance <= target_distance:
                    self.get_logger().info(
                        f'Reached target distance: {min_front_distance:.2f}m'
                    )
                    break

            self.cmd_vel_pub.publish(vel_msg)
            self.updated_scan = False

        if time.time() >= deadline:
            self.get_logger().warn('Movement timed out before reaching the target distance.')

        self._stop()
        self.get_logger().info('Movement complete!')

    def _rotate(self, angle: float):
        """Rotate the robot to the specified angle"""
        start_time = self.get_clock().now()

        if self.rotation and abs(angle) > 0.0:
            vel_msg = Twist()
            speed = abs(ROTATION_VELOCITY)
            vel_msg.angular.z = math.copysign(speed, angle)
            duration = abs(angle) / speed

            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
                self.cmd_vel_pub.publish(vel_msg)
                time.sleep(0.01)

        self._stop()

    def _stop(self):
        """Mirrors InitialPositionNode.stop()."""
        vel_msg = Twist()
        vel_msg.angular.z = 0.0
        vel_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info('Publishing: "%s"' % vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
