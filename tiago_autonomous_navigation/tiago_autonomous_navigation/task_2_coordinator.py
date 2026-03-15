"""
task_2_coordinator.py

State-machine coordinator for Task 2: autonomous discovery of two ArUco
markers (Pick ID=26, Place ID=238) while navigating a known map.

State machine
─────────────
  EXPLORE            Navigate through map waypoints; ArUco callbacks watch
                     concurrently for markers. A detection interrupts nav.

  PROCESS_TARGET     Cancel the current nav goal, call the target_pose_server
                     to obtain the marker pose in the map frame, and store it.
                     → Both found?  go to VERIFY_NAVIGATION
                     → Otherwise?   resume EXPLORE from the same waypoint.

  VERIFY_NAVIGATION  Navigate to the saved PICK pose (ID 26), then to the
                     PLACE pose (ID 238). Task complete.

  DONE               Terminal state.
"""

import time
from collections import deque

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from tiago_task2_interfaces.action import Localize
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tiago_task2_interfaces.srv import GetMarkerPose


PICK_MARKER_ID  = 26
PLACE_MARKER_ID = 238

STATE_INIT_LOCALIZATION  = 'INIT_LOCALIZATION'
STATE_EXPLORE            = 'EXPLORE'
STATE_PROCESS_TARGET     = 'PROCESS_TARGET'
STATE_VERIFY_NAVIGATION  = 'VERIFY_NAVIGATION'
STATE_DONE               = 'DONE'

# clearance from obstacles when generating waypoints (m)
COSTMAP_CLEARANCE_M = 0.7

# Waypoint grid resolution (m)
WAYPOINT_GRID_M = 0.25

# Minimum distance between kept waypoints (m), postprocessing.
WAYPOINT_MIN_DIST_M = 2.0


class Task2Coordinator(Node):
    """
    Task 2 state machine
    """

    def __init__(self):
        super().__init__('task_2_coordinator')

        # Localization action client
        self.localize_client = ActionClient(self, Localize, 'localize')

        # Nav2 action client and variables
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_handle   = None
        self.result_future = None
        self.status        = None
        self.feedback      = None

        # Head joint trajectory action client 
        self.head_client = ActionClient(
            self, FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory'
        )

        # Latched QoS for map and AMCL pose 
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, latched_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.amcl_callback, latched_qos
        )

        # ArUco detection subscribers 
        # aruco_ros "single" publishes ~/pose (private topic), so with
        # name='aruco_single_26' the full topic is /aruco_single_26/pose.
        self.create_subscription(
            PoseStamped, '/aruco_single_26/pose', self._aruco26_callback, 10
        )
        self.create_subscription(
            PoseStamped, '/aruco_single_238/pose', self._aruco238_callback, 10
        )

        # Service client for pose transformation
        self.pose_svc = self.create_client(GetMarkerPose, 'get_marker_pose')

        # State machine
        self.state = STATE_EXPLORE

        # marker_id -> PoseStamped in map frame
        self.found_markers = {}

        # Set by ArUco callbacks
        self.detected_marker_id   = None
        self.detected_marker_pose = None   # camera frame

        # Prevents overwriting detected_marker_id before the state machine handles it
        self.detection_active = True

        # Map data 
        self.map_data  = None   
        self.map_info  = None   
        self.robot_pose = None  # from AMCL

        self.get_logger().info('Task2Coordinator initialised.')

   
    # ----------- Subscription callbacks -----------
    
    def map_callback(self, msg: OccupancyGrid):
        raw = np.array(msg.data, dtype=np.int16)
        raw[raw == -1] = 255          # mark unknown cells as 255 
        self.map_data = raw
        self.map_info = msg.info

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg.pose.pose

    def _aruco26_callback(self, msg: PoseStamped):
        """Fires when aruco marker ID 26 is detected."""
        if self.detection_active and PICK_MARKER_ID not in self.found_markers:
            self.detection_active     = False   # debounce until processed
            self.detected_marker_id   = PICK_MARKER_ID
            self.detected_marker_pose = msg
            self.get_logger().info(f'ArUco marker {PICK_MARKER_ID} detected!')

    def _aruco238_callback(self, msg: PoseStamped):
        """Fires when aruco marker ID 238 is detected."""
        if self.detection_active and PLACE_MARKER_ID not in self.found_markers:
            self.detection_active     = False
            self.detected_marker_id   = PLACE_MARKER_ID
            self.detected_marker_pose = msg
            self.get_logger().info(f'ArUco marker {PLACE_MARKER_ID} detected!')

    
    # ----------- Nav2 helpers ----------------------

    def waitUntilNav2Active(self):
        self.get_logger().info('Wait for Nav2 to be ready...')
        self._waitForNodeToActivate('amcl')
        self._waitForNodeToActivate('bt_navigator')
        self.get_logger().info('Nav2 is ready for use!')
        

    def _waitForNodeToActivate(self, node_name: str):
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(node_service + ' service not available, waiting...')
        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.get_logger().debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
            self.get_logger().debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def lower_head(self, tilt: float = -0.4):
        """
        Tilt Tiago's head downward so the front camera can see aruco markers during exploration.
        """
        
        self.get_logger().info(f'Tilting head: {tilt:.2f} rad ...')

        if not self.head_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('head_controller action server not available, skipping head tilt!')
            return

        point = JointTrajectoryPoint()
        point.positions = [0.0, tilt]
        # Reach the target in 2 seconds
        point.time_from_start = DurationMsg(sec=2, nanosec=0)

        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.head_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self.get_logger().warn('Head tilt goal send timed out.')
            return

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        self.get_logger().info('Head tilt complete.')

    def go_to_pose(self, pose: PoseStamped) -> bool:
        """Send a NavigateToPose goal. Returns True if the goal was accepted."""
        
        self.nav_client.wait_for_server() # removed some messages cause was too verbose

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().info(
            f'Nav goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)

        self.goal_handle = send_goal_future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Nav goal was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def is_nav_complete(self) -> bool:
        if self.result_future is None:  # task was cancelled or completed
            return True
        if self.result_future.done():   # check if thread is done
            self.status = self.result_future.result().status
            return True
        return False

    def _cancel_nav(self):
        if self.goal_handle is None:
            return
        self.get_logger().info('Cancelling navigation goal...')
        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        self.result_future = None
        self.goal_handle   = None

    

    def _wait_for_nav(self, ignore_detection: bool = False) -> str:
        """
        Spin until navigation completes OR (if not ignored) a marker is
        detected by an ArUco callback.

        Returns:
        'MARKER_DETECTED'  - an aruco callback fired first
        'COMPLETE'         - the nav goal finished (succeeded, failed, or aborted)
        'SHUTDOWN'         - rclpy was shut down
        """
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not ignore_detection and self.detected_marker_id is not None:
                return 'MARKER_DETECTED'
            if self.is_nav_complete():
                return 'COMPLETE'
        return 'SHUTDOWN'

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback


    # ----------- Service call helpers -------------------

    def _get_map_pose(
        self, marker_id: int, pose_camera: PoseStamped
    ):
        """
        Call the get_marker_pose service to transform pose_camera -> map frame.
        Returns the map-frame PoseStamped, or None on failure.
        """
        if not self.pose_svc.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '"get_marker_pose" service not available within 5 s!'
            )
            return None

        req = GetMarkerPose.Request()
        req.marker_id = marker_id
        req.pose_in_camera_frame = pose_camera

        future = self.pose_svc.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(
                f'Service call for marker {marker_id} timed out.'
            )
            return None

        result = future.result()
        if not result.success:
            self.get_logger().error(
                f'Service returned failure for marker {marker_id}: {result.message}'
            )
            return None

        return result.pose_in_map_frame
    
    # Localization
    
    def _run_localization(self) -> bool:
        
        if not self.localize_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                'localization_server action server not available after 30 s!'
            )
            return False

        self.get_logger().info('Sending localization goal...')
        goal_msg = Localize.Goal()  # empty goal

        send_future = self.localize_client.send_goal_async(
            goal_msg,
            feedback_callback=self._localize_feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Localize goal was rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result.result.success:
            self.get_logger().info('Localization succeeded!')
            return True
        else:
            self.get_logger().error('Localization action returned success=False.')
            return False

    def _localize_feedback_callback(self, feedback_msg):
        cov = feedback_msg.feedback.current_covariance
        #self.get_logger().info(f'current_covariance: {cov:.4f}')

    

    # ----------- Waypoint generation -------------------


    def _wait_for_map_and_pose(self):
        self.get_logger().info('Waiting for /map and /amcl_pose...')
        while self.map_info is None or self.robot_pose is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info('Map and robot pose received.')

    def _build_waypoints(self) -> list:
        """Generate a filtered grid of reachable, obstacle-free waypoints."""
        robot_col = int(
            (self.robot_pose.position.x - self.map_info.origin.position.x) / self.map_info.resolution
        )
        robot_row = int(
            (self.robot_pose.position.y - self.map_info.origin.position.y) / self.map_info.resolution
        )

        reachable  = self._get_reachable_pixels(robot_col, robot_row)
        cell_size  = int(WAYPOINT_GRID_M / self.map_info.resolution)
        clearance  = int(COSTMAP_CLEARANCE_M / self.map_info.resolution)
        h, w       = int(self.map_info.height), int(self.map_info.width)

        waypoints = []

        for i in range(0, h, cell_size):
            for j in range(0, w, cell_size):
                # Check every pixel in the cell against the clearance mask
                cell_safe = True
                for px in range(i, min(i + cell_size, h)):
                    for py in range(j, min(j + cell_size, w)):
                        if not self._is_free(px, py, clearance):
                            cell_safe = False
                            break
                    if not cell_safe:
                        break

                if cell_safe:
                    
                    # save the center of the cell
                    ci, cj = i + cell_size // 2, j + cell_size // 2
                    if (ci, cj) in reachable:
                        waypoints.append(self._map_to_world(ci, cj))

        waypoints = self._filter_waypoints(waypoints)
        self.get_logger().info(f'Generated {len(waypoints)} exploration waypoints.')
        return waypoints

    def _get_reachable_pixels(self, px: int, py: int) -> set:
        """Get all reachable pixels from the robot's current grid cell."""
        h, w = int(self.map_info.height), int(self.map_info.width)
        
        if not (0 <= py < h and 0 <= px < w):
            self.get_logger().error('Robot position is outside the map!')
            return set()

        # BFS algorithm
        visited = {(py, px)}
        queue   = deque([(py, px)])

        while queue:
            row, col = queue.popleft()
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = row + dr, col + dc
                if (0 <= nr < h and 0 <= nc < w
                        and (nr, nc) not in visited
                        and self._is_free(nr, nc)):
                    visited.add((nr, nc))
                    queue.append((nr, nc))

        return visited

    def _is_free(self, row: int, col: int, clearance: int = 0) -> bool:
        h, w = int(self.map_info.height), int(self.map_info.width)
        for dr in range(-clearance, clearance + 1):
            for dc in range(-clearance, clearance + 1):
                nr, nc = row + dr, col + dc
                if 0 <= nr < h and 0 <= nc < w:
                    if self.map_data[nr * w + nc] != 0:
                        return False
        return True

    def _map_to_world(self, row: int, col: int):
        x = col * self.map_info.resolution + self.map_info.origin.position.x
        y = row * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

    def _filter_waypoints(self, waypoints, min_dist=WAYPOINT_MIN_DIST_M):
        kept_wp = []
        for i in range(len(waypoints)):
            wp = waypoints[i]
            
            too_close = False
            for k in kept_wp:
                if (wp[0] - k[0]) ** 2 + (wp[1] - k[1]) ** 2 < min_dist ** 2:
                    too_close = True
                    break
            if not too_close:
                kept_wp.append(wp)
                
        return kept_wp

    def _make_pose_stamped(self, x: float, y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0   # identity orientation
        return pose

    
    # ----------- Main state machine -------------------

    def run(self) -> None:
        self.waitUntilNav2Active()

        wp_i = 0
        waypoints = []
        self.state = STATE_INIT_LOCALIZATION

        self.get_logger().info('State machine starting.')

        # State machine loop
        while rclpy.ok() and self.state != STATE_DONE:

            if self.state == STATE_INIT_LOCALIZATION:
                self.get_logger().info('State: INIT_LOCALIZATION')

                if not self._run_localization():
                    self.get_logger().fatal(
                        'Localization phase failed. Cannot explore safely. Aborting.'
                    )
                    return

                # One-time setup that requires a localised pose
                self.lower_head(tilt=-0.4)
                self._wait_for_map_and_pose()

                waypoints = self._build_waypoints()
                if not waypoints:
                    self.get_logger().fatal(
                        'No waypoints could be generated. Aborting.'
                    )
                    return

                self.get_logger().info('Localization complete. Starting exploration.')
                self.state = STATE_EXPLORE

            elif self.state == STATE_EXPLORE:
                
                if wp_i >= len(waypoints):
                    self.get_logger().warn(
                        f'All waypoints exhausted. Found markers: {list(self.found_markers.keys())}. Restarting from the beginning.'
                    )
                    wp_i = 0    # re-scan the map from the start

                goal = self._make_pose_stamped(*waypoints[wp_i])
                accepted = self.go_to_pose(goal)

                if not accepted:
                    wp_i += 1
                    continue

                nav_result = self._wait_for_nav(ignore_detection=False)

                if nav_result == 'MARKER_DETECTED':
                    self.state = STATE_PROCESS_TARGET
                    # we will finish the scan of the area later
                else:
                    wp_i += 1
                    # Re-enable detection for the next navigation leg
                    self.detection_active = True

            
            elif self.state == STATE_PROCESS_TARGET:

                # Stop the robot if it is still moving
                if not self.is_nav_complete():
                    self._cancel_nav()

                # Consume the detection event
                marker_id    = self.detected_marker_id
                marker_pose  = self.detected_marker_pose
                self.detected_marker_id   = None
                self.detected_marker_pose = None

                self.get_logger().info(f'Processing detected marker {marker_id} ...')

                map_pose = self._get_map_pose(marker_id, marker_pose)

                if map_pose is not None:
                    self.found_markers[marker_id] = map_pose
                    self.get_logger().info(
                        f'Marker {marker_id} stored at map '
                    )
                else:
                    self.get_logger().warn(
                        f'Could not resolve marker {marker_id} to map frame. '
                    )

                if len(self.found_markers) >= 2:
                    self.state = STATE_VERIFY_NAVIGATION
                else:
                    # Resume exploration, re-enable ArUco detection
                    self.detection_active = True
                    self.state = STATE_EXPLORE

        
            elif self.state == STATE_VERIFY_NAVIGATION:

                self.get_logger().info(
                    '  Both markers found! Starting verification.\n'
                    f'  PICK  (ID {PICK_MARKER_ID})  → '
                    f'({self.found_markers[PICK_MARKER_ID].pose.position.x:.2f}, '
                    f'{self.found_markers[PICK_MARKER_ID].pose.position.y:.2f})\n'
                    f'  PLACE (ID {PLACE_MARKER_ID}) → '
                    f'({self.found_markers[PLACE_MARKER_ID].pose.position.x:.2f}, '
                    f'{self.found_markers[PLACE_MARKER_ID].pose.position.y:.2f})\n'
                )

                # Navigate to PICK location
                self._nav_to_marker(PICK_MARKER_ID, 'PICK')

                # Navigate to PLACE location
                self._nav_to_marker(PLACE_MARKER_ID, 'PLACE')

                self.get_logger().info('Task 2 COMPLETE! ')
                self.state = STATE_DONE

    def _nav_to_marker(self, marker_id: int, label: str):
        """Navigate to a stored marker pose and wait for completion."""
        if marker_id not in self.found_markers:
            self.get_logger().error(
                f'No pose stored for {label} marker (ID {marker_id})!'
            )
            return

        self.get_logger().info(
            f'Navigating to {label} location (marker ID {marker_id})...'
        )
        accepted = self.go_to_pose(self.found_markers[marker_id])
        if not accepted:
            self.get_logger().error(
                f'Navigation to {label} marker was rejected.'
            )
            return

        self._wait_for_nav(ignore_detection=True)

        if self.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached {label} location!')
        else:
            self.get_logger().warn(
                f'Navigation to {label} location ended with status {self.status}.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = Task2Coordinator()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
