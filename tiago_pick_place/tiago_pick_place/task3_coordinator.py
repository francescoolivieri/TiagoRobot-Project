import rclpy
import os

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry

from tiago_autonomous_navigation.task_2_coordinator import (
    Task2Coordinator,
)
import time
import math
from tiago_pick_place.manipulation import (
    ManipulationController,
    compute_pregrasp_pose,
    compute_grasp_pose,
    compute_lift_pose,
    compute_place_pose,
)
from tiago_task2_interfaces.srv import GetMarkerPose



PICK_MARKER_ID  = 26
PLACE_MARKER_ID = 238

PLACE_X = -6.57
PLACE_Y = -4.72

PLACE_TARGET_X = -6.57
PLACE_TARGET_Y = -4.72
PLACE_TABLE_TOP = 0.30

def str_to_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def go_to_location(nav, marker_pose, name, distance=0.0):

    pose = get_nav_pose(nav, marker_pose, distance)

    nav.get_logger().info(
        f'Going to {name}: x={pose.pose.position.x:.2f}, '
        f'y={pose.pose.position.y:.2f}, stand-off={distance:.2f} m'
    )

    if not nav.go_to_pose(pose):
        nav.get_logger().error(
            f'Could not send navigation goal to {name}.'
        )
        return False
    
    nav._wait_for_nav(ignore_detection=True)

    if nav.status == GoalStatus.STATUS_SUCCEEDED:
        nav.get_logger().info(
            f'Arrived at {name}!'
        )
        return True
    
    nav.get_logger().warn(
        f'Navigation to {name} ended with status {nav.status}.'
    )

    return False

def go_to_pick(nav):
    
    aruco_pose = nav._get_aruco_pose(PICK_MARKER_ID)
    
    if aruco_pose is not None:
        
        return go_to_location(
            nav,
            aruco_pose,
            'PICK',
        )


def go_to_place(nav):
    
    aruco_pose = nav._get_aruco_pose(PLACE_MARKER_ID)
    
    if aruco_pose is not None:
        return go_to_location(
            nav,
            aruco_pose,
            'PLACE',
        )


def search_for_cube(nav, cube_id=63, timeout=50.0):
    """
    Returns:
        PoseStamped of the cube in base_footprint if detected.
        None if timeout is reached.
    """

    detected_pose = {'pose': None}

    cube_topic = f'/cube_{cube_id}_pose'

    def cube_callback(msg: PoseStamped):
        detected_pose['pose'] = msg
    
    cube_subscription = nav.create_subscription(
        PoseStamped,
        cube_topic,
        cube_callback,
        10,
    )

    #Lower the head to find the Arucos
    nav.lower_head(tilt=-0.75)

    cmd_vel_pub = nav.create_publisher(
        Twist,
        '/cmd_vel',
        10,
    )

    nav.get_logger().info(
        f'Searching for cube {cube_id} with a small circular motion...'
    )

    start_time = nav.get_clock().now()

    while(
        rclpy.ok()
        and detected_pose['pose'] is None
        and (nav.get_clock().now() - start_time).nanoseconds / 1e9 < timeout
    ):
        cmd = Twist()

        cmd.angular.z = 0.30

        cmd_vel_pub.publish(cmd)

        rclpy.spin_once(
            nav,
            timeout_sec=0.1,
        )
    
    stop = Twist()

    for _ in range(5):
        cmd_vel_pub.publish(stop)
        rclpy.spin_once(
            nav,
            timeout_sec=0.05,
        )
    
    time.sleep(2.0)

    detected_pose['pose'] = None
    for _ in range(40):
        rclpy.spin_once(nav, timeout_sec=0.1)
        if detected_pose['pose'] is not None:
            break
    
    cube_pose = detected_pose['pose']

    if cube_pose is None:
        nav.get_logger().warn(
            f'Cube {cube_id} was not detected.'
        )

        nav.destroy_subscription(cube_subscription)

        return None
    
    p = cube_pose.pose.position

    nav.get_logger().info(
        f'Cube {cube_id} detected! '
        f'x={p.x:.3f}, '
        f'y={p.y:.3f}, '
        f'z={p.z:.3f} '
        f'in {cube_pose.header.frame_id}'
    )

    nav.destroy_subscription(cube_subscription)

    return cube_pose

def retreat(nav, distance=0.4, speed=0.1):
    """Back the base straight out of the table's inflation zone."""
    cmd_vel_pub = nav.create_publisher(Twist, '/key_vel', 10)
    odom_pose = {'pose': None}
    odom_sub = nav.create_subscription(
        Odometry,
        '/mobile_base_controller/odom',
        lambda msg: odom_pose.update(pose=msg.pose.pose),
        10,
    )

    discovery_deadline = time.monotonic() + 10.0
    while (
        (cmd_vel_pub.get_subscription_count() == 0 or odom_pose['pose'] is None)
        and time.monotonic() < discovery_deadline
    ):
        rclpy.spin_once(nav, timeout_sec=0.1)

    if odom_pose['pose'] is None:
        nav.get_logger().error('Cannot retreat: odometry is unavailable.')
        nav.destroy_subscription(odom_sub)
        nav.destroy_publisher(cmd_vel_pub)
        return False

    duration = distance / speed
    start_time = time.monotonic()
    start_x = odom_pose['pose'].position.x
    start_y = odom_pose['pose'].position.y

    cmd = Twist()
    cmd.linear.x = -abs(speed)

    while (
        rclpy.ok()
        and math.hypot(
            odom_pose['pose'].position.x - start_x,
            odom_pose['pose'].position.y - start_y,
        ) < distance
        and time.monotonic() - start_time < max(30.0, duration * 20.0)
    ):
        cmd_vel_pub.publish(cmd)
        rclpy.spin_once(nav, timeout_sec=0.05)

    stop = Twist()
    for _ in range(5):
        cmd_vel_pub.publish(stop)
        rclpy.spin_once(nav, timeout_sec=0.05)

    travelled = math.hypot(
        odom_pose['pose'].position.x - start_x,
        odom_pose['pose'].position.y - start_y,
    )
    nav.get_logger().info(f'Retreat complete: {travelled:.3f} m')
    nav.destroy_subscription(odom_sub)
    nav.destroy_publisher(cmd_vel_pub)
    return travelled >= distance

def get_nav_pose(nav, marker_pose: PoseStamped, distance=0.65):
    """Make a goal in front of the marker, facing the marker."""
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = nav.get_clock().now().to_msg()

    q = marker_pose.pose.orientation
    front_x = 2 * (q.x * q.z + q.w * q.y)
    front_y = 2 * (q.y * q.z - q.w * q.x)

    marker_x = marker_pose.pose.position.x
    marker_y = marker_pose.pose.position.y
    goal.pose.position.x = marker_x + distance * front_x
    goal.pose.position.y = marker_y + distance * front_y

    yaw = math.atan2(marker_y - goal.pose.position.y,
                     marker_x - goal.pose.position.x)

    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)

    return goal


def set_initial_pose(nav, x, y, yaw):
    initial_pose_pub = nav.create_publisher(
        PoseWithCovarianceStamped,
        '/initialpose',
        10,
    )

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.pose.orientation.w = math.cos(yaw / 2.0)

    for _ in range(10):
        pose.header.stamp = nav.get_clock().now().to_msg()
        initial_pose_pub.publish(pose)
        rclpy.spin_once(nav, timeout_sec=0.1)
        time.sleep(0.1)

    nav.destroy_publisher(initial_pose_pub)


def add_place_table_collision(manipulator):
    table_height = 0.30
    table_length = 1.0
    table_width = 0.50

    manipulator.moveit2.add_collision_box(
        id='place_table',
        size=[
            table_length,
            table_width,
            table_height,
        ],
        position=[
            PLACE_X,
            PLACE_Y,
            PLACE_TABLE_TOP - table_height / 2.0,
        ],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id='map',
    )


def place_cube_582(nav, manipulator, grasp_pose, cube_582_pose=None):
    ############# Go to PLACE table with Cube 582 #############
    if not go_to_place(nav):
        nav.get_logger().error('Could not reach PLACE.')
        return False

    nav.get_logger().info('Cube 582 reached PLACE; preparing release sequence.')
    add_place_table_collision(manipulator)
    time.sleep(1.0)

    #Place the second Cube
    place_pose = PoseStamped()
    place_pose.header.frame_id = 'base_footprint'
    place_pose.pose.position.x = 0.80
    place_pose.pose.position.y = 0.00 #15 cm beside the first
    place_pose.pose.position.z = PLACE_TABLE_TOP
    place_pose.pose.orientation = grasp_pose.pose.orientation
    release_pose = compute_place_pose(place_pose, PLACE_TABLE_TOP)

    # Go above the place position using normal MoveIt planning
    preplace_pose = compute_lift_pose(release_pose, height=0.15)

    #1. Go to the upper position and tuck
    #lift_pose_place = compute_lift_pose(release_pose, height=0.15)
    if not manipulator.move_to_pose(preplace_pose):
        nav.get_logger().error('PREPLACE FAILED')
        return False

    if not manipulator.move_to_pose(release_pose, cartesian=True):
        nav.get_logger().error('RELEASE DESCENT FAILED')
        return False

    nav.get_logger().info('Opening gripper and detaching cube 582.')
    if not manipulator.detach_cube('aruco_cube_exam_id582'):
        nav.get_logger().error('DETACH FAILED for cube 582')
        return False
    manipulator.moveit2.detach_collision_object('cube_582')
    manipulator.moveit2.add_collision_box(
        id='cube_582_placed',
        size=[0.07, 0.07, 0.07],
        position=[
            place_pose.pose.position.x,
            place_pose.pose.position.y,
            PLACE_TABLE_TOP + 0.035,
        ],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id='base_footprint',
    )
    time.sleep(0.5)
    manipulator.set_gripper(0.045)

    #1. Go to the upper position and tuck
    lift_pose_place = compute_lift_pose(release_pose, height=0.15)
    if not manipulator.move_to_pose(lift_pose_place,cartesian=True,):
        nav.get_logger().error('FINAL CARTESIAN GRASP DESCENT FAILED.')
        if cube_582_pose is not None:
            manipulator.log_pose_report('GRASP', lift_pose_place, cube_582_pose)
        return False
    manipulator.set_gripper(0.0)
    time.sleep(10.0)
    if not manipulator.tuck_arm():
        nav.get_logger().error(
            'TUCK FAILED after grasp.'
        )
        return False

    return True


def main(args=None):
    rclpy.init(args=args)

    nav = Task2Coordinator()

    nav.declare_parameter('fast_forward_582_grasped', False)
    nav.declare_parameter('fast_forward_post_pick_x', 0.87)
    nav.declare_parameter('fast_forward_post_pick_y', -3.65)
    nav.declare_parameter('fast_forward_post_pick_yaw', math.pi)

    nav.waitUntilNav2Active()

    fast_forward = (
        nav.get_parameter('fast_forward_582_grasped').value
        or str_to_bool(os.environ.get('TASK3_FAST_FORWARD_582_GRASPED', 'false'))
    )

    if fast_forward:
        x = nav.get_parameter('fast_forward_post_pick_x').value
        y = nav.get_parameter('fast_forward_post_pick_y').value
        yaw = nav.get_parameter('fast_forward_post_pick_yaw').value

        nav.get_logger().info(
            'Fast-forwarding Task 3 from cube 582 already grasped.'
        )
        set_initial_pose(nav, x, y, yaw)

        manipulator = ManipulationController()
        if not manipulator.tuck_arm():
            nav.get_logger().error('Fast-forward tuck setup failed.')
            return
        manipulator.set_gripper(0.033)
        time.sleep(2.0)
        if not manipulator.set_entity_relative_to_gripper(
            'aruco_cube_exam_id582'
        ):
            nav.get_logger().error('Fast-forward cube staging failed.')
            return
        if not manipulator.attach_cube('aruco_cube_exam_id582'):
            nav.get_logger().error('Fast-forward cube attach failed.')
            return
        nav.get_logger().info('Fast-forward cube 582 attached; navigating to PLACE.')

        cube_582_pose = PoseStamped()
        cube_582_pose.header.frame_id = 'base_footprint'
        cube_582_pose.pose.orientation.w = 1.0
        grasp_pose = compute_grasp_pose(cube_582_pose)

        if place_cube_582(nav, manipulator, grasp_pose):
            nav.get_logger().info('Fast-forward cube 582 release sequence complete.')
        rclpy.shutdown()
        return

    # Robot starts at a random position, so Task 3
    # still has to localize itself.
    if not nav._run_localization():
        nav.get_logger().error(
            'Task 3 could not localize TIAGo.'
        )

        nav.destroy_node()
        rclpy.shutdown()
        return

    nav.get_logger().info(
        'Task 3 localization complete.'
    )

    if not go_to_pick(nav):
        nav.get_logger().error('Could not reach PICK.')
        nav.destroy_node()
        rclpy.shutdown()
        return

    cube_63_pose = search_for_cube(
        nav,
        cube_id=63,
    )
    if cube_63_pose is None:
        nav.get_logger().error(
           'Cube 63 could not be found.'
        )
        return

    manipulator = ManipulationController()

    table_top = cube_63_pose.pose.position.z - 0.07
    table_height = 0.3

    manipulator.moveit2.add_collision_box(
        id='pick_table',
        size=[1.2, 0.6, table_height],
        position=[
            cube_63_pose.pose.position.x,
            cube_63_pose.pose.position.y,
            table_top - table_height / 2.0,
        ],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id=cube_63_pose.header.frame_id,
    )

    manipulator.moveit2.add_collision_box(
        id='cube_63',
        size=[0.07, 0.07, 0.07],
        position=[
            cube_63_pose.pose.position.x,
            cube_63_pose.pose.position.y,
            cube_63_pose.pose.position.z - 0.035,
        ],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id=cube_63_pose.header.frame_id,
    )
    time.sleep(1.0)

    pregrasp_pose = compute_pregrasp_pose(cube_63_pose)

    if not manipulator.move_to_pose(pregrasp_pose):
        manipulator.log_pose_report('PREGRASP-FAILED', pregrasp_pose, cube_63_pose)
        nav.get_logger().error(
            'PREGRASP FAILED — aborting grasp.'
        )
        return
    manipulator.log_pose_report('PREGRASP', pregrasp_pose, cube_63_pose)

    #Open the gripper
    manipulator.set_gripper(0.045)
    time.sleep(10)
    manipulator.log_pose_report('PREGRASP', pregrasp_pose, cube_63_pose)

    nav.get_logger().info(
        'Starting controlled final grasp approach.'
    )

    #manipulator.moveit2.allow_collisions(
    #    'cube_63',
    #    True,
    #)
    manipulator.moveit2.remove_collision_object('cube_63')
    time.sleep(10)

    grasp_pose = compute_grasp_pose(cube_63_pose)

    if not manipulator.move_to_pose(grasp_pose,cartesian=True,):
        nav.get_logger().error('FINAL CARTESIAN GRASP DESCENT FAILED.')
        manipulator.log_pose_report('GRASP', grasp_pose, cube_63_pose)
        return

    manipulator.log_pose_report('GRASP', grasp_pose, cube_63_pose)
    nav.get_logger().info(
        'Grasp depth reached. Closing gripper.'
    )

    manipulator.set_gripper(0.033)
    time.sleep(2)
    if not manipulator.attach_cube('aruco_cube_exam_id63'):
        nav.get_logger().error('ATTACH FAILED')
        return
    nav.get_logger().info(
        'Cube grasped. Lifting vertically.'
    )

    lift_pose = compute_lift_pose(grasp_pose)

    if not manipulator.move_to_pose(
        lift_pose,
        cartesian=True,
    ):
        nav.get_logger().error(
            'LIFT FAILED — cube may still be held.'
        )
        return

    if not manipulator.tuck_arm():
        nav.get_logger().error(
            'TUCK FAILED after grasp.'
        )
        return
    
    manipulator.moveit2.remove_collision_object('pick_table')
    time.sleep(10)

    retreat(nav, distance = 0.6)

    ############# Go to PLACE pose #############
    #Mental note: Put a function here
    if not go_to_place(nav):
            nav.get_logger().error('Could not reach PLACE.')
            nav.destroy_node()
            rclpy.shutdown()
            return

    table_height = 0.30
    table_length = 1.0
    table_width = 0.50

    manipulator.moveit2.add_collision_box(
        id='place_table',
        size=[
            table_length,
            table_width,
            table_height,
        ],
        position=[
            PLACE_X,
            PLACE_Y,
            PLACE_TABLE_TOP - table_height / 2.0,
        ],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id='map',
    )

    time.sleep(1.0)

    #Place the first Cube
    place_pose = PoseStamped()
    place_pose.header.frame_id = 'base_footprint'
    place_pose.pose.position.x = 0.80
    place_pose.pose.position.y = 0.15
    place_pose.pose.position.z = PLACE_TABLE_TOP
    place_pose.pose.orientation = grasp_pose.pose.orientation
    release_pose = compute_place_pose(place_pose, PLACE_TABLE_TOP)

    # Go above the place position using normal MoveIt planning
    preplace_pose = compute_lift_pose(release_pose, height=0.15)

    #1. Go to the upper position and tuck
    #lift_pose_place = compute_lift_pose(release_pose, height=0.15)
    if not manipulator.move_to_pose(preplace_pose):
        nav.get_logger().error('PREPLACE FAILED')
        return
    
    if not manipulator.move_to_pose(release_pose, cartesian=True):
        nav.get_logger().error('RELEASE DESCENT FAILED')
        return

    if not manipulator.detach_cube('aruco_cube_exam_id63'):
        nav.get_logger().error('DETACH FAILED for cube 63')
        return
    manipulator.moveit2.detach_collision_object('cube_63')
    manipulator.moveit2.add_collision_box(
        id='cube_63_placed',
        size=[0.07, 0.07, 0.07],
        position=[
            place_pose.pose.position.x,
            place_pose.pose.position.y,
            PLACE_TABLE_TOP + 0.035,
        ],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id='base_footprint',
    )
    time.sleep(0.5)
    manipulator.set_gripper(0.045)

    lift_after_place = compute_lift_pose(release_pose, height=0.15)

    if not manipulator.move_to_pose(lift_after_place, cartesian=True):
        nav.get_logger().error('LIFT AFTER PLACE FAILED')
        return

    if not manipulator.tuck_arm():
        nav.get_logger().error(
            'TUCK FAILED after grasp.'
        )
        return

    manipulator.moveit2.remove_collision_object('cube_63_placed')
    time.sleep(10)

    nav.get_logger().info("Cube 63 release sequence complete; returning for cube 582.")
    retreat(nav, distance=0.6)

    #2. Go to the Pick Position
    if not go_to_pick(nav):
        nav.get_logger().error(
            'Could not reach PICK.'
        )
        nav.destroy_node()
        rclpy.shutdown()
        return
    
    ################ Find Cube 582 ####################
    cube_582_pose = search_for_cube(
        nav,
        cube_id=582,
    )
    if cube_582_pose is None:
        nav.get_logger().error(
           'Cube 582 could not be found.'
        )
        return

    table_top = cube_582_pose.pose.position.z - 0.07
    manipulator.moveit2.add_collision_box(
        id='pick_table',
        size=[1.2, 0.6, table_height],
        position=[
            cube_582_pose.pose.position.x,
            cube_582_pose.pose.position.y,
            table_top - table_height / 2.0,
        ],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id=cube_582_pose.header.frame_id,
    )

    manipulator.moveit2.add_collision_box(
        id='cube_582',
        size=[0.07, 0.07, 0.07],
        position=[
            cube_582_pose.pose.position.x,
            cube_582_pose.pose.position.y,
            cube_582_pose.pose.position.z - 0.035,
        ],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id=cube_582_pose.header.frame_id,
    )
    time.sleep(10)

    pregrasp_pose = compute_pregrasp_pose(cube_582_pose)

    if not manipulator.move_to_pose(pregrasp_pose):
        manipulator.log_pose_report('PREGRASP-FAILED', pregrasp_pose, cube_582_pose)
        nav.get_logger().error(
            'PREGRASP FAILED — aborting grasp.'
        )
        return
    
    #manipulator.moveit2.allow_collisions(
    #    'cube_582',
    #    True,
    #)

    time.sleep(10)

    manipulator.moveit2.remove_collision_object('cube_582')
    time.sleep(10.0)

    grasp_pose = compute_grasp_pose(cube_582_pose)

    if not manipulator.move_to_pose(grasp_pose,cartesian=True,):
        nav.get_logger().error('FINAL CARTESIAN GRASP DESCENT FAILED.')
        manipulator.log_pose_report('GRASP', grasp_pose, cube_582_pose)
        return

    manipulator.log_pose_report('GRASP', grasp_pose, cube_582_pose)
    nav.get_logger().info(
        'Grasp depth reached. Closing gripper.'
    )

    manipulator.set_gripper(0.033)
    time.sleep(2)
    if not manipulator.attach_cube('aruco_cube_exam_id582'):
        nav.get_logger().error('ATTACH FAILED')
        return
    lift_pose = compute_lift_pose(grasp_pose)

    if not manipulator.move_to_pose(
        lift_pose,
        cartesian=True,
    ):
        nav.get_logger().error(
            'LIFT FAILED — cube may still be held.'
        )
        return

    if not manipulator.tuck_arm():
        nav.get_logger().error(
            'TUCK FAILED after grasp.'
        )
        return
    
    manipulator.moveit2.remove_collision_object('pick_table')
    time.sleep(10)

    retreat(nav, distance=0.6)

    if not place_cube_582(nav, manipulator, grasp_pose, cube_582_pose):
        return
    
    nav.get_logger().info("Both cube release sequences complete: 63 then 582. Verify final table poses.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
