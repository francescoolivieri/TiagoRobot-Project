import time
import rclpy
import os
from threading import Thread

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from linkattacher_msgs.srv import AttachLink, DetachLink

import math
from scipy.spatial.transform import Rotation
import numpy as np

import rclpy.time
import rclpy.duration
from tf2_ros import Buffer, TransformListener

# ----- TIAGo MoveIt configuration -----

JOINT_NAMES = [
    "torso_lift_joint",
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
]

BASE_LINK_NAME = "base_footprint"
END_EFFECTOR_NAME = "gripper_grasping_frame"
GROUP_NAME = "arm_torso"
TUCKED_JOINT_POSITIONS = [0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]

# Distance between the gripper and cube
# before the final grasping movement.
PREGRASP_DISTANCE = 0.16
GRASP_DISTANCE = 0.045
LIFT_DISTANCE = 0.20

#Place the cube
CUBE_SIZE = 0.07
RELEASE_CLEARENCE = 0.05

def compute_grasp_orientation(cube_pose: PoseStamped):
    """Top-down orientation whose finger axis matches the cube's yaw."""
    q = cube_pose.pose.orientation
    R_marker = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    # The marker's local x axis, flattened onto the horizontal plane.
    ref = R_marker[:, 0].copy()
    ref[2] = 0.0
    if np.linalg.norm(ref) < 1e-3:          # marker seen almost edge-on
        ref = R_marker[:, 1].copy()
        ref[2] = 0.0
    ref /= np.linalg.norm(ref)

    approach = np.array([0.0, 0.0, -1.0])   # straight down
    third = np.cross(approach, ref)

    R = np.column_stack([approach, ref, third])
    return Rotation.from_matrix(R).as_quat()

def compute_pregrasp_pose(cube_pose: PoseStamped) -> PoseStamped:
    pregrasp_pose = PoseStamped()
    pregrasp_pose.header = cube_pose.header

    pregrasp_pose.pose.position.x = cube_pose.pose.position.x
    pregrasp_pose.pose.position.y = cube_pose.pose.position.y
    pregrasp_pose.pose.position.z = (
        cube_pose.pose.position.z + PREGRASP_DISTANCE
    )

    qx, qy, qz, qw = compute_grasp_orientation(cube_pose)
    pregrasp_pose.pose.orientation.x = float(qx)
    pregrasp_pose.pose.orientation.y = float(qy)
    pregrasp_pose.pose.orientation.z = float(qz)
    pregrasp_pose.pose.orientation.w = float(qw)

    return pregrasp_pose

def compute_grasp_pose(cube_pose: PoseStamped) -> PoseStamped:
    grasp_pose = PoseStamped()
    grasp_pose.header = cube_pose.header

    grasp_pose.pose.position.x = cube_pose.pose.position.x
    grasp_pose.pose.position.y = cube_pose.pose.position.y
    grasp_pose.pose.position.z = (
        cube_pose.pose.position.z + GRASP_DISTANCE
    )

    qx, qy, qz, qw = compute_grasp_orientation(cube_pose)

    grasp_pose.pose.orientation.x = float(qx)
    grasp_pose.pose.orientation.y = float(qy)
    grasp_pose.pose.orientation.z = float(qz)
    grasp_pose.pose.orientation.w = float(qw)

    return grasp_pose


def compute_lift_pose(grasp_pose: PoseStamped, height=LIFT_DISTANCE) -> PoseStamped:
    lift_pose = PoseStamped()
    lift_pose.header = grasp_pose.header
    lift_pose.pose.position.x = grasp_pose.pose.position.x
    lift_pose.pose.position.y = grasp_pose.pose.position.y
    lift_pose.pose.position.z = grasp_pose.pose.position.z + height
    lift_pose.pose.orientation = grasp_pose.pose.orientation
    return lift_pose

def compute_place_pose(place_pose: PoseStamped, table_top_z: float) -> PoseStamped:
    release_pose = PoseStamped()

    release_pose.header = place_pose.header

    release_pose.pose.position.x = place_pose.pose.position.x
    release_pose.pose.position.y = place_pose.pose.position.y
    release_pose.pose.position.z = (
        table_top_z + CUBE_SIZE + RELEASE_CLEARENCE
    )

    release_pose.pose.orientation = place_pose.pose.orientation

    return release_pose

class ManipulationController(Node):
    def __init__(self):

        super().__init__(
            'manipulation_controller'
        )

        callback_group = (
            ReentrantCallbackGroup()
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK_NAME,
            end_effector_name=END_EFFECTOR_NAME,
            group_name=GROUP_NAME,
            callback_group=callback_group,
        )

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10,
        )
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        # MoveIt2 uses subscriptions, services, and actions whose callbacks
        # must be processed while the task sequence is running.
        self._executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._executor_thread = Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

        deadline = time.monotonic() + 30.0
        while self.moveit2.joint_state is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if self.moveit2.joint_state is None:
            raise RuntimeError('Timed out waiting for MoveIt joint states')

    def log_pose_report(self, label, target: PoseStamped, cube_pose: PoseStamped = None):
        """Log where the gripper actually is vs where it was told to go."""
        frame = target.header.frame_id

        try:
            tf = self.tf_buffer.lookup_transform(
                frame,
                END_EFFECTOR_NAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except Exception as exc:
            self.get_logger().warn(f'[{label}] TF lookup failed: {exc}')
            return

        g = tf.transform.translation
        t = target.pose.position

        lines = [
            f'[{label}] frame = {frame}',
            f'  commanded : x={t.x:+.3f} y={t.y:+.3f} z={t.z:+.3f}',
            f'  gripper   : x={g.x:+.3f} y={g.y:+.3f} z={g.z:+.3f}',
            f'  error     : dx={g.x-t.x:+.3f} dy={g.y-t.y:+.3f} dz={g.z-t.z:+.3f}',
        ]

        if cube_pose is not None:
            c = cube_pose.pose.position
            lateral = math.hypot(g.x - c.x, g.y - c.y)
            lines += [
                f'  cube      : x={c.x:+.3f} y={c.y:+.3f} z={c.z:+.3f}',
                f'  vs cube   : dx={g.x-c.x:+.3f} dy={g.y-c.y:+.3f} '
                f'dz={g.z-c.z:+.3f} lateral={lateral:.3f}',
            ]

        self.get_logger().info('\n'.join(lines))
    def add_table(self, x, y, z, size=(1.2, 0.6, 0.8), frame_id='map'):
        self.moveit2.add_collision_box(
            id='pick_table',
            size=list(size),
            position=[x, y, z],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            frame_id=frame_id,
        )
        time.sleep(1.0)

    def set_gripper(self, position, seconds=2):
        traj = JointTrajectory()
        traj.joint_names = [
            'gripper_left_finger_joint',
            'gripper_right_finger_joint',
        ]
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = seconds
        traj.points = [point]
        self.gripper_pub.publish(traj)

    def set_entity_relative_to_gripper(
        self,
        model_name,
        reference_frame='tiago::wrist_ft_link',
    ):
        delete_client = self.create_client(DeleteEntity, '/delete_entity')
        spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        if not delete_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/delete_entity unavailable')
            return False
        if not spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/spawn_entity unavailable')
            return False

        delete_req = DeleteEntity.Request()
        delete_req.name = model_name
        delete_future = delete_client.call_async(delete_req)
        rclpy.spin_until_future_complete(self, delete_future, timeout_sec=10.0)

        model_dir = get_package_share_directory('tiago_exam_worlds')
        sdf_path = os.path.join(
            model_dir,
            'models',
            model_name,
            f'{model_name}.sdf',
        )
        with open(sdf_path, 'r') as sdf_file:
            model_xml = sdf_file.read()

        spawn_req = SpawnEntity.Request()
        spawn_req.name = model_name
        spawn_req.xml = model_xml
        spawn_req.reference_frame = reference_frame
        spawn_req.initial_pose.position.x = 0.18
        spawn_req.initial_pose.orientation.w = 1.0

        future = spawn_client.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'Spawning {model_name} timed out')
            return False
        if not future.result().success:
            self.get_logger().error(
                f'Spawning {model_name} failed: {future.result().status_message}'
            )
            return False
        self.get_logger().info(
            f'Spawned {model_name} relative to {reference_frame}.'
        )
        return True

    def move_to_pose(self, pose: PoseStamped, cartesian=False):
        
        self.get_logger().info(
            'Moving arm to pre-grasp pose...'
        )

        p = pose.pose.position
        q = pose.pose.orientation

        self.get_logger().warn(
            f'MOVEIT TARGET: frame={pose.header.frame_id} '
            f'x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f} | '
            f'q=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})'
        )

        self.moveit2.move_to_pose(
            position=[
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ],

            quat_xyzw=[
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],

            frame_id=pose.header.frame_id,

            cartesian=cartesian,

            # Review: 2.5 mm Cartesian interpolation
            cartesian_max_step=0.0025,

            # Don't accept a partially completed Cartesian path
            cartesian_fraction_threshold=0.99,
        )

        ok = self.moveit2.wait_until_executed()
        if ok:
            self.get_logger().info('Movement finished.')
        else:
            self.get_logger().error('Movement FAILED.')
        return ok

    def tuck_arm(self) -> bool:
        
        self.get_logger().info('Tucking arm...')
        for attempt in range(3):
            self.moveit2.move_to_configuration(TUCKED_JOINT_POSITIONS)
            if self.moveit2.wait_until_executed():
                return True
            self.get_logger().warn(f'Tuck attempt {attempt + 1}/3 failed; replanning.')
            time.sleep(0.5)
        self.get_logger().error('Tuck FAILED after 3 attempts')
        return False

    def _link_call(self, srv_type, srv_name, request, timeout=10.0):
        node = rclpy.create_node('link_attacher_client')
        try:
            client = node.create_client(srv_type, srv_name)
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f'{srv_name} unavailable')
                return False
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done():
                self.get_logger().error(f'{srv_name} timed out')
                return False
            result = future.result()
            if not result.success:
                self.get_logger().error(f'{srv_name} refused: {result.message}')
                return False
            return True
        finally:
            node.destroy_node()

    def attach_cube(self, cube_model, robot='tiago', ee_link='wrist_ft_link'):
        req = AttachLink.Request()
        req.model1_name, req.link1_name = robot, ee_link
        req.model2_name, req.link2_name = cube_model, 'link'
        return self._link_call(AttachLink, '/ATTACHLINK', req)

    def detach_cube(self, cube_model, robot='tiago', ee_link='wrist_ft_link'):
        req = DetachLink.Request()
        req.model1_name, req.link1_name = robot, ee_link
        req.model2_name, req.link2_name = cube_model, 'link'
        return self._link_call(DetachLink, '/DETACHLINK', req)
