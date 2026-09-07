import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  
from tf2_ros import TransformException

from tiago_task2_interfaces.srv import GetMarkerPose


MAP_FRAME = 'map'
POSES_FILE = os.path.join(
    get_package_share_directory('tiago_autonomous_navigation'),
    'database_server',
    'marker_poses.yaml',
)

class TargetPoseServer(Node):
    """
    Service server that receives an aruco marker pose in the camera frame
    and does two things: 
    1. returns it transformed into the map frame via tf2.
    2. stores the transformed pose in a YAML file for later retrieval.
    """

    def __init__(self):
        super().__init__('target_pose_server')

        # tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.srv = self.create_service(
            GetMarkerPose,
            'get_marker_pose',
            self._handle_request,
        )

        self.get_logger().info('TargetPoseServer ready on service "get_marker_pose".')


    def _handle_request(self, request, response):
        marker_id = request.marker_id
        pose_camera = request.pose_in_camera_frame

        # An empty PoseStamped means: return the saved pose for this ID.
        if not pose_camera.header.frame_id:
            return self._get_saved_pose(marker_id, response)

        source_frame = pose_camera.header.frame_id

        self.get_logger().info(
            f'Transforming marker {marker_id}: {source_frame} to {MAP_FRAME}'
        )

        try:
            # The coordinator sends this pose after the robot has stopped, so
            # the latest available transform can be used safely.
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=4.0),
            )
            
            # Transform the pose from the source frame to the map frame
            pose_map = tf2_geometry_msgs.do_transform_pose_stamped(
                pose_camera, transform
            )
            pose_map.header.frame_id = MAP_FRAME

            self._save_pose(marker_id, pose_map)

            response.success = True
            response.message = (
                f'Marker {marker_id} transformed successfully to {MAP_FRAME}.'
            )
            response.pose_in_map_frame = pose_map


        except TransformException as exc:
            response.success = False
            response.message = f'TF lookup error: {exc}'
            self.get_logger().error(response.message)

        except Exception as exc:
            response.success = False
            response.message = f'Could not save marker pose: {exc}'
            self.get_logger().error(response.message)

        # except tf2_ros.ConnectivityException as exc:
        #     response.success = False
        #     response.message = f'TF connectivity error: {exc}'
        #     self.get_logger().error(response.message)

        # except tf2_ros.ExtrapolationException as exc:
        #     # Fallback: try with the latest available transform (time = 0)
        #     self.get_logger().warn(
        #         f'Extrapolation error for marker {marker_id}: {exc}. '
        #         'Retrying with latest available transform.'
        #     )
        #     try:
        #         transform = self.tf_buffer.lookup_transform(
        #             MAP_FRAME,
        #             source_frame,
        #             rclpy.time.Time(),
        #             timeout=rclpy.duration.Duration(seconds=2.0),
        #         )
        #         pose_map = tf2_geometry_msgs.do_transform_pose_stamped(
        #             pose_camera, transform
        #         )
        #         pose_map.header.frame_id = MAP_FRAME

        #         self._stored_poses[marker_id] = pose_map
        #         response.success = True
        #         response.message = (
        #             f'Marker {marker_id} transformed with latest TF.'
        #         )
        #         response.pose_in_map_frame = pose_map
        #         self.get_logger().info(
        #             f'[Marker {marker_id}] map position (latest TF) → '
        #             f'x={pose_map.pose.position.x:.3f}, '
        #             f'y={pose_map.pose.position.y:.3f}'
        #         )
        #     except Exception as exc2:
        #         response.success = False
        #         response.message = f'TF retry also failed: {exc2}'
        #         self.get_logger().error(response.message)

        return response

    def _read_yaml(self):
        if not os.path.exists(POSES_FILE):
            return {}

        with open(POSES_FILE, 'r') as file:
            return yaml.safe_load(file) or {}

    def _save_pose(self, marker_id, pose):
        poses = self._read_yaml()
        poses[str(marker_id)] = {
            'name': f'marker_{marker_id}',
            'position': {
                'x': float(pose.pose.position.x),
                'y': float(pose.pose.position.y),
                'z': float(pose.pose.position.z),
            },
            'orientation': {
                'x': float(pose.pose.orientation.x),
                'y': float(pose.pose.orientation.y),
                'z': float(pose.pose.orientation.z),
                'w': float(pose.pose.orientation.w),
            },
        }

        folder = os.path.dirname(POSES_FILE)
        if not os.path.exists(folder):
            os.makedirs(folder)

        with open(POSES_FILE, 'w') as file:
            yaml.safe_dump(poses, file)

    def _get_saved_pose(self, marker_id, response):
        try:
            saved = self._read_yaml().get(str(marker_id))
        except Exception as exc:
            response.success = False
            response.message = f'Could not read saved poses: {exc}'
            return response

        if saved is None:
            response.success = False
            response.message = f'Marker {marker_id} is not saved.'
            return response

        pose = response.pose_in_map_frame
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = saved['position']['x']
        pose.pose.position.y = saved['position']['y']
        pose.pose.position.z = saved['position']['z']
        pose.pose.orientation.x = saved['orientation']['x']
        pose.pose.orientation.y = saved['orientation']['y']
        pose.pose.orientation.z = saved['orientation']['z']
        pose.pose.orientation.w = saved['orientation']['w']

        response.success = True
        response.message = f'Marker {marker_id} loaded from YAML.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
