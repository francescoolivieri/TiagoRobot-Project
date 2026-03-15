import rclpy
import rclpy.duration
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  
from tf2_ros import TransformException

from tiago_task2_interfaces.srv import GetMarkerPose


CAMERA_FRAME_FALLBACK = 'head_front_camera_rgb_optical_frame'
MAP_FRAME = 'map'



class TargetPoseServer(Node):
    """
    Service server that receives an aruco marker pose in the camera frame
    and does two things: 
    1. returns it transformed into the map frame via tf2.
    2. stores the transformed pose internally for later retrieval.
    """

    def __init__(self):
        super().__init__('target_pose_server')

        # tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal store: marker_id (int) -> PoseStamped in map frame
        self._stored_poses: dict = {}

        self.srv = self.create_service(
            GetMarkerPose,
            'get_marker_pose',
            self._handle_request,
        )

        self.get_logger().info('TargetPoseServer ready on service "get_marker_pose".')


    def _handle_request(self, request, response):
        marker_id = request.marker_id
        pose_camera = request.pose_in_camera_frame

        # Use the frame embedded in the message; fall back to known frame name
        source_frame = (
            pose_camera.header.frame_id
            if pose_camera.header.frame_id
            else CAMERA_FRAME_FALLBACK
        )
        pose_camera.header.frame_id = source_frame

        self.get_logger().info(
            f'Transforming marker {marker_id}: {source_frame} to {MAP_FRAME}'
        )

        try:
            # Look up the transform at the exact time the pose was captured
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                source_frame,
                pose_camera.header.stamp,
                timeout=rclpy.duration.Duration(seconds=4.0),
            )
            
            # Transform the pose from the source frame to the map frame
            pose_map = tf2_geometry_msgs.do_transform_pose_stamped(
                pose_camera, transform
            )
            pose_map.header.frame_id = MAP_FRAME

            self._stored_poses[marker_id] = pose_map

            response.success = True
            response.message = (
                f'Marker {marker_id} transformed successfully to {MAP_FRAME}.'
            )
            response.pose_in_map_frame = pose_map


        except TransformException as exc:
            response.success = False
            response.message = f'TF lookup error: {exc}'
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


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
