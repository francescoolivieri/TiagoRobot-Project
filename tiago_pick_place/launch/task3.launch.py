import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


CAMERA_IMAGE_TOPIC = '/head_front_camera/rgb/image_raw'
CAMERA_INFO_TOPIC = '/head_front_camera/rgb/camera_info'

CAMERA_FRAME = 'head_front_camera_rgb_optical_frame'


def generate_launch_description():
    # ----------------------------------------------------------
    # 1. Start the TIAGo simulation in our exam world
    # ----------------------------------------------------------

    tiago_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_exam'),
                'launch',
                'tiago_exam.launch.py',
            )
        ),
        launch_arguments={
            'world_name': 'group10',
            'moveit': 'True',
            'use_sim_time': 'True',
        }.items(),
    )

    # ----------------------------------------------------------
    # Navigation stack: known map + AMCL + Nav2 + RViz
    # Adapted directly from our working Task 2
    # ----------------------------------------------------------

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_2dnav'),
                'launch',
                'tiago_nav_bringup.launch.py',
            )
        ),
        launch_arguments={
            # Task 2 and Task 3 must navigate in the same map frame.
            'map_path': '/home/francesco/tiago_ws/src/my_map',
            'use_sim_time': 'True',
            'is_public_sim': 'false',
            'rviz': 'True',
            'slam': 'False',
        }.items(),
    )

    localization = Node(
        package='tiago_autonomous_navigation',
        executable='localization_server',
        name='localization_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )
    
    # Target pose server (camera frame to map frame) 
    target_pose_server_node = Node(
        package='tiago_autonomous_navigation',
        executable='target_pose_server',
        name='target_pose_server',
        emulate_tty=True,
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ----------------------------------------------------------
    # 2. ArUco detector for cube 63
    # ----------------------------------------------------------

    aruco_63 = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_63',

        output='screen',

        parameters=[{
            'image_is_rectified': True,
            'marker_id': 63,

            # Cube marker is 7 cm × 7 cm
            'marker_size': 0.07,

            'camera_frame': CAMERA_FRAME,
            'marker_frame': 'aruco_marker_63',

            # Leave pose expressed in camera frame for now.
            # cube_tracker will later perform the TF transform.
            'reference_frame': '',

            'corner_refinement': 'LINES',
            'use_sim_time': True,
        }],

        remappings=[
            ('/image', CAMERA_IMAGE_TOPIC),
            ('/camera_info', CAMERA_INFO_TOPIC),
        ],
    )

    # ----------------------------------------------------------
    # 3. ArUco detector for cube 582
    # ----------------------------------------------------------

    aruco_582 = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_582',

        output='screen',

        parameters=[{
            'image_is_rectified': True,
            'marker_id': 582,
            'marker_size': 0.07,

            'camera_frame': CAMERA_FRAME,
            'marker_frame': 'aruco_marker_582',

            'reference_frame': '',
            'corner_refinement': 'LINES',
            'use_sim_time': True,
        }],

        remappings=[
            ('/image', CAMERA_IMAGE_TOPIC),
            ('/camera_info', CAMERA_INFO_TOPIC),
        ],
    )

    cube_tracker = Node(
        package='tiago_pick_place',
        executable='cube_tracker',
        name='cube_tracker',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    camera_tf_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_alias',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'head_front_camera_rgb_optical_frame',
            'head_front_camera_color_optical_frame',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    task3_coordinator = Node(
        package='tiago_pick_place',
        executable='task3_coordinator',
        name='task3_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    ld = LaunchDescription()

    # Start Gazebo/TIAGo immediately
    ld.add_action(tiago_world)

    # Camera-frame alias can also start immediately
    ld.add_action(camera_tf_alias)

    # Give Gazebo some time, then start Nav2 + RViz
    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[navigation],
        )
    )

    # After Nav2 has had time to initialise:
    # localization + all perception nodes
    ld.add_action(
        TimerAction(
            period=17.0,
            actions=[
                localization,
                target_pose_server_node,
                aruco_63,
                aruco_582,
                cube_tracker,
            ],
        )
    )

    ld.add_action(
        TimerAction(
            period=25.0,
            actions=[
                task3_coordinator,
            ],
        )
    )

    return ld
