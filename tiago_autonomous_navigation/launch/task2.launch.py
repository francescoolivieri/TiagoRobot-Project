"""
task2.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── Camera topic names for the Tiago head camera ──────────────────────────────
# aruco_ros subscribes to absolute /image and /camera_info (hardcoded in the
# binary), so remappings MUST use the leading slash.
CAMERA_IMAGE_TOPIC = '/head_front_camera/rgb/image_raw'
CAMERA_INFO_TOPIC  = '/head_front_camera/rgb/camera_info'
CAMERA_FRAME       = 'head_front_camera_rgb_optical_frame'


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # Simulation world 
    tiago_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_exam'),
                'launch',
                'tiago_exam.launch.py',
            )
        ),
        launch_arguments={
            'world_name':   'group10',
            'moveit':       'True',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Nav2 bringup (AMCL + planners + costmaps) 
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_2dnav'),
                'launch',
                'tiago_nav_bringup.launch.py',
            )
        ),
        launch_arguments={
            'map_path':      'src/my_map',
            'use_sim_time':  use_sim_time,
            'is_public_sim': 'false',
            'rviz':          'True',
            'slam':          'False',        # localization mode (AMCL)
        }.items(),
    )

    # Localization 
    localization_node = Node(
        package='tiago_autonomous_navigation',
        executable='localization_server',
        name='localization_server',
        emulate_tty=True,
        output='screen',
        parameters=[{'use_sim_time': True}],
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

    # aruco detector for the pick marker
    # The aruco_ros "single" node publishes private topics: ~/pose, ~/transform.
    # With name='aruco_single_26', those become /aruco_single_26/pose etc.
    # The binary subscribes to absolute /image and /camera_info, so remappings
    # must also be absolute (leading /).
    aruco_pick = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_26',
        emulate_tty=True,
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_id':          26,
            'marker_size':        0.25,           # metres
            'camera_frame':       CAMERA_FRAME,
            'marker_frame':       'aruco_marker_26',   # unique TF frame name
            'reference_frame':    '',             # output stays in camera frame;
                                                  # our service does the TF work
            'corner_refinement':  'LINES',
            'use_sim_time':       True,
        }],
        remappings=[
            ('/image',       CAMERA_IMAGE_TOPIC), # ??
            ('/camera_info', CAMERA_INFO_TOPIC), # ??
        ],
    )

    # ArUco detector for the PLACE marker (ID 238)
    aruco_place = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_238',
        emulate_tty=True,
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_id':          238,
            'marker_size':        0.25,
            'camera_frame':       CAMERA_FRAME,
            'marker_frame':       'aruco_marker_238',  # unique TF frame name
            'reference_frame':    '',
            'corner_refinement':  'LINES',
            'use_sim_time':       True,
        }],
        remappings=[
            ('/image',       CAMERA_IMAGE_TOPIC),
            ('/camera_info', CAMERA_INFO_TOPIC),
        ],
    )

    # State-machine coordinator
    task2_coordinator = Node(
        package='tiago_autonomous_navigation',
        executable='task_2_coordinator',
        name='task_2_coordinator',
        emulate_tty=True,
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    
    ld = LaunchDescription()
    # Add the commands to the launch description:
    
    ld.add_action(tiago_world_cmd)

    ld.add_action(TimerAction(period=2.5,  actions=[navigation_cmd]))

    ld.add_action(TimerAction(period=13.0, actions=[
        localization_node,
        target_pose_server_node,
        aruco_pick,
        aruco_place,
    ]))

    
    ld.add_action(TimerAction(period=20.0, actions=[task2_coordinator]))

    return ld
