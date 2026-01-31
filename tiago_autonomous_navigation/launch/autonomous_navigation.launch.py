import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import TimerAction


def generate_launch_description():    
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    
    tiago_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_exam'),
                'launch',
                'tiago_exam.launch.py'
            )
        ),
        launch_arguments={
                'world_name': 'group10',
                'moveit' : 'True',
                'use_sim_time': use_sim_time
        }.items()
    )
    
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_2dnav'),
                'launch', 
                'tiago_nav_bringup.launch.py')
        ),
        launch_arguments={
                'map_path': 'src/my_map',  
                'use_sim_time': use_sim_time,
                
                'is_public_sim': 'false',
                'rviz': 'True',
                
                'slam' : 'False'  # False = localization mode (AMCL)
            }.items()
        )
 

    tuck_arm = Node(
        package='tiago_exam',
        executable='tuck_arm.py',
        emulate_tty=True,
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Global localization node to handle random spawn positions
    global_localization = Node(
        package='tiago_autonomous_navigation',
        executable='localization',
        emulate_tty=True,
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
   
   
    ld = LaunchDescription()
    # Add the commands to the launch description
    
    ld.add_action(tiago_world_cmd)

    ld.add_action(
        TimerAction(
            period=2.5,
            actions=[navigation_cmd]
        )
    )
    
    ld.add_action(
        TimerAction(
            period=6.5,  # Tuck arm after MoveIt2 is ready
            actions=[tuck_arm]
        )
    )
    
    ld.add_action(
        TimerAction(
            period=13.0,  # Start localization after navigation and arm tucking
            actions=[global_localization]
        )
    )
    
    return ld
