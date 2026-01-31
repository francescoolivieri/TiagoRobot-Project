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
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tiago_exam'),
            'launch'),
            '/tiago_exam.launch.py']),
        launch_arguments={
        
            'world_name': 'group10',
            
            'moveit' : 'True',
            
            'use_sim_time': use_sim_time
        }.items()
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('tiago_2dnav'),
                'launch'), '/tiago_nav_bringup.launch.py']),
            launch_arguments={
                'map_path': '/map.yaml',  
                'use_sim_time': use_sim_time,
                
                'is_public_sim': 'false',
                'rviz': 'True',
                
                'slam' : 'True'
                }.items(),
        )
 
        
    tuck_arm = Node(
        package='tiago_exam',
        executable='tuck_arm.py',
        emulate_tty=True,
        output='screen',
        parameters=[{'use_sim_time': True}]
        )
   
    explore_lite_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('explore_lite'), 'launch'),
            '/explore.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time}.items(),
    )
    
    
    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        arguments=['-f', 'group10_map'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld = LaunchDescription()
    # Add the commands to the launch description
    
    ld.add_action(tiago_world_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(
        TimerAction(
            period=12.5,  # Increased from 10.0 to ensure MoveIt2 is fully ready
            actions=[tuck_arm]
        )
    )
    ld.add_action(
        TimerAction(
            period=17.5,
            actions=[explore_lite_cmd]
        )
    )
    
    ld.add_action(
        TimerAction(
            period=300.0,   # scegli un tempo realistico
            actions=[map_saver]
        )
    )
    return ld
