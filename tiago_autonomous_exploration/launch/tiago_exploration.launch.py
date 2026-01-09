import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    #param_file_name = 'prova.yaml'
    #param_dir = '/home/manuel/tiago_ws/install/tiago_2dnav/share/tiago_2dnav/params/prova.yaml'
    
    rviz_config_dir = os.path.join(get_package_share_directory('tiago_navigation'), 'rviz', 'navigation.rviz')
    
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
                'map' : '~/map.yaml',
                'use_sim_time': use_sim_time,
                
                'is_public_sim': 'False',
                'rviz': 'True',
                
                'slam' : 'True',
                #'params_file': param_dir
                }.items(),
        )

    #rviz_cmd = Node(
    #    package='rviz2',
     #   executable='rviz2',
      #  name='rviz2',
       # arguments=['-d', rviz_config_dir],
       # parameters=[{'use_sim_time': use_sim_time}],
       # output='screen')
    
    explore_lite_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('explore_lite'), 'launch'),
            '/explore.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time}.items(),
    )

    ld = LaunchDescription()
    # Add the commands to the launch description
    ld.add_action(tiago_world_cmd)
    ld.add_action(navigation_cmd)
    #ld.add_action(rviz_cmd)
    ld.add_action(explore_lite_cmd)
    return ld
