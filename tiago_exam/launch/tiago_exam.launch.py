# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from os import environ, pathsep

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_pal.include_utils import include_launch_py_description

from launch_ros.actions import Node


def get_model_paths(packages_names):
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ''
    for package_name in packages_names:
        if resource_paths != '':
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    return resource_paths


def generate_launch_description():

    world_name_arg = DeclareLaunchArgument("world_name", default_value="1")

    moveit_arg = DeclareLaunchArgument(
        'moveit', default_value='true',
        description='Specify if launching MoveIt 2'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tiago_exam_worlds'),
            'launch'), '/pal_gazebo_exam.launch.py']),
            launch_arguments={'world_name': LaunchConfiguration("world_name")}.items()
    )

    tiago_spawn = include_launch_py_description(
        'tiago_exam', ['launch', 'tiago_spawn.launch.py'],
        launch_arguments={'use_sim_time': 'True'}.items())

    twist_mux = include_launch_py_description(
        'tiago_bringup', ['launch', 'twist_mux.launch.py'])

    robot_state_publisher = include_launch_py_description(
        'tiago_description', ['launch', 'robot_state_publisher.launch.py'],
        launch_arguments={'use_sim_time': 'True'}.items())

    controller_config_dir = os.path.join(
        get_package_share_directory('tiago_controller_configuration'),
        'config',
    )
    controller_specs = (
        ('joint_state_broadcaster',
         'joint_state_broadcaster/JointStateBroadcaster',
         'joint_state_broadcaster.yaml'),
        ('mobile_base_controller',
         'diff_drive_controller/DiffDriveController',
         'mobile_base_controller.yaml'),
        ('torso_controller',
         'joint_trajectory_controller/JointTrajectoryController',
         'torso_controller.yaml'),
        ('head_controller',
         'joint_trajectory_controller/JointTrajectoryController',
         'head_controller.yaml'),
        ('arm_controller',
         'joint_trajectory_controller/JointTrajectoryController',
         'arm_controller.yaml'),
        ('ft_sensor_controller',
         'force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster',
         'ft_sensor_controller.yaml'),
    )
    controller_launches = [
        Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawner_{name}',
            arguments=[
                name,
                '--controller-manager', '/controller_manager',
                '--controller-type', controller_type,
                '--param-file', os.path.join(controller_config_dir, config_file),
            ],
            output='screen',
        )
        for name, controller_type, config_file in controller_specs
    ]
    controller_launches.append(include_launch_py_description(
        'pal_gripper_controller_configuration',
        ['launch', 'pal_gripper_controller.launch.py']))


    move_group = include_launch_py_description(
        'tiago_moveit_config', ['launch', 'move_group.launch.py'],
        launch_arguments={'use_sim_time': 'True'}.items(),
        condition=IfCondition(LaunchConfiguration('moveit')))

    tuck_arm = Node(package='tiago_exam',
                    executable='tuck_arm.py',
                    emulate_tty=True,
                    output='both',
                    parameters=[{'use_sim_time': True}])


    packages = ['tiago_description', 'pmb2_description',
                'pal_hey5_description', 'pal_gripper_description',
                'pal_robotiq_description']
    model_path = get_model_paths(packages)
    resource_path = get_resource_paths(packages)
    plugin_path = os.path.join(
        get_package_prefix('ros2_linkattacher'), 'lib'
    )

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep + environ['GAZEBO_MODEL_PATH']

    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GAZEBO_RESOURCE_PATH']

    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin_path += pathsep + environ['GAZEBO_PLUGIN_PATH']

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))
    ld.add_action(SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path))
    ld.add_action(world_name_arg)

    ld.add_action(gazebo)
    # robot_description must exist before spawn_entity can insert TIAGo.
    ld.add_action(robot_state_publisher)
    ld.add_action(twist_mux)
    ld.add_action(tiago_spawn)

    # The controller manager serializes load/configure calls poorly under the
    # simulation startup load. Start one controller at a time after spawn.
    for index, controller_launch in enumerate(controller_launches):
        ld.add_action(TimerAction(
            period=5.0 + index * 1.5,
            actions=[controller_launch],
        ))

    ld.add_action(moveit_arg)
    ld.add_action(TimerAction(period=18.0, actions=[move_group]))
    ld.add_action(TimerAction(period=24.0, actions=[tuck_arm]))

    return ld
