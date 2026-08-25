#!/usr/bin/env python3

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

import time
from threading import Thread

from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup

import rclpy
from rclpy.node import Node

# Tiago Parameters
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
END_EFFECTOR_NAME = "arm_tool_link"
GROUP_NAME = "arm_torso"

# Home/tucked position for the arm (final waypoint from home motion)
HOME_JOINT_POSITIONS = [0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]


class ArmTucker(Node):

    def __init__(self):
        super().__init__('arm_tucker')
        self._is_successful = None
        
        # Create callback group that allows parallel execution of callbacks
        callback_group = ReentrantCallbackGroup()
        
        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK_NAME,
            end_effector_name=END_EFFECTOR_NAME,
            group_name=GROUP_NAME,
            callback_group=callback_group,
        )
        
        # Set velocity and acceleration scaling
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        
        # Start executor in background
        self._tuck_executor = rclpy.executors.MultiThreadedExecutor(2)
        self._tuck_executor.add_node(self)
        self._executor_thread = Thread(
            target=self._tuck_executor.spin,
            daemon=True,
        )
        self._executor_thread.start()
        
        self.get_logger().info('Arm tucker initialized, waiting for MoveIt2...')

    def wait_for_moveit(self, timeout=30.0):
        """Wait for MoveIt2 to be ready"""
        start_time = time.time()
        rate = self.create_rate(2.0)
        
        while time.time() - start_time < timeout:
            try:
                # Try to get current joint positions as a readiness check
                if self.moveit2.joint_state is not None:
                    self.get_logger().info('MoveIt2 is ready!')
                    return True
            except Exception as e:
                self.get_logger().debug(f'Waiting for MoveIt2: {e}')
            
            rate.sleep()
        
        self.get_logger().error('Timeout waiting for MoveIt2')
        return False

    def is_successful(self):
        return self._is_successful

    def close(self):
        self._tuck_executor.shutdown()
        self._executor_thread.join(timeout=2.0)
        self.destroy_node()

    def _is_configuration_reached(self, target_positions, tolerance=0.12, timeout=5.0):
        """Verify that joints reached target within tolerance."""
        start_time = time.time()

        while time.time() - start_time < timeout:
            joint_state = self.moveit2.joint_state
            if joint_state is None:
                time.sleep(0.1)
                continue

            joint_map = dict(zip(joint_state.name, joint_state.position))
            max_error = 0.0

            for joint_name, target in zip(JOINT_NAMES, target_positions):
                current = joint_map.get(joint_name, None)
                if current is None:
                    max_error = float('inf')
                    break
                error = abs(current - target)
                if error > max_error:
                    max_error = error

            if max_error <= tolerance:
                return True

            time.sleep(0.1)

        return False

    def tuck_arm(self):
        """Move the arm to the home/tucked position"""
        try:
            self.get_logger().info('Moving arm to tucked position...')
            self.moveit2.move_to_configuration(HOME_JOINT_POSITIONS)
            self.moveit2.wait_until_executed()
            self._is_successful = self._is_configuration_reached(
                HOME_JOINT_POSITIONS,
                tolerance=0.12,
                timeout=5.0,
            )
            if self._is_successful:
                self.get_logger().info('Arm tucked successfully')
            else:
                self.get_logger().error(
                    'Arm motion finished but target configuration was not reached'
                )
        except Exception as e:
            self._is_successful = False
            self.get_logger().error(f'Failed to tuck arm: {e}')


def main(args=None):
    rclpy.init(args=args)

    arm_tucker = ArmTucker()

    # Wait for MoveIt2 to be ready
    if not arm_tucker.wait_for_moveit(timeout=30.0):
        arm_tucker.get_logger().error('MoveIt2 not ready, aborting')
        arm_tucker.close()
        rclpy.shutdown()
        return

    for i in range(5):
        arm_tucker.get_logger().info(f'Tucking arm... Try {i + 1}')
        arm_tucker.tuck_arm()

        if arm_tucker.is_successful():
            arm_tucker.get_logger().info('Arm tucked successfully!')
            break
        else:
            arm_tucker.get_logger().error('Tuck failed, retrying...')
            time.sleep(2)

    if not arm_tucker.is_successful():
        arm_tucker.get_logger().error('Failed to tuck arm after 5 tries')

    # Keep node alive for a moment
    time.sleep(1)

    arm_tucker.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
