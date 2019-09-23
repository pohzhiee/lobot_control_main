# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""
Demo for spawn_entity.

Launches Gazebo and spawns a model
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import launch
import launch_ros

import lifecycle_msgs.msg

def generate_launch_description():
    desired_pos_gui_pkg_name = 'desired_joint_publisher'
    # Get gazebo_ros package path
    lobot_desc_share_path = get_package_share_directory('lobot_description')
    lobot_urdf_path = os.path.join(lobot_desc_share_path, 'robots/biped.urdf')

    # Launch GUI for desired positions
    desired_pos_gui = Node(package=desired_pos_gui_pkg_name, node_executable=desired_pos_gui_pkg_name,
                           arguments=[lobot_urdf_path], output='screen')
    ld = LaunchDescription([desired_pos_gui])
    return ld
