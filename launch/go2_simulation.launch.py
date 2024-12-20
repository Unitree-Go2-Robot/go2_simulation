# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FindExecutable
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():

    declare_pkg_dir_cmd = DeclareLaunchArgument(
        'pkg_dir',
        default_value='go2_description',
        description='Package'
    )

    declare_description_filer_cmd = DeclareLaunchArgument(
        'description_file',
        default_value='go2_description.urdf.xacro',
        description='URDF'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch rviz'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )

    declare_gazebo_gui_cmd = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='True',
        description='Use gazebo GUI'
    )

    declare_lite_cmd = DeclareLaunchArgument(
        'lite',
        default_value='False',
        description='Lite'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='go2',
        description='Robot name'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='World name'
    )

    declare_world_init_x_cmd = DeclareLaunchArgument(
        'world_init_x',
        default_value='0.0',
        description='Initial world x position'
    )

    declare_world_init_y_cmd = DeclareLaunchArgument(
        'world_init_y',
        default_value='0.0',
        description='Initial world y position'
    )

    declare_world_init_z_cmd = DeclareLaunchArgument(
        'world_init_z',
        default_value='0.275',
        description='Initial world z position'
    )

    declare_world_init_heading_cmd = DeclareLaunchArgument(
        'world_init_heading',
        default_value='0.0',
        description='Initial world heading'
    )

    description_path = os.path.join(get_package_share_directory("go2_description"), "urdf/go2_description.urdf.xacro")
    joints_path = os.path.join(get_package_share_directory("go2_simulation"), "config/go2_joints.yaml")
    links_path = os.path.join(get_package_share_directory("go2_simulation"), "config/go2_links.yaml")
    gait_path = os.path.join(get_package_share_directory("go2_simulation"), "config/go2_gait.yaml")

    go2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": description_path,
            "joints_map_path": joints_path,
            "links_map_path": links_path,
            "gait_config_path": gait_path,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("go2_simulation"),
                "launch",
                "gazebo.launch.py",
            )
        ),
    )

    ld = LaunchDescription()
    ld.add_action(declare_pkg_dir_cmd)
    ld.add_action(declare_description_filer_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gazebo_gui_cmd)
    ld.add_action(declare_lite_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_world_init_x_cmd)
    ld.add_action(declare_world_init_y_cmd)
    ld.add_action(declare_world_init_z_cmd)
    ld.add_action(declare_world_init_heading_cmd)
    # ld.add_action(go2_bringup)
    ld.add_action(gazebo_bringup)

    return ld
