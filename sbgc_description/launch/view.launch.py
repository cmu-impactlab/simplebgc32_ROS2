# Copyright 2026 Yousef Hussein
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
View the gimbal model with sliders driving the joints.

This is a description check, not a driver check: joint_state_publisher_gui
supplies the angles, so nothing here talks to hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    prefix = LaunchConfiguration('prefix')
    gui = LaunchConfiguration('gui')

    # ParameterValue with an explicit type, or launch tries to parse the
    # expanded URDF as YAML and fails on the first colon in an attribute.
    robot_description = ParameterValue(Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('sbgc_description'), 'urdf', 'standalone.urdf.xacro',
        ]),
        ' prefix:=', prefix,
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='gimbal_'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Run joint_state_publisher_gui. Set false in a headless check.'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),
    ])
