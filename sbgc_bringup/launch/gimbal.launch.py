# Copyright 2026 Yousef Hussein
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Bring up the gimbal driver, its description and TF.

The driver is a lifecycle node and is deliberately NOT auto-transitioned here.
Configuring opens the serial port and activating starts commanding, and both
should be things someone asks for. Pass auto_start:=true when a supervisor
wants them done for it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    prefix = LaunchConfiguration('prefix')
    params_file = LaunchConfiguration('params_file')
    auto_start = LaunchConfiguration('auto_start')

    # ParameterValue with an explicit type, or launch tries to parse the
    # expanded URDF as YAML and fails on the first colon in an attribute.
    robot_description = ParameterValue(Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('sbgc_description'), 'urdf', 'standalone.urdf.xacro',
        ]),
        ' prefix:=', prefix,
    ]), value_type=str)

    driver = LifecycleNode(
        package='sbgc_driver',
        executable='sbgc_driver_node',
        name='sbgc_driver',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {
                'port': LaunchConfiguration('port'),
                'allow_control': LaunchConfiguration('allow_control'),
                'simulate': LaunchConfiguration('simulate'),
                'joint_prefix': prefix,
            },
        ],
    )

    # robot_state_publisher owns TF: the driver publishes /joint_states and this
    # turns them into transforms. Two publishers of the same transform is a
    # conflict rather than redundancy, which is why the driver's publish_tf
    # defaults to false.
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(driver),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(auto_start),
    )

    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='gimbal_'),
        DeclareLaunchArgument(
            'port', default_value='',
            description='Serial device. Empty discovers one.'),
        DeclareLaunchArgument(
            'allow_control', default_value='false',
            description='Process-level half of the motion gate. False brings the '
                        'driver up as a monitor that will not move anything.'),
        DeclareLaunchArgument('simulate', default_value='false'),
        DeclareLaunchArgument(
            'auto_start', default_value='false',
            description='Configure and activate without being asked.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('sbgc_driver'), 'config', 'sbgc_driver.yaml',
            ])),

        driver,
        state_publisher,
        activate_on_configured,
        configure,
    ])
