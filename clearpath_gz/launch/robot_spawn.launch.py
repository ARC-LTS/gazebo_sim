# Copyright 2021 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from clearpath_config.clearpath_config import ClearpathConfig

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
    Shutdown
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


# Files the robot spawn group includes from setup_path. They are produced by
# the Clearpath generators (generate:=true) or must already exist there, as in
# the frozen RaMonES campaign configuration (generate:=false).
REQUIRED_GENERATED_FILES = [
    os.path.join('platform', 'launch', 'platform-service.launch.py'),
    os.path.join('sensors', 'launch', 'sensors-service.launch.py'),
    os.path.join('manipulators', 'launch', 'manipulators-service.launch.py'),
]


def default_setup_path():
    return EnvironmentVariable(
        'RAMONES_SETUP_PATH', default_value=os.path.expanduser('~/clearpath/'))


def check_setup_path(setup_path, generate):
    robot_yaml = os.path.join(setup_path, 'robot.yaml')
    if not os.path.isfile(robot_yaml):
        raise RuntimeError(
            f'No robot.yaml in Clearpath setup path {setup_path!r}. Set RAMONES_SETUP_PATH '
            '(or pass setup_path:=) to the campaign config, e.g. ramones_ws/config/clearpath '
            'as rendered by setup.sh.')
    if generate:
        return
    missing = [f for f in REQUIRED_GENERATED_FILES
               if not os.path.isfile(os.path.join(setup_path, f))]
    if missing:
        raise RuntimeError(
            f'No generated Clearpath config at {setup_path!r} (missing: {", ".join(missing)}). '
            'Point RAMONES_SETUP_PATH at the campaign config (ramones_ws/config/clearpath, '
            'rendered by setup.sh), or pass generate:=true to generate it. Note that '
            'generate:=true overwrites the generated files in setup_path.')


def continue_if_succeeded(step, next_actions):
    # OnProcessExit fires on any exit; stop the launch if a generator failed
    # instead of going on to include files it never wrote.
    def on_exit(event, context):
        if event.returncode != 0:
            return [Shutdown(
                reason=f'Clearpath generator {step!r} failed with exit code '
                       f'{event.returncode}; not spawning the robot.')]
        return next_actions
    return on_exit


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='warehouse_barricaded',
                          description='Gazebo World'),
    DeclareLaunchArgument('setup_path',
                          default_value=default_setup_path(),
                          description='Clearpath setup path (default: $RAMONES_SETUP_PATH, '
                                      'else ~/clearpath/)'),
    DeclareLaunchArgument('generate',
                          default_value='true',
                          choices=['true', 'false'],
                          description='Generate parameters and launch files')
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.15',
                 description='z component of the robot pose.'))


def launch_setup(context, *args, **kwargs):
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    generate = LaunchConfiguration('generate')

    check_setup_path(setup_path.perform(context),
                     generate.perform(context).lower() == 'true')

    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(os.path.join(
        str(setup_path.perform(context)), 'robot.yaml'))

    namespace = clearpath_config.system.namespace
    if namespace in ('', '/'):
        robot_name = 'robot'
    else:
        robot_name = namespace + '/robot'

    # Directories
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    # Paths
    rviz_launch = PathJoinSubstitution(
        [pkg_clearpath_viz, 'launch', 'view_robot.launch.py'])
    launch_file_platform_service = PathJoinSubstitution([
        setup_path, 'platform/launch', 'platform-service.launch.py'])
    launch_file_sensors_service = PathJoinSubstitution([
        setup_path, 'sensors/launch', 'sensors-service.launch.py'])
    launch_file_manipulators_service = PathJoinSubstitution([
        setup_path, 'manipulators/launch', 'manipulators-service.launch.py'])

    group_action_spawn_robot = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_platform_service]),
            launch_arguments=[
              ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_sensors_service]),
            launch_arguments=[
              ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_manipulators_service]),
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),
    ])

    node_generate_description = Node(
        package='clearpath_generator_common',
        executable='generate_description',
        name='generate_description',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path]
    )

    node_generate_semantic_description = Node(
        package='clearpath_generator_common',
        executable='generate_semantic_description',
        name='generate_semantic_description',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path]
    )

    node_generate_launch = Node(
        package='clearpath_generator_gz',
        executable='generate_launch',
        name='generate_launch',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path]
    )

    node_generate_param = Node(
        package='clearpath_generator_gz',
        executable='generate_param',
        name='generate_launch',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path]
    )

    event_generate_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_description,
            on_exit=continue_if_succeeded('generate_description', [node_generate_semantic_description])
        )
    )

    event_generate_semantic_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_semantic_description,
            on_exit=continue_if_succeeded('generate_semantic_description', [node_generate_launch])
        )
    )

    event_generate_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_launch,
            on_exit=continue_if_succeeded('generate_launch', [node_generate_param])
        )
    )

    event_generate_param = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_param,
            on_exit=continue_if_succeeded('generate_param', [group_action_spawn_robot])
        )
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    do_generate = GroupAction(
        actions=[
            node_generate_description,
            event_generate_description,
            event_generate_semantic_description,
            event_generate_launch,
            event_generate_param,
            rviz
        ],
        condition=IfCondition(LaunchConfiguration('generate'))
    )
    
    do_not_generate = GroupAction(actions=[group_action_spawn_robot],
                                  condition=UnlessCondition(LaunchConfiguration('generate')))

    return [
        do_generate,
        do_not_generate
    ]


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
