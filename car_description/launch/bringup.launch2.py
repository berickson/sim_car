#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    
    use_sim = False

    if use_sim:
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sim_car = get_package_share_directory('sim_car')

    bringup_dir = get_package_share_directory('sim_car')
    launch_dir = os.path.join(bringup_dir, 'launch')
    if use_sim:
        world_config = DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_sim_car, 'worlds', 'simple.world'), ''],
            description='SDF world file')

    # Specify the actions

    rviz_config_path = os.path.join(pkg_sim_car, 'config', 'red-crash.rviz')
    start_rviz2_cmd = ExecuteProcess(
        cmd=['rviz2', '--display-config', rviz_config_path],
        cwd=[launch_dir],
        output='screen')

    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')

    nav2_params_path = os.path.join(
        get_package_share_directory('sim_car'),
        'config/',
        'nav2_params.yaml')

    behavior_tree_path = os.path.join(
        get_package_share_directory('sim_car'),
        'behavior_trees',
        'navigate_w_replanning_time2.xml')

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_path,
        param_rewrites={'default_nav_to_pose_bt_xml': behavior_tree_path, 'use_sim_time': 'False'},
        convert_types=True)

    nav_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'use_composition': "1",
            'slam': "1",
            'map': 'map.yaml',
            'params_file': configured_nav2_params,
        }.items()
    )

    # # Gazebo launch
    if use_sim:
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
            )
        )

        car = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                # os.path.join(pkg_sim_car, 'launch', 'spawn_red_crash.launch.py'),
                os.path.join(pkg_sim_car, 'launch', 'spawn_car.launch.py'),
            )
        )

        # joy = Node(
        #         package='joy', executable='joy_node', output='screen')
        # teleop_twist = Node(
        #         package='teleop_twist_joy',
        #         executable='teleop_node',
        #         output='screen',
        #         parameters=[os.path.join(pkg_sim_car, "config", "teleop_twist_joy.yaml")])

    ld = LaunchDescription()
#    ld.add_action(world_config)
#    ld.add_action(gazebo)
    # ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    # ld.add_action(TimerAction(
    #         period=5.0,
    #         actions=[start_rviz2_cmd]))
#    ld.add_action(car)
    # ld.add_action(joy)
    # ld.add_action(teleop_twist)
    ld.add_action(
        TimerAction(
            period=0.0,
            actions=[nav_bringup_cmd]
        ))
    return ld
