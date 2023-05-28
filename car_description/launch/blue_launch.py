#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
\
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_sim_car = get_package_share_directory('sim_car')
    bringup_dir = get_package_share_directory('sim_car')
    launch_dir = os.path.join(bringup_dir, 'launch')

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
        'blue_params.yaml')

    behavior_tree_path = os.path.join(
        get_package_share_directory('sim_car'),
        'behavior_trees',
        'navigate_w_replanning_time2.xml')

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_path,
        param_rewrites={'default_nav_to_pose_bt_xml': behavior_tree_path},
        convert_types=True)

    nav_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'slam': "1",
            'map': 'map.yaml',
            'use_sim_time': "false",
            'params_file': configured_nav2_params,
            }.items()
        )

    ld = LaunchDescription()
    ld.add_action(TimerAction(
            period=5.0,
            actions=[start_rviz2_cmd]))
    ld.add_action(
        TimerAction(
            period=0.0,
            actions=[nav_bringup_cmd]
        ))
    return ld
