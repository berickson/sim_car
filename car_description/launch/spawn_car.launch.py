#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import TimerAction

import xacro


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('sim_car'), 'robot/', 'car.urdf.xacro')
    assert os.path.exists(xacro_file), "The xacro doesnt exist in " + str(xacro_file)

    install_dir = get_package_prefix('sim_car')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = \
            os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = \
            os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    spawn_car = Node(
        package="gazebo_ros",
        executable='spawn_entity.py',
        arguments=["-topic", "/robot_description", "-entity", "car"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen")

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        TimerAction(
            period=0.0,
            actions=[spawn_car]),
        TimerAction(
            period=0.0,
            actions=[robot_state_publisher])
    ])

    return ld
