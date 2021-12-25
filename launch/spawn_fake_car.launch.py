import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # urdf = os.path.join(get_package_share_directory('box_car_description'), 'robot/', 'box_bot.urdf')    
    # assert os.path.exists(urdf), "Thebox_bot.urdf doesnt exist in "+str(urdf)

    xacro_file = os.path.join(get_package_share_directory('sim_car'), 'robot', 'car.urdf.xacro')
    assert os.path.exists(xacro_file), "The xacro doesnt exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    print(robot_desc)
    
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()


    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("box_car_description"),
    #         "config",
    #         "box_car_controllers.yaml",
    #     ]
    # )
    print ("x1")

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(package='sim_car', executable='spawn_bot.py', arguments=[robot_desc], output='screen'),
        # Node(
        #     package="robot_state_publisher",
        #     executable="robot_state_publisher",
        #     name="robot_state_publisher",
        #     parameters=[
        #         {"robot_description": robot_desc}],
        #     output="screen"),
        # Node(
        # package="controller_manager",
        # executable="ros2_control_node",
        # parameters=[{"robot_description": robot_desc}, 
        #             robot_controllers],
        # output={
        #     "stdout": "screen",
        #     "stderr": "screen",
        #     },
        # ),
    ])