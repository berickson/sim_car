#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import xacro

xacro_file = os.path.join(get_package_share_directory('sim_car'), 'robot', 'car.urdf.xacro')
assert os.path.exists(xacro_file), "The xacro doesnt exist in "+str(xacro_file)

robot_description_config = xacro.process_file(xacro_file)
robot_desc = robot_description_config.toxml()

# print(robot_desc)
