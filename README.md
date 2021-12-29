# sim_car
New version of my http:/berickson/fake_car simulator targeting ROS 2. This is currently in early development and works with Galactic.

My goal is to make a series of Gazebo models, worlds, and plugins to simulate my robots for test.


## Getting Started

- install ROS2 Galactic
- install other required packages (list tbd)
- clone this repo into your workspaces src folder
- colcon build
- soure install/setup.bash


## Launch Files

| File |  |
| --- | --- |
| all.launch.py | Run my latest test setup in Gazebo, currently an Ackermann drive car in a simple world with joystick control |
| spawn_car.launch.py | Spawn an Ackermann drive car in Gazebo |
| spawn_world.launch.py | Launch Gazeboy with an empty world |
