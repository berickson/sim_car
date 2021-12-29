# sim_car
New version of my http:/berickson/fake_car simulator targeting ROS 2. This is currently in early development and works with Galactic.

My goal is to make a series of Gazebo models, worlds, and plugins to simulate my robots for test.


## Getting Started

- install ROS2 Galactic
- install other required packages (list tbd)
- clone this repo into your workspaces src folder
- colcon build
- soure install/setup.bash

## Models
### car
- Simulate a Traxxas Slash robot like blue-crash (https://github.com/berickson/car).
- Ackermann steering
- Top mounted lidar
- Odometers on front wheels
- Independent shocks on all wheels
- Rear wheel drive

#### ROS 2 Integration

| Topic | |
| /car/cmd_ackermann | Publish messages here to control the car |
| /joy | Car listens to joystick and turns it into Ackermman commands |
| /car/odo_fl | Odometer tick count for front left wheel |
| /car/odo_fr | Odometer tick count for front right wheel |
| /car/pose | Ground truth pose from Simulator |
| /scan | Lidar scan output |
| /joint_states | joint positions from Gazebo

## Launch Files

| File |  |
| --- | --- |
| all.launch.py | Run latest test setup in Gazebo, currently an Ackermann drive car in a simple world with joystick control |
| spawn_car.launch.py | Spawn an Ackermann drive car in Gazebo |
| spawn_world.launch.py | Launch Gazeboy with an empty world |
