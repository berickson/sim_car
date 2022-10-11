# sim_car
New version of my http:/berickson/fake_car simulator targeting ROS 2. This is currently in early development and works with Galactic.

My goal is to make a series of Gazebo models, worlds, and plugins to simulate my robots for test.


## Getting Started

- install ROS2 Humble desktop full, see https://docs.ros.org/en/humble/Installation.html
- make a workspace, see https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
- clone this repo into your workspaces src folder
- cd workspace folder
- install dependencies
```
pip3 install transforms3d
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```
- Then:

```
colcon build --simlink-install
source install/setup.bash
ros2 launch sim_car bringup.launch
```

- verify it works by driving with keyboard in another terminal:
```
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


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
| --- | --- |
| /car/cmd_ackermann | Publish messages here to control the car |
| /joy | Car listens to joystick and turns it into Ackermman commands |
| /car/odo_fl | Odometer tick count for front left wheel |
| /car/odo_fr | Odometer tick count for front right wheel |
| /car/pose | Ground truth pose from Simulator |
| /scan | Lidar scan output |
| /joint_states | joint positions from Gazebo |

## Launch Files

| File |  |
| --- | --- |
| all.launch.py | Run latest test setup in Gazebo, currently an Ackermann drive car in a simple world with joystick control |
| spawn_car.launch.py | Spawn an Ackermann drive car in Gazebo |
| spawn_world.launch.py | Launch Gazeboy with an empty world |

## Todo List
I keep the todo for this on in a Google Doc (https://docs.google.com/document/d/1eDoU0HPi217dMZmRpME3X8Uc7ZKnQc10sKgLSsA3TdY/edit?usp=sharing)
