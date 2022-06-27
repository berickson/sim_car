sleep 1

function wait_for_lock {
    i=0
    tput sc
    while fuser /var/lib/dpkg/lock >/dev/null 2>&1 ; do
        case $(($i % 4)) in
            0 ) j="-" ;;
            1 ) j="\\" ;;
            2 ) j="|" ;;
            3 ) j="/" ;;
        esac
        tput rc
        echo -en "\r[$j] Waiting for other software managers to finish..." 
        sleep 0.5
        ((i=i+1))
    done 
}

echo Updating OS
wait_for_lock
apt-mark hold libsystemd0
wait_for_lock
apt-get update 
wait_for_lock
apt-get install locales -y
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
LANG=en_US.UTF-8
wait_for_lock
apt-get install curl gnupg lsb-release -y
wait_for_lock
apt-get autoremove -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
/bin/bash -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main"' | tee /etc/apt/sources.list.d/ros2.list > /dev/null

wait_for_lock
DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
wait_for_lock
apt-mark unhold libsystemd0

echo Installing ROS Humble
wait_for_lock
DEBIAN_FRONTEND=noninteractive apt-get install ros-humble-desktop -y
echo Installing colcon and rosdep
wait_for_lock
DEBIAN_FRONTEND=noninteractive apt-get install python3-colcon-common-extensions python3-rosdep -y

rosdep init
rosdep update

echo sourcing ROS Humble
. /opt/ros/humble/setup.bash

echo cloning source code from Github
git clone https://github.com/berickson/sim_car.git

echo installing project dependencies
wait_for_lock
DEBIAN_FRONTEND=noninteractive rosdep install -y --from-paths sim_car/car_description/
wait_for_lock
DEBIAN_FRONTEND=noninteractive rosdep install -y --from-paths sim_car/car_gazebo_plugin/
touch /root/sim_car/car_description/params