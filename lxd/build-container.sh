lxc launch ubuntu:22.04 car -p default -p gui
lxc file push install-inside-container.sh car/root/
lxc exec car bash install-inside-container.sh