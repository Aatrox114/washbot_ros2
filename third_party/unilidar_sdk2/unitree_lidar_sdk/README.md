sudo ip addr flush dev enp4s0
sudo ip addr add 192.168.1.2/24 dev enp4s0
sudo ip link set enp4s0 up




cd ~/unilidar_sdk2/unitree_lidar_sdk

mkdir -p build
cd build

cmake ..
make -j2





cd ~/unilidar_sdk2/unitree_lidar_sdk/build
../bin/lidar_control_udp stop

../bin/lidar_control_udp start
ros2 launch unitree_lidar_ros2 launch.py


../bin/lidar_control_serial stop
../bin/lidar_control_serial start
../bin/lidar_control_serial reset

../bin/lidar_set_mode_udp 0
../bin/lidar_set_mode_udp 1
../bin/lidar_set_mode_udp 2
../bin/lidar_set_mode_udp 16
