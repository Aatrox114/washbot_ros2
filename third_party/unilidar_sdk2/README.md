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
