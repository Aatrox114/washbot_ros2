本文记录如何在 `unilidar_sdk2` 官方 SDK 基础上，新增一个命令行控制工具，实现对 Unitree L2 雷达的启动、停止和重置控制。

## 1. 背景说明

Unitree L2 雷达默认通过网口 UDP 通信，默认参数为：

```text
雷达 IP：192.168.1.62
本机 IP：192.168.1.2
雷达端口：6101
本机端口：6201
````

官方 ROS2 驱动可以正常发布：

```text
/unilidar/cloud
/unilidar/imu
```

但是在调试过程中，如果只想让雷达在不断电的情况下停止旋转，单纯 `Ctrl+C` 关闭 ROS2 节点并不等价于关闭雷达旋转。

因此需要调用 SDK 中的控制接口：

```cpp
stopLidarRotation();
startLidarRotation();
resetLidar();
```

实现一个独立命令行工具：

```bash
../bin/lidar_control_udp stop
../bin/lidar_control_udp start
../bin/lidar_control_udp reset
```

---

## 2. 注意事项

在运行控制工具之前，需要先停止占用 UDP 端口的程序，例如：

```bash
pkill -f unitree_lidar_ros2_node
pkill -f unitree_lidar_ros2
pkill -f example_lidar_udp
pkill -f lidar_control_udp
sudo fuser -k 6201/udp
```

因为官方 ROS2 驱动和 SDK example 都会占用本机 UDP 端口 `6201`。
如果端口被占用，会出现类似错误：

```text
[UDPHandler] bind udp port failed.
Unilidar initialization failed!
```

---

## 3. 确认本机网卡 IP

雷达默认要求本机有线网卡配置为：

```text
192.168.1.2/24
```

假设有线网卡名称为 `enp4s0`，可以执行：

```bash
sudo ip addr flush dev enp4s0
sudo ip addr add 192.168.1.2/24 dev enp4s0
sudo ip link set enp4s0 up
```

检查：

```bash
ip addr show enp4s0
```

应该看到：

```text
inet 192.168.1.2/24
```

---

## 4. 新增控制源码文件

进入 SDK 目录：

```bash
cd ~/unilidar_sdk2/unitree_lidar_sdk
```

新增文件：

```bash
cat > examples/lidar_control_udp.cpp <<'EOF'
#include <iostream>
#include <string>
#include <unistd.h>
#include "unitree_lidar_sdk.h"

using namespace unilidar_sdk2;

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Usage: lidar_control_udp [stop|start|reset]" << std::endl;
        return 1;
    }

    std::string cmd = argv[1];

    std::string lidar_ip = "192.168.1.62";
    std::string local_ip = "192.168.1.2";
    unsigned short lidar_port = 6101;
    unsigned short local_port = 6201;

    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    int ret = lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip);
    if (ret != 0) {
        std::cerr << "Unilidar initialization failed!" << std::endl;
        return -1;
    }

    std::cout << "Unilidar initialization succeed." << std::endl;

    if (cmd == "stop") {
        std::cout << "Stopping lidar rotation..." << std::endl;
        lreader->stopLidarRotation();
    } else if (cmd == "start") {
        std::cout << "Starting lidar rotation..." << std::endl;
        lreader->startLidarRotation();
    } else if (cmd == "reset") {
        std::cout << "Resetting lidar..." << std::endl;
        lreader->resetLidar();
    } else {
        std::cerr << "Unknown command: " << cmd << std::endl;
        std::cerr << "Usage: lidar_control_udp [stop|start|reset]" << std::endl;
        lreader->closeUDP();
        return 1;
    }

    sleep(1);
    lreader->closeUDP();

    std::cout << "Done." << std::endl;
    return 0;
}
EOF
```

这个程序的核心逻辑是：

```cpp
lreader->initializeUDP(...)
```

先通过 UDP 初始化雷达连接，然后根据命令参数调用：

```cpp
lreader->stopLidarRotation();
lreader->startLidarRotation();
lreader->resetLidar();
```

---

## 5. 修改 CMakeLists.txt

在 `unitree_lidar_sdk/CMakeLists.txt` 末尾添加：

```cmake
add_executable(lidar_control_udp examples/lidar_control_udp.cpp)
target_link_libraries(lidar_control_udp libunilidar_sdk2.a)
```

可以使用命令自动追加：

```bash
cd ~/unilidar_sdk2/unitree_lidar_sdk

grep -q "lidar_control_udp" CMakeLists.txt || cat >> CMakeLists.txt <<'EOF'

add_executable(lidar_control_udp examples/lidar_control_udp.cpp)
target_link_libraries(lidar_control_udp libunilidar_sdk2.a)
EOF
```

检查是否添加成功：

```bash
grep -n "lidar_control_udp" CMakeLists.txt
```

正常应该看到：

```text
add_executable(lidar_control_udp examples/lidar_control_udp.cpp)
target_link_libraries(lidar_control_udp libunilidar_sdk2.a)
```

---

## 6. 重新编译 SDK

如果之前 `build` 目录来自其他路径，例如从 `Downloads/unilidar_sdk2` 复制过来，可能会出现 CMake 缓存路径错误：

```text
CMake Error: The current CMakeCache.txt directory ...
is different than the directory ...
```

这种情况下需要删除旧的 `build`，重新生成：

```bash
cd ~/unilidar_sdk2/unitree_lidar_sdk

rm -rf build
mkdir build
cd build

cmake ..
make -j2
```

编译完成后检查生成文件：

```bash
ls -l ../bin/lidar_control_udp
```

正常应该看到：

```text
../bin/lidar_control_udp
```

---

## 7. 使用命令控制雷达

进入 build 目录：

```bash
cd ~/unilidar_sdk2/unitree_lidar_sdk/build
```

### 停止雷达旋转

```bash
../bin/lidar_control_udp stop
```

正常输出：

```text
Unilidar initialization succeed.
Stopping lidar rotation...
Done.
```

### 启动雷达旋转

```bash
../bin/lidar_control_udp start
```

正常输出：

```text
Unilidar initialization succeed.
Starting lidar rotation...
Done.
```

### 重置雷达

```bash
../bin/lidar_control_udp reset
```

正常输出：

```text
Unilidar initialization succeed.
Resetting lidar...
Done.
```

---

## 8. 常见问题

### 8.1 找不到程序

如果运行：

```bash
../bin/lidar_control_udp stop
```

出现：

```text
bash: ../bin/lidar_control_udp: 没有那个文件或目录
```

说明程序没有编译出来。需要检查：

```bash
grep -n "lidar_control_udp" ~/unilidar_sdk2/unitree_lidar_sdk/CMakeLists.txt
ls ~/unilidar_sdk2/unitree_lidar_sdk/examples/lidar_control_udp.cpp
```

然后重新编译：

```bash
cd ~/unilidar_sdk2/unitree_lidar_sdk
rm -rf build
mkdir build
cd build
cmake ..
make -j2
```

---

### 8.2 UDP 端口绑定失败

如果出现：

```text
[UDPHandler] bind udp port failed.
Unilidar initialization failed!
```

说明本机 UDP 端口 `6201` 被占用。执行：

```bash
pkill -f unitree_lidar_ros2_node
pkill -f unitree_lidar_ros2
pkill -f example_lidar_udp
pkill -f lidar_control_udp
sudo fuser -k 6201/udp
```

检查：

```bash
sudo ss -lunp | grep 6201
```

如果没有输出，说明端口已经释放。

---

### 8.3 CMake 缓存路径错误

如果出现：

```text
The current CMakeCache.txt directory ...
is different than the directory ...
```

说明 `build` 目录记录的是旧路径，需要删除重建：

```bash
cd ~/unilidar_sdk2/unitree_lidar_sdk
rm -rf build
mkdir build
cd build
cmake ..
make -j2
```

---

## 9. 最终效果

完成以上步骤后，可以在不断开雷达电源的情况下，通过命令控制雷达旋转状态：

```bash
../bin/lidar_control_udp stop
../bin/lidar_control_udp start
../bin/lidar_control_udp reset
```

这使得调试过程中可以快速停止雷达旋转，避免长时间空转，也方便在 ROS2 驱动、SDK example、RTAB-Map 等不同测试流程之间切换。

---

## 10. 后续扩展

后续可以继续扩展类似工具，例如：

```bash
../bin/lidar_set_mode_udp 0
../bin/lidar_set_mode_udp 1
../bin/lidar_set_mode_udp 2
../bin/lidar_set_mode_udp 16
```

用于设置雷达工作模式，例如：

```text
mode 0  = 标准 FOV + 3D + IMU 开启 + 网口 + 上电自启动
mode 1  = 广角 FOV + 3D + IMU 开启 + 网口 + 上电自启动
mode 2  = 标准 FOV + 2D + IMU 开启 + 网口 + 上电自启动
mode 16 = 标准 FOV + 3D + IMU 开启 + 网口 + 上电不自启动
```

目前本项目中推荐保持：

```text
mode = 0
```

即：

```text
标准 FOV
3D 点云
IMU 开启
网口通信
上电自启动
```

然后在 ROS2 中使用：

```text
/unilidar/cloud
/unilidar/imu
```

进行点云显示、LaserScan 转换和后续 SLAM 测试。

```

---

另外你标题里说“写头文件”，实际更准确的说法是：**新增了一个 C++ 示例源码文件 `examples/lidar_control_udp.cpp`，不是单独写头文件**。README 里我已经按实际流程写成“新增控制源码文件 + 修改 CMakeLists.txt”。
```
