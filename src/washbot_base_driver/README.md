# washbot_base_driver

洗车机器人 ROS2 C++ 底盘串口驱动节点。

## 1. 功能

本节点负责完成 ROS2 上位机和下位机之间的串口通信。

主要功能：

- 订阅 `/cmd_vel`
- 将速度指令通过串口发送给下位机
- 接收下位机返回的里程计数据
- 发布 `/odom`
- 发布 `odom -> base_link` TF
- 发布 `/base_status`
- `/cmd_vel` 超时后自动发送停车指令

---

## 2. 当前通信链路

```text
/cmd_vel
  ↓
base_driver_node
  ↓
串口
  ↓
下位机
  ↓
里程计反馈
  ↓
base_driver_node
  ↓
/odom + TF