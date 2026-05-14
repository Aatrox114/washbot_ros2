#!/usr/bin/env python3
import math
import time
from pathlib import Path
from typing import Dict

import yaml
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class CenterScanOdomNode(Node):
    def __init__(self):
        super().__init__("center_scan_odom_node")

        self.declare_parameter(
            "config_file",
            str(Path.home() / "ros2_ws/src/washbot_decision/config/wash_task.yaml"),
        )

        self.config_file = self.get_parameter("config_file").value
        self.config = self.load_config(self.config_file)

        self.cfg = self.config.get("center_scan_mapping", {})

        self.target_x = float(self.cfg.get("target_x", 0.0))
        self.target_y = float(self.cfg.get("target_y", 0.0))
        self.target_yaw = float(self.cfg.get("target_yaw", 0.0))

        self.position_tolerance = float(self.cfg.get("position_tolerance", 0.08))
        self.heading_tolerance = float(self.cfg.get("heading_tolerance", 0.08))

        self.max_linear_speed = float(self.cfg.get("max_linear_speed", 0.08))
        self.max_angular_speed = float(self.cfg.get("max_angular_speed", 0.25))

        self.kp_linear = float(self.cfg.get("kp_linear", 0.45))
        self.kp_angular = float(self.cfg.get("kp_angular", 1.2))

        self.scan_angular_speed = float(self.cfg.get("scan_angular_speed", 0.025))
        self.scan_time = float(self.cfg.get("scan_time", 35.0))
        self.stop_time = float(self.cfg.get("stop_time", 1.0))

        self.odom_msg = None

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.get_logger().info("中心点自动扫描建图节点启动")
        self.get_logger().info(f"配置文件: {self.config_file}")
        self.get_logger().info(
            f"目标点 odom 坐标: x={self.target_x:.3f}, y={self.target_y:.3f}, yaw={self.target_yaw:.3f}"
        )

    def load_config(self, path: str) -> Dict:
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"配置文件不存在: {path}")
        with p.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if data is None:
            raise RuntimeError(f"配置文件为空: {path}")
        return data

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg

    def publish_stop(self, n: int = 10):
        msg = Twist()
        for _ in range(n):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def get_pose(self):
        if self.odom_msg is None:
            return None

        p = self.odom_msg.pose.pose.position
        q = self.odom_msg.pose.pose.orientation
        yaw = yaw_from_quat(q)
        return p.x, p.y, yaw

    def wait_odom(self):
        self.get_logger().info("等待 /odom...")
        start = time.time()
        while rclpy.ok() and self.odom_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 10.0:
                raise RuntimeError("等待 /odom 超时，请检查 Gazebo DiffDrive / bridge 是否正常")
        self.get_logger().info("/odom 已收到")

    def go_to_center(self):
        self.get_logger().info("开始自动前往中心点...")

        rate = 20.0
        period = 1.0 / rate

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            pose = self.get_pose()
            if pose is None:
                time.sleep(period)
                continue

            x, y, yaw = pose

            dx = self.target_x - x
            dy = self.target_y - y
            dist = math.hypot(dx, dy)

            if dist <= self.position_tolerance:
                self.get_logger().info(
                    f"已到达中心点附近: 当前 x={x:.3f}, y={y:.3f}, dist={dist:.3f}"
                )
                self.publish_stop()
                return

            target_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(target_heading - yaw)

            cmd = Twist()

            # 先转向目标，再前进，避免边走边大角度偏航导致建图乱
            if abs(heading_error) > 0.25:
                cmd.linear.x = 0.0
                cmd.angular.z = clamp(
                    self.kp_angular * heading_error,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )
            else:
                speed = clamp(
                    self.kp_linear * dist,
                    0.02,
                    self.max_linear_speed,
                )
                # 角度误差越大，线速度越慢
                speed *= max(0.2, 1.0 - abs(heading_error))

                cmd.linear.x = speed
                cmd.angular.z = clamp(
                    self.kp_angular * heading_error,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )

            self.cmd_pub.publish(cmd)
            time.sleep(period)

    def align_final_yaw(self):
        self.get_logger().info("调整最终朝向...")

        rate = 20.0
        period = 1.0 / rate

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            pose = self.get_pose()
            if pose is None:
                time.sleep(period)
                continue

            _, _, yaw = pose
            err = normalize_angle(self.target_yaw - yaw)

            if abs(err) <= self.heading_tolerance:
                self.publish_stop()
                self.get_logger().info("最终朝向调整完成")
                return

            cmd = Twist()
            cmd.angular.z = clamp(
                self.kp_angular * err,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self.cmd_pub.publish(cmd)
            time.sleep(period)

    def rotate_scan(self):
        self.get_logger().info(
            f"开始中心点原地扫描: angular_speed={self.scan_angular_speed:.3f}, scan_time={self.scan_time:.1f}s"
        )

        msg = Twist()
        msg.angular.z = self.scan_angular_speed

        start = time.time()
        rate = 20.0
        period = 1.0 / rate

        while rclpy.ok() and time.time() - start < self.scan_time:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.cmd_pub.publish(msg)
            time.sleep(period)

        self.publish_stop()

        if self.stop_time > 0:
            time.sleep(self.stop_time)

        self.get_logger().info("中心点扫描完成")

    def run(self):
        self.wait_odom()
        self.go_to_center()
        self.align_final_yaw()
        self.rotate_scan()
        self.publish_stop()
        self.get_logger().info("自动前往中心点并扫描建图流程完成")


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = CenterScanOdomNode()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"任务异常: {e}")
        else:
            print(f"任务异常: {e}")
    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
