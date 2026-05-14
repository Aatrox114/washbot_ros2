#!/usr/bin/env python3
import math
import time
from pathlib import Path
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import yaml


def yaw_to_quaternion(yaw: float):
    """
    只考虑平面 yaw，转换成 geometry_msgs Quaternion 需要的四元数。
    """
    half = yaw * 0.5
    z = math.sin(half)
    w = math.cos(half)
    return 0.0, 0.0, z, w


def normalize_angle(angle: float) -> float:
    """
    归一化到 [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WashbotRectPatrol(Node):
    def __init__(self):
        super().__init__("washbot_rect_patrol")

        self.declare_parameter(
            "config_file",
            str(Path.home() / "ros2_ws/src/washbot_decision/config/wash_task.yaml"),
        )

        self.config_file = self.get_parameter("config_file").value
        self.config = self.load_config(self.config_file)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("WashBot 矩形巡航决策节点启动")
        self.get_logger().info(f"配置文件: {self.config_file}")

    def load_config(self, path: str) -> Dict:
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"配置文件不存在: {path}")

        with p.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if data is None:
            raise RuntimeError(f"配置文件为空: {path}")

        return data

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def wait_nav2_server(self):
        self.get_logger().info("等待 Nav2 navigate_to_pose action server...")
        ok = self.nav_client.wait_for_server(timeout_sec=30.0)

        if not ok:
            raise RuntimeError(
                "等待 Nav2 action server 超时。请确认 Nav2 已启动，并且 Navigation 显示 active。"
            )

        self.get_logger().info("Nav2 action server 已连接")

    def navigate_to(self, name: str, x: float, y: float, yaw: float) -> bool:
        """
        调用 Nav2 NavigateToPose 到指定目标。
        """
        self.get_logger().info(
            f"导航到 {name}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(x, y, yaw)

        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error(f"{name}: 发送目标失败，goal_handle 为空")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"{name}: Nav2 拒绝目标")
            return False

        self.get_logger().info(f"{name}: 目标已接受，等待结果...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result is None:
            self.get_logger().error(f"{name}: 获取导航结果失败")
            return False

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"{name}: 导航成功")
            return True

        self.get_logger().error(f"{name}: 导航失败，status={status}")
        return False

    def publish_stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def rotate_scan(self):
        """
        场景交互点处的原地低速扫描。
        这一版只负责旋转采集视野，不做点云拟合。
        """
        scan_cfg = self.config.get("local_scan", {})
        enable = bool(scan_cfg.get("enable", True))

        if not enable:
            self.get_logger().info("local_scan.enable=false，跳过自转扫描")
            return

        angular_speed = float(scan_cfg.get("angular_speed", 0.03))
        scan_time = float(scan_cfg.get("scan_time", 25.0))
        stop_time = float(scan_cfg.get("stop_time", 1.0))

        self.get_logger().info(
            f"开始自转扫描: angular_speed={angular_speed:.3f} rad/s, scan_time={scan_time:.1f} s"
        )

        msg = Twist()
        msg.angular.z = angular_speed

        start = time.time()
        rate_hz = 20.0
        period = 1.0 / rate_hz

        while rclpy.ok() and (time.time() - start) < scan_time:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.get_logger().info("自转扫描结束，停车")
        for _ in range(10):
            self.publish_stop()
            time.sleep(0.05)

        if stop_time > 0:
            time.sleep(stop_time)

    def generate_rect_path(self) -> List[Tuple[str, float, float, float]]:
        """
        根据 washbay 和 patrol 配置生成矩形巡航路径。

        输出列表：
        [
          (name, x, y, yaw),
          ...
        ]
        """
        washbay = self.config["washbay"]
        patrol = self.config["patrol"]

        cx = float(washbay["center_x"])
        cy = float(washbay["center_y"])
        theta = float(washbay.get("yaw", 0.0))

        patrol_w = float(patrol.get("width", 3.7))
        patrol_l = float(patrol.get("length", 6.7))
        points_per_edge = int(patrol.get("points_per_edge", 3))
        face_mode = str(patrol.get("face_mode", "path_direction"))

        if points_per_edge < 1:
            points_per_edge = 1

        hw = patrol_w * 0.5
        hl = patrol_l * 0.5

        # u 是宽度方向，v 是长度方向
        ux = math.cos(theta)
        uy = math.sin(theta)

        vx = -math.sin(theta)
        vy = math.cos(theta)

        # 四个角点：P1 -> P2 -> P3 -> P4
        corners = [
            (cx - hw * ux - hl * vx, cy - hw * uy - hl * vy),
            (cx + hw * ux - hl * vx, cy + hw * uy - hl * vy),
            (cx + hw * ux + hl * vx, cy + hw * uy + hl * vy),
            (cx - hw * ux + hl * vx, cy - hw * uy + hl * vy),
        ]

        # 每条边插点，避免 Nav2 直接斜着切
        raw_points: List[Tuple[float, float]] = []
        for i in range(4):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % 4]

            for k in range(points_per_edge):
                t = k / float(points_per_edge)
                x = x1 * (1.0 - t) + x2 * t
                y = y1 * (1.0 - t) + y2 * t
                raw_points.append((x, y))

        # 回到起点，闭合一圈
        raw_points.append(raw_points[0])

        path: List[Tuple[str, float, float, float]] = []

        for i, (x, y) in enumerate(raw_points):
            if i < len(raw_points) - 1:
                nx, ny = raw_points[i + 1]
            else:
                nx, ny = raw_points[i]

            if face_mode == "face_center":
                yaw = math.atan2(cy - y, cx - x)
            else:
                yaw = math.atan2(ny - y, nx - x)

            yaw = normalize_angle(yaw)
            name = f"patrol_{i + 1:02d}"
            path.append((name, x, y, yaw))

        self.get_logger().info("生成矩形巡航路径:")
        self.get_logger().info(
            f"  工位中心: ({cx:.3f}, {cy:.3f}), yaw={theta:.3f}"
        )
        self.get_logger().info(
            f"  巡航尺寸: width={patrol_w:.3f}, length={patrol_l:.3f}"
        )
        self.get_logger().info(f"  路径点数量: {len(path)}")

        for name, x, y, yaw in path:
            self.get_logger().info(
                f"  {name}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
            )

        return path

    def run_task(self):
        self.wait_nav2_server()

        interaction = self.config["interaction_point"]
        ix = float(interaction["x"])
        iy = float(interaction["y"])
        iyaw = float(interaction.get("yaw", 0.0))

        # 1. 先到场景交互点
        ok = self.navigate_to("interaction_point", ix, iy, iyaw)
        if not ok:
            self.get_logger().error("到达场景交互点失败，任务终止")
            return

        # 2. 到点后自转扫描
        self.rotate_scan()

        # 3. 生成 3.7 × 6.7 矩形路径
        path = self.generate_rect_path()

        patrol = self.config["patrol"]
        wait_time = float(patrol.get("wait_time", 1.5))
        return_to_interaction = bool(patrol.get("return_to_interaction_point", False))

        # 4. 依次跑矩形巡航点
        for name, x, y, yaw in path:
            ok = self.navigate_to(name, x, y, yaw)
            if not ok:
                self.get_logger().error(f"{name} 导航失败，任务终止")
                return

            self.get_logger().info(f"{name}: 到点停留 {wait_time:.1f} 秒")
            time.sleep(wait_time)

        # 5. 可选回到交互点
        if return_to_interaction:
            self.get_logger().info("返回场景交互点")
            self.navigate_to("return_interaction_point", ix, iy, iyaw)

        self.publish_stop()
        self.get_logger().info("矩形巡航任务完成")


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = WashbotRectPatrol()
        node.run_task()
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
