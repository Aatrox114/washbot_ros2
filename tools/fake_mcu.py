#!/usr/bin/env python3
import math
import os
import struct
import time
import select

PORT = "/tmp/washbot_mcu"

HEAD = b"\xAA\x55"
TYPE_CMD_VEL = 0x01
TYPE_ODOM = 0x02
LEN_CMD_VEL = 7
LEN_ODOM = 20


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def make_odom_frame(seq, x, y, yaw, vx, vy, wz, status=0):
    x_mm = int(round(x * 1000.0))
    y_mm = int(round(y * 1000.0))
    yaw_mrad = int(round(yaw * 1000.0))

    vx_mm_s = int(round(vx * 1000.0))
    vy_mm_s = int(round(vy * 1000.0))
    wz_mrad_s = int(round(wz * 1000.0))

    payload = struct.pack(
        "<Biii hhh B",
        seq & 0xFF,
        x_mm,
        y_mm,
        yaw_mrad,
        vx_mm_s,
        vy_mm_s,
        wz_mrad_s,
        status & 0xFF
    )

    frame = bytearray()
    frame += HEAD
    frame += bytes([TYPE_ODOM])
    frame += bytes([LEN_ODOM])
    frame += payload

    crc = crc16_modbus(frame)
    frame += struct.pack("<H", crc)

    return bytes(frame)


def parse_cmd_frame(buf: bytearray):
    while len(buf) >= 6:
        if buf[0] != 0xAA or buf[1] != 0x55:
            del buf[0]
            continue

        frame_type = buf[2]
        length = buf[3]
        total_len = 2 + 1 + 1 + length + 2

        if len(buf) < total_len:
            return None

        frame = bytes(buf[:total_len])
        recv_crc = struct.unpack("<H", frame[-2:])[0]
        calc_crc = crc16_modbus(frame[:-2])

        if recv_crc != calc_crc:
            print("[fake_mcu] CRC错误，丢弃1字节")
            del buf[0]
            continue

        payload = frame[4:4 + length]
        del buf[:total_len]

        if frame_type == TYPE_CMD_VEL and length == LEN_CMD_VEL:
            seq = payload[0]
            vx_mm_s, vy_mm_s, wz_mrad_s = struct.unpack("<hhh", payload[1:7])

            vx = vx_mm_s / 1000.0
            vy = vy_mm_s / 1000.0
            wz = wz_mrad_s / 1000.0

            return seq, vx, vy, wz

        print(f"[fake_mcu] 未知帧 type=0x{frame_type:02X}, len={length}")

    return None


def safe_write(fd, data: bytes):
    """
    非阻塞串口写入。
    如果缓冲区暂时满了，就本次跳过，避免假下位机直接崩掉。
    """
    try:
        os.write(fd, data)
        return True
    except BlockingIOError:
        return False
    except OSError as e:
        print(f"[fake_mcu] 串口写入错误: {e}")
        return False


def main():
    print(f"[fake_mcu] 等待虚拟串口: {PORT}")

    while not os.path.exists(PORT):
        time.sleep(0.1)

    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    print(f"[fake_mcu] 已打开 {PORT}")
    print("[fake_mcu] 等待 ROS2 base_driver_node 发送速度帧...")

    rx_buf = bytearray()

    x = 0.0
    y = 0.0
    yaw = 0.0

    vx = 0.0
    vy = 0.0
    wz = 0.0

    seq = 0

    last_time = time.time()
    last_print = time.time()
    last_odom_send = time.time()
    last_no_reader_warn = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # 里程计积分：把机器人自身坐标系速度转换到 odom 坐标系
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        x += (vx * cos_yaw - vy * sin_yaw) * dt
        y += (vx * sin_yaw + vy * cos_yaw) * dt
        yaw += wz * dt

        while yaw > math.pi:
            yaw -= 2.0 * math.pi
        while yaw < -math.pi:
            yaw += 2.0 * math.pi

        # 读取 ROS2 节点发来的速度帧
        rlist, _, _ = select.select([fd], [], [], 0.001)

        if rlist:
            try:
                data = os.read(fd, 1024)
                if data:
                    rx_buf.extend(data)
            except BlockingIOError:
                pass

        cmd = parse_cmd_frame(rx_buf)

        if cmd is not None:
            seq, vx, vy, wz = cmd

            if now - last_print > 0.2:
                print(
                    f"[fake_mcu] 收到 cmd_vel: "
                    f"vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f} | "
                    f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
                )
                last_print = now

        # 50Hz 返回 odom
        if now - last_odom_send >= 0.02:
            odom_frame = make_odom_frame(seq, x, y, yaw, vx, vy, wz, status=0)
            ok = safe_write(fd, odom_frame)

            if not ok and now - last_no_reader_warn > 1.0:
                print("[fake_mcu] odom 写入暂时失败，可能 ROS2 节点未读取或缓冲区满")
                last_no_reader_warn = now

            last_odom_send = now

        time.sleep(0.001)


if __name__ == "__main__":
    main()
