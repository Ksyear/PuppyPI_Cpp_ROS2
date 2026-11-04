#!/usr/bin/env python3
"""LD06 LiDAR 드라이버(ROS 2): 시리얼 → sensor_msgs/LaserScan.

기본 로봇 이미지에서는 LiDAR 제조사 드라이버가 /dev/ttyAMA0(Pi UART, GPIO14/15) 230400 baud 에서 읽어 frame "laser" 로 발행했고,
static TF base_footprint → laser = (0, 0, 0.18 m) 을 함께 띄웠다(ldlidar/launch/LD06.launch, 팀 실측과 일치).
이 노드는 같은 포트·프레임으로 /scan 을 발행한다(해석은 puppy_lidar.ld06).
"""

import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from puppy_lidar import ld06


class Ld06Driver(Node):
    def __init__(self):
        super().__init__("ld06_driver")
        p = self.declare_parameter
        self.port = p("port", "/dev/ttyAMA0").value
        self.baud = int(p("baud", 230400).value)
        self.frame_id = p("frame_id", "laser").value
        topic = p("topic", "scan").value
        self.assembler = ld06.ScanAssembler(
            beams=int(p("beams", 450).value), range_min=float(p("range_min", 0.02).value),
            range_max=float(p("range_max", 12.0).value),
            self_box=(float(p("self_filter_x", 0.16).value), float(p("self_filter_y", 0.105).value))
            if bool(p("self_filter", True).value) else None,
            use_near_filter=bool(p("near_filter", True).value))
        self.stats_period = float(p("stats_log_period_sec", 10.0).value)
        self.pub = self.create_publisher(LaserScan, topic, qos_profile_sensor_data)
        self.parser = ld06.Parser()
        self.scans = 0
        self.last_scan_t = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="ld06-serial", daemon=True)
        self._thread.start()
        self.get_logger().info("LD06 %s @ %d baud → %s (frame %s)" % (self.port, self.baud, topic, self.frame_id))

    def _run(self):
        import serial  # python3-serial
        last_stats = time.monotonic()
        last_data = time.monotonic()
        warned = False
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.05) as ser:
                    self.get_logger().info("LD06 포트 열림: %s" % self.port)
                    while not self._stop.is_set():
                        data = ser.read(4096)
                        now = time.monotonic()
                        if data:
                            last_data = now
                            warned = False
                            for pkt in self.parser.feed(data):
                                scan = self.assembler.add(pkt)
                                if scan is not None:
                                    self._publish(scan)
                        elif now - last_data > 5.0 and not warned:
                            warned = True
                            self.get_logger().warning(
                                "LD06 데이터 없음 %.0f s: 연결(TX→GPIO15/RX), 전원, 포트(%s) 확인" % (now - last_data, self.port))
                        if now - last_stats >= self.stats_period:
                            last_stats = now
                            self.get_logger().info(
                                "LD06 통계: 스캔 %d, 패킷 %d, CRC 오류 %d, 재동기 %d, 각도 오류 %d, 회전 %.1f Hz" % (
                                    self.scans, self.parser.packets, self.parser.crc_errors, self.parser.resyncs,
                                    self.parser.angle_errors, self.assembler._speed / 360.0))
            except Exception as exc:  # 포트 없음·권한·분리 → 2 s 뒤 다시
                self.get_logger().warning("LD06 포트 %s 열기/읽기 실패: %s (2 s 뒤 재시도)" % (self.port, exc),
                                          throttle_duration_sec=30.0)
                self._stop.wait(2.0)

    def _publish(self, scan):
        now = self.get_clock().now()
        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_increment = scan["angle_increment"]
        msg.angle_max = msg.angle_increment * (len(scan["ranges"]) - 1)
        t = time.monotonic()
        msg.scan_time = float(t - self.last_scan_t) if self.last_scan_t else (
            360.0 / scan["speed"] if scan["speed"] else 0.1)
        self.last_scan_t = t
        msg.time_increment = msg.scan_time / max(1, len(scan["ranges"]) - 1)
        msg.range_min = self.assembler.range_min
        msg.range_max = self.assembler.range_max
        msg.ranges = [float(r) for r in scan["ranges"]]
        msg.intensities = [float(v) for v in scan["intensities"]]
        self.pub.publish(msg)
        self.scans += 1

    def destroy_node(self):
        self._stop.set()
        self._thread.join(1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ld06Driver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
