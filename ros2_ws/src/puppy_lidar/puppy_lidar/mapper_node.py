#!/usr/bin/env python3
"""LiDAR 연속 지도작성 노드: /scan → 자세 추정·점유격자(puppy_lidar.mapper_core).

출력
  /map                         nav_msgs/OccupancyGrid (transient_local, map_publish_period)
  TF map → base_footprint      스캔마다
  /lidar_map/pose              geometry_msgs/PoseStamped
  /lidar_map/image/compressed  sensor_msgs/CompressedImage(JPEG) — web_video_server 로 바로 보기:
                               http://<로봇IP>:8080/stream?topic=/lidar_map/image&type=ros_compressed
  /lidar_map/info              std_msgs/String(JSON) — 그 이미지의 좌표 정보(왼쪽 아래 m, px/m, 로봇 자세·픽셀).
                               규격: README 'PuppyPi Media v1'
  ~/save_map, ~/reset_map      std_srvs/Trigger
저장: 부팅(노드 시작)마다 <map_dir>/<YYYYmmdd_HHMMSS>/map.{pgm,yaml,png} 를 autosave_period 마다 덮어쓰고
     <map_dir>/latest.{pgm,yaml,png} 도 갱신. 오래된 세션은 keep_sessions 개만 남긴다(이 노드가 만든 폴더만).
"""

import json
import math
import os
import shutil
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from puppy_lidar.mapper_core import OccupancyMapper

SESSION_FMT = "%Y%m%d_%H%M%S"


class LidarMapper(Node):
    def __init__(self):
        super().__init__("lidar_mapper")
        p = self.declare_parameter
        self.map_frame = p("map_frame", "map").value
        self.base_frame = p("base_frame", "base_footprint").value
        self.laser = (float(p("laser_x", 0.0).value), float(p("laser_y", 0.0).value), float(p("laser_yaw", 0.0).value))
        self.core_args = dict(size_m=float(p("map_size_m", 30.0).value), resolution=float(p("resolution", 0.05).value),
                              max_range=float(p("max_range", 11.5).value), min_range=float(p("min_range", 0.12).value),
                              min_travel=float(p("min_travel_distance", 0.05).value),
                              min_turn=float(p("min_travel_heading", 0.05).value))
        self.mapper = OccupancyMapper(**self.core_args)
        self.map_dir = os.path.expanduser(p("map_dir", "~/puppypi_maps").value)
        self.keep_sessions = int(p("keep_sessions", 30).value)
        self.session = time.strftime(SESSION_FMT)
        self.jpeg_quality = int(p("image_jpeg_quality", 80).value)
        # map = 누적 지도(이전 방식) | live = 최신 스캔만(2026-11-05, 위치 추정용 지도는 내부에만)
        self.display_mode = p("display_mode", "map").value
        self.live_window = float(p("live_window_sec", 1.0).value)  # live: 이 시간 안의 스캔만 겹쳐 그림
        self.proc_ms = []

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "map", latched)
        self.pose_pub = self.create_publisher(PoseStamped, "lidar_map/pose", 10)
        self.img_pub = self.create_publisher(CompressedImage, "lidar_map/image/compressed", 1)
        self.info_pub = self.create_publisher(String, "lidar_map/info", 1)
        self.image_seq = 0
        self.tf = TransformBroadcaster(self)
        self.create_subscription(LaserScan, p("scan_topic", "scan").value, self._on_scan, qos_profile_sensor_data)
        self.create_timer(float(p("map_publish_period", 2.0).value), self._publish_map)
        self.create_timer(float(p("image_publish_period", 1.0).value), self._publish_image)
        self.create_timer(float(p("autosave_period", 30.0).value), lambda: self._save(auto=True))
        self.create_timer(30.0, self._log_stats)
        self.create_service(Trigger, "~/save_map", lambda req, resp: self._srv(resp, self._save(auto=False)))
        self.create_service(Trigger, "~/reset_map", self._srv_reset)
        self.get_logger().info("LiDAR 지도작성: %.0f m × %.0f m @ %.2f m, 저장 %s/%s" % (
            self.core_args["size_m"], self.core_args["size_m"], self.core_args["resolution"], self.map_dir, self.session))

    # ── 스캔 ──
    def _on_scan(self, msg):
        t0 = time.monotonic()
        pts = OccupancyMapper.scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                          max(msg.range_min, self.mapper.min_range),
                                          min(msg.range_max, self.mapper.max_range), self.laser)
        pose = self.mapper.process(pts)
        self.proc_ms.append((time.monotonic() - t0) * 1e3)
        if len(self.proc_ms) > 300:
            del self.proc_ms[:100]
        if not self.mapper.initialized:
            return
        q = (0.0, 0.0, math.sin(pose[2] / 2.0), math.cos(pose[2] / 2.0))
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y = float(pose[0]), float(pose[1])
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        self.tf.sendTransform(t)
        ps = PoseStamped()
        ps.header = t.header
        ps.pose.position.x, ps.pose.position.y = float(pose[0]), float(pose[1])
        ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
        self.pose_pub.publish(ps)

    # ── 발행 ──
    def _publish_map(self):
        m = self.mapper
        if not m.initialized:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = float(m.res)
        msg.info.width = msg.info.height = m.n
        msg.info.origin.position.x = msg.info.origin.position.y = float(m.origin)
        msg.info.origin.orientation.w = 1.0
        msg.data = m.occupancy().reshape(-1).tolist()
        self.map_pub.publish(msg)

    def _publish_image(self):
        m = self.mapper
        if not m.initialized or (self.img_pub.get_subscription_count() == 0 and self.info_pub.get_subscription_count() == 0):
            return
        import cv2
        img, info = (m.render_live_with_info(window_s=self.live_window) if self.display_mode == "live"
                     else m.render_with_info())
        stamp = self.get_clock().now()
        self.image_seq += 1
        if self.img_pub.get_subscription_count() > 0:
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                msg = CompressedImage()
                msg.header.stamp = stamp.to_msg()
                msg.header.frame_id = self.map_frame
                msg.format = "jpeg"
                msg.data = buf.tobytes()
                self.img_pub.publish(msg)
        info.update({"media": "puppypi-media/1", "frame": self.map_frame, "image_seq": self.image_seq,
                     "stamp_unix": time.time(), "session": self.session, "scans": m.scans,
                     "integrations": m.integrations, "map_size_m": m.n * m.res})
        self.info_pub.publish(String(data=json.dumps(info)))

    # ── 저장 ──
    def _save(self, auto):
        m = self.mapper
        if not m.initialized:
            return False, "아직 스캔 없음"
        import cv2
        d = os.path.join(self.map_dir, self.session)
        try:
            os.makedirs(d, exist_ok=True)
            for name in ("map", "latest"):
                base = os.path.join(d if name == "map" else self.map_dir, name)
                cv2.imwrite(base + ".pgm.tmp.pgm", m.pgm())
                os.replace(base + ".pgm.tmp.pgm", base + ".pgm")
                with open(base + ".yaml.tmp", "w") as fp:
                    fp.write("image: %s.pgm\nresolution: %.6f\norigin: [%.6f, %.6f, 0.000000]\nnegate: 0\n"
                             "occupied_thresh: 0.65\nfree_thresh: 0.25\n" % (name, m.res, m.origin, m.origin))
                os.replace(base + ".yaml.tmp", base + ".yaml")
                cv2.imwrite(base + ".png.tmp.png", m.render())
                os.replace(base + ".png.tmp.png", base + ".png")
            self._prune()
        except OSError as exc:
            self.get_logger().warning("지도 저장 실패(%s): %s" % (d, exc), throttle_duration_sec=60.0)
            return False, str(exc)
        if not auto:
            self.get_logger().info("지도 저장: %s" % d)
        return True, os.path.join(d, "map.pgm")

    def _prune(self):
        sessions = sorted(e for e in os.listdir(self.map_dir) if _is_session(os.path.join(self.map_dir, e)))
        for old in sessions[:-self.keep_sessions] if self.keep_sessions > 0 else []:
            if old != self.session:
                shutil.rmtree(os.path.join(self.map_dir, old), ignore_errors=True)

    def _srv(self, resp, result):
        resp.success, resp.message = result
        return resp

    def _srv_reset(self, req, resp):
        self._save(auto=True)
        self.mapper = OccupancyMapper(**self.core_args)
        self.session = time.strftime(SESSION_FMT)
        self.get_logger().info("지도 초기화 → 새 세션 %s (현재 위치가 새 원점)" % self.session)
        resp.success, resp.message = True, self.session
        return resp

    def _log_stats(self):
        m = self.mapper
        if not m.scans:
            self.get_logger().info("LiDAR 지도작성: 스캔 대기 중(/scan 없음)")
            return
        ms = sorted(self.proc_ms) or [0.0]
        self.get_logger().info("지도 통계: 스캔 %d, 통합 %d, 튐 거부 %d, 처리 p50 %.1f / p95 %.1f ms, 자세 (%.2f, %.2f, %.0f°)" % (
            m.scans, m.integrations, m.rejected_jumps, ms[len(ms) // 2], ms[int(len(ms) * 0.95)],
            m.pose[0], m.pose[1], math.degrees(m.pose[2])))

    def destroy_node(self):
        try:
            self._save(auto=True)
        finally:
            super().destroy_node()


def _is_session(path):
    name = os.path.basename(path)
    if not os.path.isdir(path) or len(name) != 15:
        return False
    try:
        time.strptime(name, SESSION_FMT)
        return True
    except ValueError:
        return False


def main(args=None):
    rclpy.init(args=args)
    node = LidarMapper()
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
