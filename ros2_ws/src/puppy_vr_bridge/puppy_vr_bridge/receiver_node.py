#!/usr/bin/env python3
"""ROS2 Protocol v2 수신 노드.

Quest(Unity, PuppyPiV2Client.cs) ──UDP 5001 JSON v2──▶ 이 노드 ──▶
  /puppy_control/vr/cmd_vel  geometry_msgs/TwistStamped (SI: m/s, rad/s, REP-103)
  ──▶ 보행 제어 노드(이 저장소에 없음: 제어권 중재·watchdog) ──▶ 보행 엔진 ──▶ 서보

ACK 는 보낸 주소로 같은 소켓에서 즉시 응답한다(8필드, V2AckParser.cs 호환).
VR 입력값(forward/turn, -1~1)은 단위 없는 정규화 값이며, 이 노드가 설정된 상한으로
cm/s·rad/s 로 바꾼 뒤 TwistStamped(m/s)로 내보낸다.
"""

import json
import logging
import socket
import threading
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .protocol_v2 import SlewLimiter, StartHold, V2Session
from .v2_server import V2UdpServer

STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


class _RosLogHandler(logging.Handler):
    def __init__(self, node):
        super().__init__()
        self._node = node

    def emit(self, record):
        msg = self.format(record)
        logger = self._node.get_logger()
        if record.levelno >= logging.ERROR:
            logger.error(msg)
        elif record.levelno >= logging.WARNING:
            logger.warning(msg)
        else:
            logger.info(msg)


class VrProtocolV2Receiver(Node):
    def __init__(self):
        super().__init__("vr_protocol_v2_receiver")
        p = self.declare_parameter
        self.bind_address = p("bind_address", "0.0.0.0").value
        self.port = int(p("port", 5001).value)
        self.command_timeout = float(p("command_timeout_sec", 0.4).value)
        self.lease_timeout = float(p("client_lease_timeout_sec", 1.0).value)
        self.poll_interval = float(p("poll_interval_sec", 0.02).value)
        self.max_forward = float(p("max_forward_cm_s", 25.0).value)
        self.max_reverse = float(p("max_reverse_cm_s", 5.0).value)
        # 후진 환산 폭. 0 = max_reverse 와 같음(이전 동작). 상한은 항상 max_reverse.
        self.reverse_scale = float(p("reverse_scale_cm_s", 0.0).value)
        self.start_hold = float(p("start_hold_sec", 0.0).value)  # 0 = 끔
        self.max_yaw = float(p("max_yaw_rate_rad_s", 0.28).value)
        self.accel_lin = float(p("max_linear_accel_cm_s2", 0.0).value)  # 0 = 제한 없음(vr_bridge.yaml 설명)
        self.accel_yaw = float(p("max_yaw_accel_rad_s2", 0.0).value)
        self.publish_rate = float(p("publish_rate_hz", 20.0).value)
        self.estop_any = bool(p("accept_estop_from_any_client", True).value)
        self.output_topic = p("output_topic", "/puppy_control/vr/cmd_vel").value
        self.arbiter_topic = p("arbiter_state_topic", "/puppy_control/arbiter_state").value
        self.arbiter_timeout = float(p("arbiter_state_timeout_sec", 2.0).value)
        self.estop_topic = p("estop_request_topic", "/puppy_control/estop_request").value
        self.frame_id = p("frame_id", "base_link").value
        self.stats_log_period = float(p("stats_log_period_sec", 10.0).value)
        self.discovery_enabled = bool(p("discovery.enabled", True).value)
        self.discovery_port = int(p("discovery.port", 5002).value)
        self.discovery_name = p("discovery.name", "").value
        self._validate()

        handler = _RosLogHandler(self)
        handler.setFormatter(logging.Formatter("%(message)s"))
        for name in ("puppy_vr_bridge.v2_server",):
            lg = logging.getLogger(name)
            lg.handlers[:] = [handler]
            lg.setLevel(logging.INFO)
            lg.propagate = False

        self.pub = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.state_pub = self.create_publisher(String, "~/state", STATE_QOS)
        self.estop_pub = self.create_publisher(String, self.estop_topic, 10)
        self.create_subscription(String, self.arbiter_topic, self._on_arbiter, STATE_QOS)

        self._lock = threading.Lock()
        self._target = (0.0, 0.0)  # (x cm/s, yaw rad/s) — 세션 목표
        self._slew = SlewLimiter(self.accel_lin, self.accel_yaw)
        self._start_hold = StartHold(self.start_hold)
        self._last_arbiter_rx = None
        self._arbiter = {}
        self._last_preempt_seq = None
        self._published_nonzero = False
        self._last_pub_time = 0.0
        self._last_published = None
        self._last_stats_log = time.monotonic()
        self._last_state_pub = 0.0
        self.discovery = None  # 서버 시작 직후 상태 알림이 이 속성을 읽는다

        session = V2Session(self.max_forward, self.max_reverse, self.max_yaw,
                            self.command_timeout, self.lease_timeout,
                            accept_estop_from_any_client=self.estop_any,
                            reverse_scale=self.reverse_scale if self.reverse_scale > 0 else None)
        self.server = V2UdpServer(
            session, self._on_output, self.bind_address, self.port, self.poll_interval,
            on_estop=self._on_estop, on_state=self._on_state,
        )
        # 중재기 상태를 받기 전에는 이동을 허용하지 않는다(부팅 직후 이동 금지).
        self.server.set_motion_permitted(False, "puppy_control 상태 수신 전")
        self.server.start()

        self.discovery = None
        if self.discovery_enabled:
            try:
                from .discovery import DiscoveryServer  # VR 앱 쪽 코드에서 온 모듈이라 공개본에는 없다
                self.discovery = DiscoveryServer(
                    name=self.discovery_name or socket.gethostname()[:32],
                    control_port=self.server.bound_port, port=self.discovery_port)
                self.discovery.start()
                self.get_logger().info(
                    "VR 기기 검색 응답: UDP %d (이름 %s)" % (self.discovery.bound_port, self.discovery.name))
            except ImportError:
                self.get_logger().warning("검색 응답기 모듈 없음: 앱에 로봇 주소를 직접 입력하십시오")
                self.discovery = None
            except Exception as exc:  # 검색 실패는 제어에 영향 없음
                self.get_logger().warning("검색 응답기 시작 실패: %s" % exc)
                self.discovery = None

        self.create_timer(1.0 / 50.0, self._tick)
        self.get_logger().info(
            "Protocol v2 수신 %s:%d → %s (x≤+%.1f/-%.1f cm/s, 후진 환산 %.1f, |yaw|≤%.2f rad/s, "
            "watchdog %.2fs, lease %.1fs, 출발 대기 %.2fs)"
            % (self.bind_address, self.server.bound_port, self.output_topic, self.max_forward,
               self.max_reverse, self.reverse_scale if self.reverse_scale > 0 else self.max_reverse,
               self.max_yaw, self.command_timeout, self.lease_timeout, self.start_hold))

    def _validate(self):
        positive = dict(command_timeout_sec=self.command_timeout, client_lease_timeout_sec=self.lease_timeout,
                        max_forward_cm_s=self.max_forward, max_reverse_cm_s=self.max_reverse,
                        max_yaw_rate_rad_s=self.max_yaw, publish_rate_hz=self.publish_rate)
        for name, value in positive.items():
            if not value > 0:
                raise ValueError("%s 는 0보다 커야 합니다" % name)
        if self.lease_timeout <= self.command_timeout:
            raise ValueError("client_lease_timeout_sec 는 command_timeout_sec 보다 커야 합니다")
        # 보행 엔진이 받는 범위(|x|<=35 cm/s, |yaw|<=51°/s) 밖의 상한은 쓰지 않는다.
        if not (0.0 <= self.start_hold <= 1.0):
            raise ValueError("start_hold_sec 는 0~1 s 여야 합니다")
        if self.reverse_scale < 0 or self.reverse_scale > 35:
            raise ValueError("reverse_scale_cm_s 는 0~35 cm/s 여야 합니다")
        if self.max_forward > 35 or self.max_reverse > 35 or self.max_yaw > 0.8901:
            raise ValueError("상한이 보행 엔진 허용 범위(35 cm/s, 51°/s)를 넘습니다")

    # ── 서버 콜백 (수신 스레드) ──

    def _on_output(self, velocity, reason):
        x, _y, yaw = velocity
        with self._lock:
            self._target = (x, yaw)
            if x == 0.0 and yaw == 0.0:
                # 정지는 평활화보다 우선: 즉시 0 을 발행한다.
                self._slew.reset()
                self._start_hold.reset()
                self._publish_locked(0.0, 0.0)

    def _on_estop(self, latched: bool):
        msg = String()
        msg.data = json.dumps({"source": "vr", "latch": bool(latched)})
        self.estop_pub.publish(msg)

    def _on_state(self, snapshot):
        self._publish_state(force=True)

    # ── 중재기 상태 ──

    def _on_arbiter(self, msg: String):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self._last_arbiter_rx = time.monotonic()
        self._arbiter = state
        permitted = bool(state.get("vr_permitted", False))
        self.server.set_motion_permitted(permitted, None if permitted else state.get("vr_block_reason"))
        seq = state.get("vr_preempt_seq")
        if self._last_preempt_seq is not None and seq is not None and seq != self._last_preempt_seq:
            self.server.preempt(state.get("last_preempt_reason") or "중재기 선점")
        self._last_preempt_seq = seq
        self.server.set_external_estop(bool(state.get("estop", False)))

    # ── 주기 처리 ──

    def _tick(self):
        now = time.monotonic()
        self.server.check_timeouts()
        if self._last_arbiter_rx is not None and now - self._last_arbiter_rx > self.arbiter_timeout:
            self.server.set_motion_permitted(False, "puppy_control 상태 %.1fs 미수신" % self.arbiter_timeout)
        with self._lock:
            tx, tyaw = self._target
            hx, hyaw = self._start_hold.update(tx, tyaw, now)
            ox, oyaw = self._slew.update(hx, hyaw, now)
            changed = (ox, oyaw) != self._last_published
            due = now - self._last_pub_time >= 1.0 / self.publish_rate
            if changed or due:
                self._publish_locked(ox, oyaw)
        if now - self._last_state_pub >= 0.5:
            self._publish_state()
        if now - self._last_stats_log >= self.stats_log_period:
            self._last_stats_log = now
            stats = self.server.stats.summary(now)
            if stats["packets"]:
                self.get_logger().info(
                    "v2 통계: 수신 %d, 수락 %d, 주기 %s Hz, p95 간격 %s s, 최대 간격 %s s, 유실 추정 %d, 역순/중복 %d, 사유 %s"
                    % (stats["packets"], stats["accepted"], _fmt(stats["rate_hz"]),
                       _fmt(stats["interval_p95_s"]), _fmt(stats["interval_max_s"]),
                       stats["estimated_lost"], stats["reordered_or_duplicate"], stats["reasons"]))

    def _publish_locked(self, x_cm_s, yaw):
        if not self.context.ok():
            return  # ROS 종료 중(Ctrl-C·systemd 정지). 이때 정지는 보행 제어 쪽 watchdog 이 맡는다.
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(x_cm_s) / 100.0  # cm/s → m/s
        msg.twist.angular.z = float(yaw)
        self.pub.publish(msg)
        self._last_published = (x_cm_s, yaw)
        self._last_pub_time = time.monotonic()

    def _publish_state(self, force=False):
        snap = self.server.snapshot()
        with self._lock:
            snap["output_x_cm_s"], snap["output_yaw_rad_s"] = self._slew.output
            snap["start_hold_pending"] = self._start_hold.pending
        snap["arbiter_owner"] = self._arbiter.get("owner")
        if self.discovery is not None:
            snap["discovery_replies"] = self.discovery.replies_sent
        msg = String()
        msg.data = json.dumps(snap, ensure_ascii=False, allow_nan=False, default=str)
        self.state_pub.publish(msg)
        self._last_state_pub = time.monotonic()

    def shutdown(self):
        if self.discovery is not None:
            self.discovery.shutdown()
        self.server.shutdown()
        with self._lock:
            self._target = (0.0, 0.0)
            self._slew.reset()
            self._start_hold.reset()
            for _ in range(3):
                self._publish_locked(0.0, 0.0)


def _fmt(value):
    return "-" if value is None else "%.3f" % value


def main(args=None):
    rclpy.init(args=args)
    node = VrProtocolV2Receiver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.shutdown()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
