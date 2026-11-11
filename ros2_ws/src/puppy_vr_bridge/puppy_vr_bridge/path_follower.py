"""경로 지정 주행(2026-11-05, ROS 비의존·가짜 시계로 시험).

VR 에서 LiDAR 지도 위에 찍은 경유점(지도 좌표 m)을 '제자리 회전 → 직진' 으로 차례로 따라간다.
- 4족 보행은 '느린 전진 + 회전' 에서 넘어지기 쉬워(실기 관찰) 회전은 제자리에서만, 직진 중에는 작은 방향 보정만 한다.
- 장애물 판단·경로 취소·우회는 VR(MR) 앱이 맡는다(2026-11-11 MR 팀 요청). 로봇은 LiDAR 점만으로 멈추지 않는다.
  MR 이 장애물을 확정하면 path_cancel 을 보낸다.
- 로봇 위치(/lidar_map/pose)나 LiDAR(/scan)가 timeout 넘게 끊기면 회전·직진 모두 멈춘다(경로는 유지).
- 실행 식별자: 실제로 적용한 path 패킷의 client_id·sequence 를 request_client_id·request_sequence 로 보관한다.
  done·cancelled 뒤에도 유지하고, 다음 경로가 실제로 적용될 때만 바뀐다.
- 로봇 위치가 한 번에 jump_cancel_m / jump_cancel_deg 넘게 바뀌면(지도 초기화·위치 추정 튐) 경로를 취소한다.
  경유점은 이전 지도 좌표라 그대로 따라가면 엉뚱한 곳으로 간다. (조이스틱 조종에는 영향 없음)
- 출력: (x cm/s, yaw rad/s). 안전 규칙(조종자·watchdog·비상정지)은 수신기(receiver_node)가 따로 지킨다.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

ACTIVE = ("rotate", "drive")


@dataclass
class FollowerConfig:
    speed_cm_s: float = 12.5      # 조이스틱 속도 50 % 와 같게
    yaw_rate: float = 0.14        # rad/s, 제자리 회전(조이스틱 50 % 와 같게)
    rot_enter_deg: float = 15.0   # 직진 중 방향 오차가 이보다 크면 멈추고 다시 회전
    rot_exit_deg: float = 6.0     # 회전 중 이보다 작아지면 직진
    heading_kp: float = 0.5       # 직진 중 방향 보정(rad/s per rad)
    heading_max: float = 0.05     # rad/s
    arrive_m: float = 0.10        # 경유점 도착 반경
    pass_m: float = 0.30          # 경유점을 지나쳤을 때 이 거리 안이면 도착으로 본다
    pose_timeout_sec: float = 0.6
    scan_timeout_sec: float = 0.6
    jump_cancel_m: float = 0.25     # 위치 한 번 갱신(0.1 s)에 정상 이동은 약 1 cm
    jump_cancel_deg: float = 25.0


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def scan_to_points(ranges, angle_min, angle_inc, range_min, range_max, laser=(0.0, 0.0, 0.0)):
    """LaserScan → 로봇 기준 (x 앞, y 왼쪽) 점 목록."""
    lx, ly, lyaw = laser
    pts = []
    for i, r in enumerate(ranges):
        if not (range_min <= r <= range_max) or not math.isfinite(r):
            continue
        a = angle_min + i * angle_inc + lyaw
        pts.append((lx + r * math.cos(a), ly + r * math.sin(a)))
    return pts


@dataclass
class PathFollower:
    cfg: FollowerConfig = field(default_factory=FollowerConfig)
    state: str = "idle"          # idle | rotate | drive | done | cancelled ("blocked" 는 더 이상 내지 않음)
    points: List[Tuple[float, float]] = field(default_factory=list)
    index: int = 0
    owner: Optional[str] = None
    reason: Optional[str] = None
    waiting: Optional[str] = None  # None | "pose" | "scan" — 정보가 끊겨 잠시 멈춤
    blocked_count: int = 0       # 상태 형식 호환용. 로봇이 장애물로 멈추지 않으므로 늘 0
    started: float = 0.0
    request_client_id: Optional[str] = None  # 이 경로를 보낸 조종자 UUID(패킷 그대로)
    request_sequence: Optional[int] = None   # 이 경로 path 패킷의 sequence
    _seg_start: Optional[Tuple[float, float]] = None
    _last_pose: Optional[Tuple[float, float, float]] = None
    _last_pose_t: Optional[float] = None

    @property
    def active(self) -> bool:
        return self.state in ACTIVE

    def set_path(self, points, owner, now, request_client_id=None, request_sequence=None):
        self.points = [(float(x), float(y)) for x, y in points]
        self.index, self.owner, self.reason, self.waiting = 0, owner, None, None
        self.blocked_count, self.started = 0, now
        # 두 값이 모두 있어야 기록한다(한쪽만 채우지 않음). 식별자 없는 호출은 둘 다 None.
        ok = type(request_client_id) is str and type(request_sequence) is int
        self.request_client_id = request_client_id if ok else None
        self.request_sequence = request_sequence if ok else None
        self.state = "rotate" if self.points else "done"
        self._seg_start = None
        self._last_pose, self._last_pose_t = None, None

    def cancel(self, reason: str):
        if self.active:
            self.state, self.reason, self.waiting = "cancelled", reason, None

    def _reached(self, x, y) -> bool:
        tx, ty = self.points[self.index]
        d = math.hypot(tx - x, ty - y)
        last = self.index == len(self.points) - 1
        if d <= self.cfg.arrive_m:
            return True
        if self._seg_start is None:
            return False
        sx, sy = self._seg_start
        vx, vy = tx - sx, ty - sy
        seg = math.hypot(vx, vy)
        if seg < 1e-6:
            return False
        t = ((x - sx) * vx + (y - sy) * vy) / (seg * seg)  # 구간 위 진행 비율
        return t >= 1.0 and d <= (self.cfg.arrive_m * 1.5 if last else self.cfg.pass_m)

    def update(self, now, pose, pose_t, scan_pts, scan_t) -> Tuple[float, float]:
        if not self.active:
            return 0.0, 0.0
        c = self.cfg
        if pose is None or now - pose_t > c.pose_timeout_sec:
            self.waiting = "pose"
            return 0.0, 0.0
        x, y, th = pose
        if self._last_pose_t is not None and pose_t != self._last_pose_t and self._last_pose is not None:
            lx, ly, lth = self._last_pose
            if (math.hypot(x - lx, y - ly) > c.jump_cancel_m
                    or abs(wrap(th - lth)) > math.radians(c.jump_cancel_deg)):
                self.state, self.waiting = "cancelled", None
                self.reason = "위치 급변(지도 초기화·위치 추정 튐)"
                return 0.0, 0.0
        self._last_pose, self._last_pose_t = pose, pose_t
        if self._seg_start is None:
            self._seg_start = (x, y)
        while self.index < len(self.points) and self._reached(x, y):
            self._seg_start = self.points[self.index]
            self.index += 1
            self.state = "rotate"
        if self.index >= len(self.points):
            self.state, self.waiting = "done", None
            return 0.0, 0.0
        # LiDAR 가 끊기면 MR 이 장애물을 볼 수 없으므로 회전·직진 모두 멈춘다(첫 회전 전에도). 점 내용은 보지 않는다.
        if scan_pts is None or now - scan_t > c.scan_timeout_sec:
            self.waiting = "scan"
            return 0.0, 0.0
        self.waiting = None
        tx, ty = self.points[self.index]
        err = wrap(math.atan2(ty - y, tx - x) - th)
        if self.state == "rotate":
            if abs(err) > math.radians(c.rot_exit_deg):
                return 0.0, math.copysign(c.yaw_rate, err)
            self.state = "drive"
        if abs(err) > math.radians(c.rot_enter_deg):
            self.state = "rotate"
            return 0.0, math.copysign(c.yaw_rate, err)
        yaw = max(-c.heading_max, min(c.heading_max, c.heading_kp * err))
        return c.speed_cm_s, yaw

    def status(self, now, pose=None) -> dict:
        remaining = None
        if pose is not None and self.points and self.index < len(self.points):
            px, py = pose[0], pose[1]
            remaining = 0.0
            for tx, ty in self.points[self.index:]:
                remaining += math.hypot(tx - px, ty - py)
                px, py = tx, ty
        return {
            "state": self.state, "waiting": self.waiting, "reason": self.reason,
            "request_client_id": self.request_client_id, "request_sequence": self.request_sequence,
            "points": [[round(x, 3), round(y, 3)] for x, y in self.points],
            "index": self.index, "count": len(self.points),
            "remaining_m": None if remaining is None else round(remaining, 3),
            "blocked_count": self.blocked_count,
            "elapsed_s": round(now - self.started, 2) if self.started else None,
        }
