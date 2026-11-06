"""경로 지정 주행(2026-11-05, ROS 비의존·가짜 시계로 시험).

VR 에서 LiDAR 지도 위에 찍은 경유점(지도 좌표 m)을 '제자리 회전 → 직진' 으로 차례로 따라간다.
- 4족 보행은 '느린 전진 + 회전' 에서 넘어지기 쉬워(실기 관찰) 회전은 제자리에서만, 직진 중에는 작은 방향 보정만 한다.
- 직진 중 몸 앞 stop_distance_m 안(폭 ±half_width_m)에 LiDAR 점이 있으면 멈추고, clear_hold_sec 동안 비어 있으면 다시 간다.
- 로봇 위치(/lidar_map/pose)나 LiDAR(/scan)가 timeout 넘게 끊기면 멈춘다(경로는 유지).
- 로봇 위치가 한 번에 jump_cancel_m / jump_cancel_deg 넘게 바뀌면(지도 초기화·위치 추정 튐) 경로를 취소한다.
  경유점은 이전 지도 좌표라 그대로 따라가면 엉뚱한 곳으로 간다. (조이스틱 조종에는 영향 없음)
- 출력: (x cm/s, yaw rad/s). 안전 규칙(조종자·watchdog·비상정지)은 수신기(receiver_node)가 따로 지킨다.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

ACTIVE = ("rotate", "drive", "blocked")


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
    stop_distance_m: float = 0.30  # 몸 앞 끝에서 이 거리 안의 장애물에 멈춤
    body_front_m: float = 0.16    # 로봇 중심에서 몸 앞 끝까지
    half_width_m: float = 0.17    # 장애물 검사 폭의 절반(몸 + 여유)
    min_obstacle_points: int = 2
    clear_hold_sec: float = 1.0
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


def obstacle_count(points: Sequence[Tuple[float, float]], cfg: FollowerConfig) -> int:
    x0, x1, hw = cfg.body_front_m, cfg.body_front_m + cfg.stop_distance_m, cfg.half_width_m
    return sum(1 for x, y in points if x0 < x < x1 and -hw < y < hw)


@dataclass
class PathFollower:
    cfg: FollowerConfig = field(default_factory=FollowerConfig)
    state: str = "idle"          # idle | rotate | drive | blocked | done | cancelled
    points: List[Tuple[float, float]] = field(default_factory=list)
    index: int = 0
    owner: Optional[str] = None
    reason: Optional[str] = None
    waiting: Optional[str] = None  # None | "pose" | "scan" — 정보가 끊겨 잠시 멈춤
    blocked_count: int = 0
    started: float = 0.0
    _seg_start: Optional[Tuple[float, float]] = None
    _clear_since: Optional[float] = None
    _last_pose: Optional[Tuple[float, float, float]] = None
    _last_pose_t: Optional[float] = None

    @property
    def active(self) -> bool:
        return self.state in ACTIVE

    def set_path(self, points, owner, now):
        self.points = [(float(x), float(y)) for x, y in points]
        self.index, self.owner, self.reason, self.waiting = 0, owner, None, None
        self.blocked_count, self.started = 0, now
        self.state = "rotate" if self.points else "done"
        self._seg_start, self._clear_since = None, None
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
        tx, ty = self.points[self.index]
        err = wrap(math.atan2(ty - y, tx - x) - th)
        if self.state == "rotate":
            if abs(err) > math.radians(c.rot_exit_deg):
                self.waiting = None
                return 0.0, math.copysign(c.yaw_rate, err)
            self.state = "drive"
        if abs(err) > math.radians(c.rot_enter_deg):
            self.state, self.waiting = "rotate", None
            return 0.0, math.copysign(c.yaw_rate, err)
        if scan_pts is None or now - scan_t > c.scan_timeout_sec:
            self.waiting = "scan"
            return 0.0, 0.0
        self.waiting = None
        if obstacle_count(scan_pts, c) >= c.min_obstacle_points:
            if self.state != "blocked":
                self.blocked_count += 1
            self.state, self._clear_since = "blocked", None
            return 0.0, 0.0
        if self.state == "blocked":
            if self._clear_since is None:
                self._clear_since = now
            if now - self._clear_since < c.clear_hold_sec:
                return 0.0, 0.0
            self.state = "drive"
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
            "points": [[round(x, 3), round(y, 3)] for x, y in self.points],
            "index": self.index, "count": len(self.points),
            "remaining_m": None if remaining is None else round(remaining, 3),
            "blocked_count": self.blocked_count,
            "elapsed_s": round(now - self.started, 2) if self.started else None,
        }
