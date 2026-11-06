"""경로 지정 주행(puppy_vr_bridge.path_follower) 시험: 간단한 운동 모사기로 끝까지 따라가 본다(ROS 없이).

모사기: 50 Hz 로 (x cm/s, yaw rad/s) 를 적분, 위치·LiDAR 는 10 Hz 로 갱신. 장애물은 지도 좌표 점 묶음.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "puppy_vr_bridge"))
from puppy_vr_bridge.path_follower import FollowerConfig, PathFollower, obstacle_count, scan_to_points, wrap  # noqa: E402


def box(cx, cy, half=0.05, step=0.02):
    pts = []
    n = int(2 * half / step) + 1
    for i in range(n):
        for j in range(n):
            pts.append((cx - half + i * step, cy - half + j * step))
    return pts


def to_robot(world, pose, max_range=8.0):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    out = []
    for wx, wy in world:
        dx, dy = wx - x, wy - y
        rx, ry = c * dx + s * dy, -s * dx + c * dy
        if math.hypot(rx, ry) <= max_range:
            out.append((rx, ry))
    return out


def simulate(path, start=(0.0, 0.0, 0.0), obstacles=lambda t: (), T=90.0, dt=0.02, sensor_dt=0.1, cfg=None,
             pose_ok=lambda t: True, scan_ok=lambda t: True):
    f = PathFollower(cfg or FollowerConfig())
    f.set_path(path, "owner", 0.0)
    x, y, th = start
    pose = scan = None
    pose_t = scan_t = -10.0
    log, t = [], 0.0
    while t < T and f.active:
        if t - max(pose_t, scan_t, -1.0) >= sensor_dt - 1e-9 or pose is None:
            if pose_ok(t):
                pose, pose_t = (x, y, th), t
            if scan_ok(t):
                scan, scan_t = to_robot(obstacles(t), (x, y, th)), t
        vx, w = f.update(t, pose, pose_t, scan, scan_t)
        log.append((t, vx, w, f.state, (x, y, th)))
        x += vx / 100.0 * math.cos(th) * dt
        y += vx / 100.0 * math.sin(th) * dt
        th = wrap(th + w * dt)
        t += dt
    return f, (x, y, th), log


def test_l_path_reaches_goal_and_turns_only_in_place():
    f, (x, y, _), log = simulate([(1.0, 0.0), (1.0, 0.8)])
    assert f.state == "done"
    assert math.hypot(x - 1.0, y - 0.8) < 0.15
    # 큰 회전(제자리 회전 속도)은 전진 0 일 때만 — '느린 전진 + 회전' 조합이 없다
    assert all(vx == 0.0 for _, vx, w, _, _ in log if abs(w) > 0.06)
    assert all(abs(w) <= 0.05 + 1e-9 for _, vx, w, _, _ in log if vx > 0)
    t_done = log[-1][0]
    assert 10.0 < t_done < 40.0   # 1.8 m ÷ 12.5 cm/s ≈ 14 s + 90° 회전 ≈ 11 s


def test_starts_by_rotating_when_facing_away():
    f, _, log = simulate([(1.0, 0.0)], start=(0.0, 0.0, math.pi))
    first_drive = next(i for i, r in enumerate(log) if r[1] > 0)
    assert all(r[1] == 0.0 and abs(r[2]) > 0 for r in log[:first_drive])
    assert f.state == "done"


def test_obstacle_stops_then_resumes_after_clear():
    # 1.2 m 직진 경로 한가운데(0.7 m)에 상자. 20 s 에 치운다.
    obs = lambda t: box(0.7, 0.0) if t < 20.0 else ()
    f, (x, y, _), log = simulate([(1.2, 0.0)], obstacles=obs)
    assert f.state == "done" and f.blocked_count >= 1
    blocked = [r for r in log if r[3] == "blocked"]
    assert blocked and all(r[1] == 0.0 and r[2] == 0.0 for r in blocked)
    # 멈춘 위치: 상자 앞면(0.65 m)까지 몸 앞 끝(+0.16 m)이 닿지 않았다
    stop_x = max(r[4][0] for r in log if r[0] < 20.0)
    assert stop_x + 0.16 < 0.65
    assert stop_x + 0.16 > 0.65 - 0.30 - 0.05   # 너무 일찍 서지도 않음(정지 거리 0.30 m + 한 주기 여유)
    # 치운 뒤 clear_hold(1 s) 지나서 다시 출발
    resume = next(r[0] for r in log if r[0] >= 20.0 and r[1] > 0)
    assert 20.9 <= resume <= 21.5


def test_side_obstacle_outside_width_does_not_block():
    obs = lambda t: box(0.6, 0.35)   # 경로 옆 35 cm
    f, _, log = simulate([(1.2, 0.0)], obstacles=obs)
    assert f.state == "done" and f.blocked_count == 0


def test_pose_or_scan_loss_stops_but_keeps_path():
    f, _, log = simulate([(1.5, 0.0)], pose_ok=lambda t: not (5.0 <= t < 8.0), T=40.0)
    lost = [r for r in log if 5.8 < r[0] < 8.0]
    assert lost and all(r[1] == 0.0 and r[2] == 0.0 for r in lost)
    assert f.state == "done"
    f2, _, log2 = simulate([(1.5, 0.0)], scan_ok=lambda t: not (5.0 <= t < 8.0), T=40.0)
    lost2 = [r for r in log2 if 5.8 < r[0] < 8.0]
    assert lost2 and all(r[1] == 0.0 for r in lost2) and f2.state == "done"


def test_cancel_and_empty_path():
    f = PathFollower()
    f.set_path([(1.0, 0.0)], "o", 0.0)
    assert f.active
    f.cancel("VR 취소")
    assert not f.active and f.state == "cancelled" and f.update(0.1, (0, 0, 0), 0.1, [], 0.1) == (0.0, 0.0)
    f.set_path([], "o", 0.0)
    assert f.state == "done"


def test_scan_to_points_and_obstacle_box():
    pts = scan_to_points([0.3, float("inf"), 0.05, 1.0], -math.pi / 2, math.pi / 2, 0.12, 8.0)
    assert len(pts) == 2   # inf·최소 거리 미만 제외
    cfg = FollowerConfig()
    assert obstacle_count([(0.30, 0.0), (0.40, 0.1)], cfg) == 2
    assert obstacle_count([(0.10, 0.0), (0.50, 0.0), (0.30, 0.25)], cfg) == 0   # 몸 안·멀리·옆


def test_status_reports_progress():
    f = PathFollower()
    f.set_path([(1.0, 0.0), (1.0, 1.0)], "o", 0.0)
    st = f.status(1.0, (0.0, 0.0, 0.0))
    assert st["state"] == "rotate" and st["count"] == 2 and abs(st["remaining_m"] - 2.0) < 1e-6


def test_pose_jump_cancels_path():
    # 5 s 에 지도 초기화 흉내: 0.1 s 마다 새 위치를 내다가, 5 s 부터 보고 위치가 (0, 0, 0) 으로 튄다
    f = PathFollower()
    f.set_path([(2.0, 0.0)], "o", 0.0)
    x, pose, pose_t, t = 0.0, None, -1.0, 0.0
    for i in range(400):
        t = i * 0.02
        if t - pose_t >= 0.1 - 1e-9:
            pose, pose_t = ((x, 0.0, 0.0) if t < 5.0 else (0.0, 0.0, 0.0)), t
        vx, _ = f.update(t, pose, pose_t, [], pose_t)
        x += vx / 100.0 * 0.02
        if not f.active:
            break
    assert f.state == "cancelled" and "위치 급변" in f.reason
    assert 4.9 < t < 5.2


def test_normal_motion_is_not_a_jump():
    f, _, log = simulate([(1.0, 0.0), (1.0, 0.8), (0.0, 0.8)])
    assert f.state == "done"

