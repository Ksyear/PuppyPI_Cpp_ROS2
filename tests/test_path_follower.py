"""경로 지정 주행(puppy_vr_bridge.path_follower) 시험: 간단한 운동 모사기로 끝까지 따라가 본다(ROS 없이).

모사기: 50 Hz 로 (x cm/s, yaw rad/s) 를 적분, 위치·LiDAR 는 10 Hz 로 갱신. 장애물은 지도 좌표 점 묶음.
2026-11-11: 장애물 판단은 VR(MR) 이 맡는다. 로봇은 LiDAR 점만으로 멈추지 않는다(MR 팀 작업 지시서 3절).
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "puppy_vr_bridge"))
from puppy_vr_bridge.path_follower import FollowerConfig, PathFollower, scan_to_points, wrap  # noqa: E402


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
             pose_ok=lambda t: True, scan_ok=lambda t: True, request=(None, None), cancel_at=None):
    f = PathFollower(cfg or FollowerConfig())
    f.set_path(path, "owner", 0.0, *request)
    x, y, th = start
    pose = scan = None
    pose_t = scan_t = -10.0
    log, t = [], 0.0
    while t < T and f.active:
        if cancel_at is not None and t >= cancel_at:
            f.cancel("VR 취소")   # MR 이 장애물을 확정하고 path_cancel 을 보낸 경우
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


def test_repeated_front_noise_does_not_stop_or_cancel():
    # 0.8 s 마다 0.1 s 동안 몸 바로 앞(0.25·0.30 m)에 잡음 점 2개. 예전에는 이 조건으로 blocked → 1 s 대기를 반복했다.
    noise = lambda t: ((0.25 + t * 0.125, 0.0), (0.30 + t * 0.125, 0.02)) if (t % 0.8) < 0.1 else ()
    f0, _, base = simulate([(1.2, 0.0)])
    f, (x, y, _), log = simulate([(1.2, 0.0)], obstacles=noise)
    assert f.state == "done" and f.reason is None and f.blocked_count == 0
    assert not any(r[3] == "blocked" for r in log)
    assert [r[1:3] for r in log] == [r[1:3] for r in base]   # 잡음이 없을 때와 출력이 같다
    assert all(r[1] > 0 for r in log if r[3] == "drive")


def test_continuous_obstacle_points_do_not_stop_until_mr_cancels():
    # 경로 한가운데 상자(계속 보임). 로봇은 스스로 서지 않고, MR 의 path_cancel(3 s)에서만 멈춘다.
    obs = lambda t: box(0.7, 0.0)
    f, _, log = simulate([(1.2, 0.0)], obstacles=obs, cancel_at=3.0, request=("cid", 7))
    assert all(r[1] > 0 for r in log if r[3] == "drive")
    assert any(r[3] == "drive" for r in log)
    assert f.state == "cancelled" and f.reason == "VR 취소"
    assert f.update(3.1, (0.3, 0, 0), 3.1, [], 3.1) == (0.0, 0.0)
    st = f.status(3.1)
    assert (st["request_client_id"], st["request_sequence"]) == ("cid", 7)   # 취소해도 실행 식별자 유지


def test_scan_stale_blocks_first_rotation_too():
    # 뒤를 보고 시작(먼저 제자리 회전). LiDAR 가 한 번도 안 왔거나 0.6 s 넘게 끊기면 회전 출력도 0.
    f = PathFollower()
    f.set_path([(1.0, 0.0)], "o", 0.0)
    assert f.update(0.1, (0.0, 0.0, math.pi), 0.1, None, 0.0) == (0.0, 0.0) and f.waiting == "scan"
    assert f.update(0.9, (0.0, 0.0, math.pi), 0.9, [], 0.2) == (0.0, 0.0) and f.waiting == "scan"
    vx, w = f.update(1.0, (0.0, 0.0, math.pi), 1.0, [], 1.0)    # 스캔이 다시 오면 별도 RESET 없이 바로 회전
    assert vx == 0.0 and abs(w) > 0 and f.waiting is None and f.state == "rotate"


def test_scan_loss_during_rotation_stops_rotation():
    f, _, log = simulate([(1.0, 0.0)], start=(0.0, 0.0, math.pi), scan_ok=lambda t: not (2.0 <= t < 4.0), T=60.0)
    lost = [r for r in log if 2.75 < r[0] < 4.0]
    assert lost and all(r[1] == 0.0 and r[2] == 0.0 for r in lost)
    assert f.state == "done" and f.reason is None


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


def test_scan_to_points():
    pts = scan_to_points([0.3, float("inf"), 0.05, 1.0], -math.pi / 2, math.pi / 2, 0.12, 8.0)
    assert len(pts) == 2   # inf·최소 거리 미만 제외


def test_no_obstacle_settings_left():
    names = set(FollowerConfig.__dataclass_fields__)
    assert not names & {"stop_distance_m", "body_front_m", "half_width_m", "min_obstacle_points", "clear_hold_sec"}


def test_status_reports_progress():
    f = PathFollower()
    f.set_path([(1.0, 0.0), (1.0, 1.0)], "o", 0.0)
    st = f.status(1.0, (0.0, 0.0, 0.0))
    assert st["state"] == "rotate" and st["count"] == 2 and abs(st["remaining_m"] - 2.0) < 1e-6


CID_A = "a92e1842-7ef1-4d1f-bc14-2a04dcd0da87"


def test_request_identity_rules():
    f = PathFollower()
    st = f.status(0.0)
    assert st["request_client_id"] is None and st["request_sequence"] is None   # 시작 전: 둘 다 null
    f.set_path([(1.0, 0.0)], "o", 0.0, CID_A, 142)
    for state in ("rotate", "drive", "cancelled"):
        f.state = state
        st = f.status(0.1)
        assert (st["state"], st["request_client_id"], st["request_sequence"]) == (state, CID_A, 142)
    f.state = "rotate"
    f.cancel("VR 취소")
    assert (f.request_client_id, f.request_sequence) == (CID_A, 142)
    f.set_path([(1.0, 0.0)], "o", 0.0, CID_A, 150)                # 다음 경로가 실제 적용될 때만 바뀜
    assert f.request_sequence == 150
    for cid, seq in ((None, 151), (CID_A, None), (CID_A, True), (CID_A, 1.0), (CID_A, "152"), (5, 152)):
        f.set_path([(1.0, 0.0)], "o", 0.0, cid, seq)            # 한쪽만·잘못된 형식 → 둘 다 null
        assert f.request_client_id is None and f.request_sequence is None, (cid, seq)
    f.set_path([(1.0, 0.0)], "o", 0.0)                          # 식별자 없는 이전 호출 방식
    assert f.request_client_id is None and f.request_sequence is None


def test_short_path_done_keeps_request_identity():
    # 12 cm 목표, 도착 반경 10 cm: 첫 상태 조회 전에 끝나도 이번 요청의 ID 를 가진 done 이 남는다.
    f, (x, _, _), log = simulate([(0.0, 0.0), (0.12, 0.0)], request=(CID_A, 142))
    st = f.status(log[-1][0] + 1.0)
    assert st["state"] == "done" and st["index"] == st["count"] == 2
    assert (st["request_client_id"], st["request_sequence"]) == (CID_A, 142)
    assert log[-1][0] < 1.0 and 0.02 <= x < 0.13


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

