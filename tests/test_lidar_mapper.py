"""지도작성 핵심(puppy_lidar.mapper_core): 오도메트리 없이 가상 방을 걸어 다니며 자세·벽 위치 확인."""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "puppy_lidar"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from puppy_lidar.mapper_core import OccupancyMapper  # noqa: E402
import lidar_sim  # noqa: E402

ANG = np.arange(450) * math.radians(0.8)


def _path():
    """25 cm/s 직진 2 m(10 Hz) → 0.28 rad/s 로 90° 좌회전 → 1 m 직진. 로봇은 이 경로를 모른다."""
    poses, x, y, th = [], 0.0, 0.0, 0.0
    for _ in range(80):
        x += 0.025
        poses.append((x, y, th))
    while th < math.pi / 2:
        th = min(math.pi / 2, th + 0.028)
        poses.append((x, y, th))
    for _ in range(40):
        y += 0.025
        poses.append((x, y, th))
    return poses


def _run(sway_m=0.01, sway_rad=math.radians(0.7), noise=0.01, seed=0):
    rng = np.random.default_rng(seed)
    m = OccupancyMapper(size_m=20.0)
    errs = []
    for (x, y, th) in [(0.0, 0.0, 0.0)] + _path():
        true = (x + rng.normal(0, sway_m), y + rng.normal(0, sway_m), th + rng.normal(0, sway_rad))  # 보행 흔들림
        r = lidar_sim.raycast(true, ANG) + rng.normal(0, noise, ANG.shape)
        pts = OccupancyMapper.scan_points(r, 0.0, ANG[1], 0.12, 11.5)
        est = m.process(pts)
        errs.append((math.hypot(est[0] - true[0], est[1] - true[1]),
                     abs(math.atan2(math.sin(est[2] - true[2]), math.cos(est[2] - true[2])))))
    return m, errs, true


def test_tracks_pose_without_odometry_at_25cm_s():
    m, errs, true = _run()
    pos = [e[0] for e in errs]
    ang = [e[1] for e in errs]
    assert max(pos) < 0.10 and pos[-1] < 0.08, (max(pos), pos[-1])
    assert max(ang) < math.radians(4) and ang[-1] < math.radians(3)
    assert m.rejected_jumps == 0 and m.integrations > 20


def test_walls_land_on_true_positions():
    m, _, _ = _run(seed=3)
    occ = m.occupancy()
    ys, xs = np.nonzero(occ >= 65)
    wx = m.origin + (xs + 0.5) * m.res
    wy = m.origin + (ys + 0.5) * m.res
    # 점유 셀의 대부분이 실제 벽(선분)에서 10 cm 안
    def dist_to_walls(px, py):
        best = np.full(px.shape, np.inf)
        for (x1, y1), (x2, y2) in lidar_sim.ROOM:
            ex, ey = x2 - x1, y2 - y1
            t = np.clip(((px - x1) * ex + (py - y1) * ey) / (ex * ex + ey * ey), 0, 1)
            best = np.minimum(best, np.hypot(px - (x1 + t * ex), py - (y1 + t * ey)))
        return best
    d = dist_to_walls(wx, wy)
    assert len(d) > 300 and np.mean(d < 0.10) > 0.9, (len(d), float(np.mean(d < 0.10)))
    # 방 안쪽은 빈 공간, 방 밖은 미탐사
    cell = lambda x, y: occ[int((y - m.origin) / m.res), int((x - m.origin) / m.res)]
    assert 0 <= cell(0.5, 0.0) <= 25 and cell(-5.0, 5.0) == -1


def test_outputs_pgm_and_render():
    m, _, _ = _run(seed=5)
    pgm = m.pgm()
    assert pgm.dtype == np.uint8 and set(np.unique(pgm)) <= {0, 205, 254}
    img = m.render()
    assert img.ndim == 3 and img.shape[2] == 3 and max(img.shape[:2]) >= 300


def test_live_render_shows_only_latest_scan():
    m, _errs, _true = _run()
    img, info = m.render_live_with_info()
    assert info["display"] == "live" and img.shape[0] == info["height_px"] and img.shape[1] == info["width_px"]
    assert len(m.last_world) > 30
    # 같은 창 규칙: render_with_info 와 왼쪽 아래·축척이 같다(VR 좌표 변환이 그대로 맞음)
    _, info2 = m.render_with_info()
    for k in ("left_m", "bottom_m", "px_per_m", "width_px", "height_px"):
        assert info[k] == info2[k], k


def test_live_window_keeps_recent_scans_only():
    m = OccupancyMapper(size_m=10.0)
    ang = np.linspace(0, 2 * math.pi, 360, endpoint=False)
    near = OccupancyMapper.scan_points(np.full(360, 1.0), 0.0, ang[1], 0.12, 11.5)   # 1 m 원
    far = OccupancyMapper.scan_points(np.full(360, 2.0), 0.0, ang[1], 0.12, 11.5)    # 2 m 원
    m.process(near, now=0.0)
    m.process(far, now=5.0)
    img_new, info = m.render_live_with_info(window_s=1.0, now=5.0)    # 1 m 원은 창 밖 → 안 그림
    img_both, _ = m.render_live_with_info(window_s=10.0, now=5.0)
    def dark_at(img, x, y):
        u = int((x - info["left_m"]) * info["px_per_m"]); v = int(info["height_px"] - (y - info["bottom_m"]) * info["px_per_m"])
        return img[max(0, v - 2):v + 3, max(0, u - 2):u + 3].min() < 200
    assert dark_at(img_new, 2.0, 0.0) and not dark_at(img_new, 1.0, 0.0)
    assert dark_at(img_both, 1.0, 0.0) and dark_at(img_both, 2.0, 0.0)

