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
