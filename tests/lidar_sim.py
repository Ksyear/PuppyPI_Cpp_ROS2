"""LiDAR 시험용 가상 방: 벽(선분) 광선 추적 + LD06 패킷 생성(제조사 형식·시계 방향 각도)."""
import math

import numpy as np

# 6 m × 4 m 방 + 상자 2개(특징). 시작 자세 (0,0,0) 이 지도 원점.
ROOM = [((-2, -2), (4, -2)), ((4, -2), (4, 2)), ((4, 2), (-2, 2)), ((-2, 2), (-2, -2)),
        ((1.0, 0.8), (1.6, 0.8)), ((1.6, 0.8), (1.6, 1.3)), ((1.6, 1.3), (1.0, 1.3)), ((1.0, 1.3), (1.0, 0.8)),
        ((2.5, -1.2), (3.0, -0.6))]


def raycast(pose, angles, walls=ROOM, max_range=12.0):
    """pose(x,y,θ) 에서 로봇 기준 각도들(반시계, rad)로 쏜 거리(m). 안 맞으면 inf."""
    x, y, th = pose
    a = th + np.asarray(angles)
    dx, dy = np.cos(a), np.sin(a)
    best = np.full(a.shape, np.inf)
    for (x1, y1), (x2, y2) in walls:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x1 - x) * ey - (y1 - y) * ex) / den
            u = ((x1 - x) * dy - (y1 - y) * dx) / den
        ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(ok & (t < best), t, best)
    best[best > max_range] = np.inf
    return best


def ld06_packets(pose, speed=3600, rng=None, noise_m=0.0, turns=1):
    """turns 바퀴를 끊김 없는 LD06 패킷 흐름으로(시계 방향, 패킷당 12점, 0°를 넘어가는 패킷 포함, 0.01° 단위)."""
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                    "ros2_ws", "src", "puppy_lidar"))
    from puppy_lidar import ld06
    step = speed / 4500.0  # °/점
    n = int(round(360.0 / step)) * turns
    n -= n % 12
    cw = (np.arange(n) * step) % 360.0  # LD06 각도(시계 방향)
    ranges = raycast(pose, np.radians(-cw))  # 시계 방향 θ = ROS -θ
    if rng is not None and noise_m:
        ranges = ranges + rng.normal(0, noise_m, ranges.shape)
    pkts = []
    for i in range(0, n, 12):
        pts = []
        for j in range(12):
            r = ranges[i + j]
            d = 0 if not np.isfinite(r) else int(round(r * 1000))
            pts.append((min(d, 65535), 200 if d else 0))
        start = int(round(cw[i] * 100)) % 36000
        end = int(round(cw[i + 11] * 100)) % 36000
        pkts.append(ld06.encode_packet(speed, start, end, pts))
    return pkts
