"""LD06 해석(puppy_lidar.ld06): 제조사 드라이버와 같은 CRC·각도·필터, 가상 방 왕복."""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "puppy_lidar"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from puppy_lidar import ld06  # noqa: E402
import lidar_sim  # noqa: E402


def test_crc_table_matches_vendor_values():
    # 제조사 lipkg.cpp CrcTable 의 처음·중간·끝 값(다항식 0x4D)
    assert ld06.CRC_TABLE[:8] == [0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE]
    assert ld06.CRC_TABLE[128] == 0x7C and ld06.CRC_TABLE[255] == 0xA8


def test_packet_roundtrip_and_crc_reject():
    raw = ld06.encode_packet(3600, 1000, 1880, [(1000 + i, 200) for i in range(12)], timestamp=123)
    assert len(raw) == ld06.PKT_LEN
    pkt = ld06.decode_packet(raw)
    assert pkt.speed == 3600 and pkt.start == 1000 and pkt.end == 1880 and pkt.points[11] == (1011, 200)
    bad = bytearray(raw)
    bad[10] ^= 0xFF
    assert ld06.decode_packet(bytes(bad)) is None
    ang = [a for a, _, _ in ld06.packet_points(pkt)]
    assert abs(ang[0] - 10.0) < 1e-9 and abs(ang[-1] - 18.8) < 1e-9  # 0.01° 단위, 11 등분


def test_parser_resyncs_through_garbage_and_split_chunks():
    pkts = lidar_sim.ld06_packets((0.0, 0.0, 0.0))
    stream = b"\x00\x54\x99garbage" + b"".join(pkts[:5])
    bad = bytearray(pkts[5])
    bad[20] ^= 0x55  # CRC 오류 하나
    stream += bytes(bad) + b"".join(pkts[6:])
    p = ld06.Parser()
    got = []
    rng = np.random.default_rng(1)
    i = 0
    while i < len(stream):
        n = int(rng.integers(1, 90))
        got += p.feed(stream[i:i + n])
        i += n
    assert len(got) == len(pkts) - 1 and p.crc_errors >= 1 and p.resyncs >= 1


def _scan(pose, rng=None, noise=0.0):
    asm = ld06.ScanAssembler()
    p = ld06.Parser()
    out = None
    scans = []
    for raw in lidar_sim.ld06_packets(pose, rng=rng, noise_m=noise, turns=3):
        for pkt in p.feed(raw):
            s = asm.add(pkt)
            if s is not None:
                scans.append(s)
    return scans[1]  # 첫 스캔은 시작 위치에 따라 일부만, 두 번째부터 완전한 한 바퀴


def test_assembled_scan_matches_ground_truth_and_ros_direction():
    scan = _scan((0.0, 0.0, 0.0))
    r = np.array(scan["ranges"])
    inc = scan["angle_increment"]
    assert len(r) == 450 and abs(inc - math.radians(0.8)) < 1e-12
    th = np.arange(len(r)) * inc
    truth = lidar_sim.raycast((0.0, 0.0, 0.0), th)
    ok = np.isfinite(r)
    assert ok.mean() > 0.95
    # 빔 각도 양자화(0.8°) 만큼의 오차만 허용
    err = np.abs(r[ok] - truth[ok])
    assert np.median(err) < 0.02 and np.percentile(err, 95) < 0.08
    # 방향: ROS +90°(왼쪽, +y) = 벽 y=+2 → 2 m, -90°(오른쪽) = 벽 y=-2 → 2 m, 앞(+x) = 4 m, 뒤 = 2 m
    at = lambda deg: r[int(round(math.radians(deg % 360) / inc)) % len(r)]
    assert abs(at(0) - 4.0) < 0.05 and abs(at(180) - 2.0) < 0.05
    assert abs(at(90) - 2.0) < 0.05 and abs(at(270) - 2.0) < 0.05
    # 상자(1.0~1.6, 0.8~1.3)는 왼쪽 앞(+x, +y) 에 보여야 한다(시계/반시계를 뒤집으면 오른쪽에 나타남)
    left_front = r[int(math.radians(45) / inc)]
    right_front = r[int(math.radians(315) / inc)]
    assert left_front < 2.0 < right_front


def test_self_filter_and_near_filter():
    asm = ld06.ScanAssembler()
    # 3 m 벽(빽빽한 점, 신뢰 200). 몸·다리가 가린 각도(앞 0.10 m, 옆 0.09 m 묶음)는 그 점만 돌아온다(각도당 한 점)
    body = {i: 100 for i in range(0, 5)}
    body.update({i: 90 for i in range(112, 117)})
    asm._pts = [(0.8 * i, body.get(i, 3000), 230 if i in body else 200) for i in range(450)]
    asm._speed = 3600
    scan = asm._finish()
    r = np.array(scan["ranges"])
    assert np.nanmin(r) >= 2.9 and np.isfinite(r).sum() == 440  # 다리 범위 안 10점만 제거, 벽은 남음
    # 저신뢰 외톨이 점은 near_filter 가 버림, 긴 묶음은 남김
    lone = [(50.0, 1000, 10)]
    group = [(100.0 + 0.8 * i, 2000, 200) for i in range(20)]
    kept = ld06.near_filter(lone + group, 3600)
    assert (50.0, 1000, 10) not in kept and len(kept) == 20
