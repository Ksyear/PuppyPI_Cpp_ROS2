"""LDROBOT LD06 시리얼 프로토콜 해석(ROS 비의존, 단독 시험 가능).

LD06 공개 프로토콜로 새로 작성했다. LiDAR 제조사(LDROBOT) 드라이버의 코드는 복사하지 않았고,
아래 필터 기준값만 그 드라이버의 동작에 맞췄다:
  - 47 B 패킷: 0x54, 0x2C, speed(u16, °/s), start_angle(u16, 0.01°), 12 × (distance u16 mm, confidence u8),
    end_angle(u16), timestamp(u16 ms), crc8   (모두 little-endian)
  - CRC8: 다항식 0x4D (MSB 우선, 초기값 0) 표 방식
  - 각도 폭이 speed 로 설명되지 않는 패킷은 버림, 각도가 350° 이상 되돌아가면 한 바퀴 완료
  - 근거리(5 m 미만) 점을 각도·거리 연속성으로 묶어 점 2개 이하 저신뢰(평균 < 220) 묶음과
    점 3~15개 저신뢰(평균 ≤ 15) 묶음을 버림
  - LD06 은 시계 방향이라 ROS(반시계)로 바꿀 때 빔 순서를 뒤집는다: ros_index = N-1-index
추가: 로봇 몸·다리에 맞은 점 제거(팀 실측 다리 범위 x ±0.14, y ±0.085 m + 여유 0.02 m).
"""

import math
import struct

HEADER = 0x54
VER_LEN = 0x2C
PKT_LEN = 47
POINTS_PER_PACK = 12
SAMPLE_RATE = 4500.0  # 점/s
CONFIDENCE_LOW = 15
CONFIDENCE_SINGLE = 220


def _crc_table(poly=0x4D):
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return table


CRC_TABLE = _crc_table()


def crc8(data):
    c = 0
    for b in data:
        c = CRC_TABLE[(c ^ b) & 0xFF]
    return c


class Packet(object):
    __slots__ = ("speed", "start", "end", "timestamp", "points")

    def __init__(self, speed, start, end, timestamp, points):
        self.speed, self.start, self.end, self.timestamp, self.points = speed, start, end, timestamp, points


def decode_packet(buf):
    """정확히 47 B → Packet 또는 None(CRC·형식 오류)."""
    if len(buf) != PKT_LEN or buf[0] != HEADER or buf[1] not in (VER_LEN, VER_LEN | (0x07 << 5)):
        return None
    if crc8(buf[:-1]) != buf[-1]:
        return None
    speed, start = struct.unpack_from("<HH", buf, 2)
    points = [struct.unpack_from("<HB", buf, 6 + 3 * i) for i in range(POINTS_PER_PACK)]
    end, ts = struct.unpack_from("<HH", buf, 6 + 3 * POINTS_PER_PACK)
    return Packet(speed, start, end, ts, points)


def encode_packet(speed, start, end, points, timestamp=0):
    """시험·모의 스트림용: Packet 을 바이트로(제조사 형식)."""
    body = struct.pack("<BBHH", HEADER, VER_LEN, speed, start)
    for d, c in points:
        body += struct.pack("<HB", d, c)
    body += struct.pack("<HH", end, timestamp)
    return body + bytes([crc8(body)])


class Parser(object):
    """바이트 흐름 → Packet. 헤더로 재동기화, CRC 오류면 헤더 2 B 만 버리고 다시 찾는다(제조사와 같음)."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.resyncs = 0
        self.angle_errors = 0
        self.packets = 0

    def feed(self, data):
        self.buf += data
        out = []
        if len(self.buf) > PKT_LEN * 100:  # 쓰레기만 쌓이면 비움
            self.buf.clear()
            self.resyncs += 1
            return out
        while len(self.buf) >= PKT_LEN:
            i = 0
            while i < len(self.buf) - 1 and not (self.buf[i] == HEADER and self.buf[i + 1] in (VER_LEN, VER_LEN | 0xE0)):
                i += 1
            if i:
                del self.buf[:i]
                self.resyncs += 1
            if len(self.buf) < PKT_LEN:
                break
            pkt = decode_packet(bytes(self.buf[:PKT_LEN]))
            if pkt is None:
                self.crc_errors += 1
                del self.buf[:2]
                continue
            del self.buf[:PKT_LEN]
            span = ((pkt.end // 100) - (pkt.start // 100) + 360) % 360
            if span > pkt.speed * POINTS_PER_PACK / SAMPLE_RATE * 3 / 2:
                self.angle_errors += 1
                continue
            self.packets += 1
            out.append(pkt)
        return out


def packet_points(pkt):
    """Packet → [(angle_deg(시계 방향, 0~360), distance_mm, confidence)] — 제조사와 같은 각도 보간."""
    diff = (pkt.end + 36000 - pkt.start) % 36000
    step = diff / (POINTS_PER_PACK - 1) / 100.0
    start = pkt.start / 100.0
    pts = []
    for i, (d, c) in enumerate(pkt.points):
        a = start + i * step
        if a >= 360.0:
            a -= 360.0
        pts.append((a, d, c))
    return pts


def near_filter(points, speed):
    """제조사 Tofbf::NearFilter 와 같은 규칙(입력 [(angle, dist, conf)])."""
    normal = [p for p in points if p[1] >= 5000]
    pending = sorted((p for p in points if p[1] < 5000), key=lambda p: p[0])
    if not points:
        return normal
    limit = speed / SAMPLE_RATE * 2
    groups, item, last = [], [], (-10.0, 0, 0)
    for p in pending:
        if abs(p[0] - last[0]) > limit or abs(p[1] - last[1]) > last[1] * 0.03:
            if item:
                groups.append(item)
                item = []
        item.append(p)
        last = p
    if item:
        groups.append(item)
    if not groups:
        return normal
    first, lastp = groups[0][0], groups[-1][-1]
    if len(groups) > 1 and abs(first[0] + 360.0 - lastp[0]) < limit and abs(first[1] - lastp[1]) < last[1] * 0.03:
        groups[0] = groups[-1] + groups[0]
        groups.pop()
    for g in groups:
        if len(g) > 15:
            normal.extend(g)
            continue
        if len(g) < 3 and sum(p[2] for p in g) / len(g) < CONFIDENCE_SINGLE:
            continue
        if sum(p[2] for p in g) / len(g) > CONFIDENCE_LOW:
            normal.extend(g)
    return normal


class ScanAssembler(object):
    """Packet 들을 모아 한 바퀴가 되면 (ranges[m], intensities, angle_increment, speed) 를 돌려준다.

    빔 수는 고정(beams, 기본 450 = 10 Hz 에서 0.8°)으로 LaserScan 크기가 흔들리지 않게 한다.
    """

    def __init__(self, beams=450, range_min=0.02, range_max=12.0, self_box=(0.16, 0.105), use_near_filter=True):
        self.beams = beams
        self.inc = 2 * math.pi / beams
        self.range_min, self.range_max = range_min, range_max
        self.self_box = self_box
        self.use_near_filter = use_near_filter
        self._pts = []
        self._last_angle = None
        self._speed = 0

    def add(self, pkt):
        """완성된 스캔(dict) 또는 None."""
        self._speed = pkt.speed
        done = None
        for a, d, c in packet_points(pkt):
            if self._last_angle is not None and a - self._last_angle < -350.0 and self._pts:
                done = self._finish()
            self._pts.append((a, d, c))
            self._last_angle = a
        return done

    def _finish(self):
        pts = near_filter(self._pts, self._speed) if self.use_near_filter else self._pts
        self._pts = []
        n = self.beams
        ranges = [float("nan")] * n
        inten = [0.0] * n
        bx, by = self.self_box if self.self_box else (0.0, 0.0)
        for a, d, c in pts:
            r = d / 1000.0
            if r < self.range_min or r > self.range_max:
                continue
            # 제조사는 int() 로 버림 — 각도가 빔 경계에 걸리면 이웃 빔과 겹쳐 빈 빔이 생겨 반올림한다(최대 0.4° 차이)
            idx = int(round(math.radians(a) / self.inc)) % n
            ri = n - 1 - idx  # 시계 → 반시계(제조사 ToLaserscan)
            if self.self_box:
                th = ri * self.inc
                if abs(r * math.cos(th)) <= bx and abs(r * math.sin(th)) <= by:
                    continue  # 로봇 몸·다리
            if math.isnan(ranges[ri]) or r < ranges[ri]:
                ranges[ri] = r
                inten[ri] = float(c)
        return {"ranges": ranges, "intensities": inten, "angle_increment": self.inc, "speed": self._speed}
