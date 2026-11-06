"""Protocol v2 수신기 단위·UDP 시험 (요구사항 §4.4~§4.8).

ROS 없이 실행된다:  python3 -m pytest tests/test_protocol_v2.py -v
모든 ACK 는 vr_protocol_v2/schemas/ack_v2.schema.json 의 필드 8개 형식인지 함께 확인한다.
"""

import json
import os
import socket
import sys
import threading
import time
import uuid

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "puppy_vr_bridge"))

from puppy_vr_bridge.protocol_v2 import (STOP, V2_MAX_SEQUENCE, SlewLimiter, StartHold, V2PacketError,  # noqa: E402
                                         V2Session, encode_ack, normalized_to_velocity, parse_v2_packet)
from puppy_vr_bridge.v2_server import V2UdpServer  # noqa: E402

CID = "d8b27f10-09a8-4c21-b649-1e7058d46b72"
OTHER = "0f0e0d0c-0b0a-4908-8706-050403020100"
ADDR = ("192.0.2.11", 50000)


def cmd(seq, active=True, fwd=0.0, turn=0.0, client=CID, **override):
    d = {"protocol": 2, "type": "command", "client_id": client, "sequence": seq,
         "control_active": active, "forward": fwd, "turn": turn}
    d.update(override)
    return json.dumps(d).encode()


def ev(kind, seq, client=CID):
    return json.dumps({"protocol": 2, "type": kind, "client_id": client, "sequence": seq}).encode()


ACK_FIELDS = set(json.load(open(os.path.join(ROOT, "vr_protocol_v2", "schemas", "ack_v2.schema.json"),
                                  encoding="utf-8"))["required"])


def _check_ack_shape(text):
    """ACK 는 평평한 JSON 한 개, 스키마의 필드 8개만."""
    data = json.loads(text)
    assert isinstance(data, dict) and set(data) == ACK_FIELDS, data


class H:
    """가짜 시계 하네스."""

    def __init__(self, **kw):
        args = dict(max_forward=5.0, max_reverse=3.0, max_yaw_rate=0.2, command_timeout=0.25, lease_timeout=1.0)
        args.update(kw)
        self.s = V2Session(**args)
        self.now = 100.0
        self.seq = 0

    def send(self, data, addr=ADDR, dt=0.05):
        self.now += dt
        r = self.s.handle_datagram(data, addr, self.now)
        _check_ack_shape(encode_ack(r.ack).decode())
        return r

    def next(self, active=True, fwd=0.0, turn=0.0, client=CID, addr=ADDR, dt=0.05):
        self.seq += 1
        return self.send(cmd(self.seq, active, fwd, turn, client), addr, dt)

    def neutral(self):
        return self.next(False)

    def tick(self, dt):
        self.now += dt
        return self.s.check_timeouts(self.now)


def ok(r, accepted=True, reason="ok", state=None):
    assert r.ack["accepted"] is accepted, r.ack
    assert r.ack["reason"] == reason, r.ack
    if state:
        assert r.ack["state"] == state, r.ack


# ───────────── 파싱·검증 ─────────────

def test_normal_json_example_from_spec():
    p = parse_v2_packet(cmd(123, True, 0.6, -0.3))
    assert (p.action, p.sequence, p.control_active, p.forward, p.turn) == ("command", 123, True, 0.6, -0.3)
    assert p.client_id == CID


def test_utf8_whitespace_and_field_order_free():
    raw = '  {"turn": -0.3, "forward": 0.6, "control_active": true, "sequence": 1,\n "client_id": "%s", "type": "command", "protocol": 2}  ' % CID
    assert parse_v2_packet(raw.encode("utf-8")).turn == -0.3


@pytest.mark.parametrize("data,reason", [
    (cmd(1, protocol=1), "unsupported_protocol"),
    (cmd(1, protocol=3), "unsupported_protocol"),
    (cmd(1, protocol="2"), "invalid_packet"),
    (cmd(1, protocol=True), "invalid_packet"),
    (cmd(1, type="drive"), "invalid_packet"),
    (cmd(1, type=None), "invalid_packet"),
])
def test_protocol_and_type_checks(data, reason):
    with pytest.raises(V2PacketError) as e:
        parse_v2_packet(data)
    assert e.value.reason == reason


def _without(key):
    d = json.loads(cmd(5, True, 0.1, 0.1))
    del d[key]
    return json.dumps(d).encode()


@pytest.mark.parametrize("key", ["protocol", "type", "client_id", "sequence", "control_active", "forward", "turn"])
def test_each_required_field_missing_is_rejected(key):
    with pytest.raises(V2PacketError) as e:
        parse_v2_packet(_without(key))
    assert e.value.reason in ("invalid_packet", "unsupported_protocol")


def test_missing_client_id_and_sequence_ack_identifiers():
    h = H()
    r = h.send(_without("client_id"))
    ok(r, False, "invalid_packet")
    assert r.ack["client_id"] is None and r.ack["sequence"] == 5
    r = h.send(_without("sequence"))
    ok(r, False, "invalid_packet")
    assert r.ack["client_id"] == CID and r.ack["sequence"] == -1


@pytest.mark.parametrize("field,value", [
    ("control_active", 1), ("control_active", "true"), ("control_active", None),
    ("forward", True), ("forward", False), ("forward", "0.5"), ("forward", None), ("forward", [0.1]),
    ("turn", True), ("turn", {"v": 1}),
    ("sequence", True), ("sequence", 1.0), ("sequence", "1"), ("sequence", -1), ("sequence", V2_MAX_SEQUENCE + 1),
    ("client_id", 123), ("client_id", "not-a-uuid"), ("client_id", "00000000-0000-0000-0000-000000000000"),
    ("client_id", "D8B27F1009A84C21B6491E7058D46B72"), ("client_id", "{%s}" % CID),
])
def test_exact_types_bool_is_not_int(field, value):
    d = json.loads(cmd(1, True, 0.1, 0.1))
    d[field] = value
    with pytest.raises(V2PacketError):
        parse_v2_packet(json.dumps(d).encode())


@pytest.mark.parametrize("literal", ["NaN", "Infinity", "-Infinity", "1e400", "-1e400"])
def test_nan_infinity_rejected(literal):
    raw = ('{"protocol":2,"type":"command","client_id":"%s","sequence":1,"control_active":true,'
           '"forward":%s,"turn":0}' % (CID, literal)).encode()
    with pytest.raises(V2PacketError):
        parse_v2_packet(raw)


@pytest.mark.parametrize("value", [1.0000001, -1.5, 2, 10 ** 400, -(10 ** 30)])
def test_out_of_range_axes_rejected(value):
    with pytest.raises(V2PacketError):
        parse_v2_packet(cmd(1, True, value, 0.0))


def test_boundary_values_and_exponent_floats_accepted():
    assert parse_v2_packet(cmd(1, True, 1, -1)).forward == 1.0
    raw = ('{"protocol":2,"type":"command","client_id":"%s","sequence":1,"control_active":true,'
           '"forward":1E-05,"turn":-0.0}' % CID).encode()
    p = parse_v2_packet(raw)
    assert p.forward == 1e-05 and p.turn == 0.0


@pytest.mark.parametrize("data", [
    b"", b"{", b"[]", b"null", b"\xff\xfe", "{\"a\":1}".encode(), b"x" * 3000,
    ("{" * 5000 + "}" * 5000).encode(),
    ('{"protocol":2,"protocol":2,"type":"command","client_id":"%s","sequence":1,"control_active":true,"forward":0,"turn":0}' % CID).encode(),
    cmd(1, extra=1),
])
def test_malformed_oversized_duplicate_extra_rejected_without_crash(data):
    h = H()
    r = h.send(data)
    ok(r, False, r.ack["reason"])
    assert r.ack["reason"] in ("invalid_packet", "unsupported_protocol")


def test_events_require_exact_common_fields():
    assert parse_v2_packet(ev("disconnect", 3)).action == "disconnect"
    bad = json.loads(ev("emergency_stop", 3))
    bad["forward"] = 0
    with pytest.raises(V2PacketError):
        parse_v2_packet(json.dumps(bad).encode())


# ───────────── 변환 ─────────────

def test_mapping_example_and_signs():
    assert normalized_to_velocity(0.6, -0.3, 5.0, 3.0, 0.2) == pytest.approx((3.0, 0.0, -0.06))
    assert normalized_to_velocity(-1.0, 0.0, 5.0, 3.0, 0.2) == (-3.0, 0.0, 0.0)
    assert normalized_to_velocity(0.0, 1.0, 5.0, 3.0, 0.2) == (0.0, 0.0, 0.2)  # +turn = 좌회전(+yaw)
    assert normalized_to_velocity(0.0, 0.0, 5.0, 3.0, 0.2) == (0.0, 0.0, 0.0)


def test_slew_limiter_ramps_but_stop_is_immediate():
    s = SlewLimiter(10.0, 0.4)
    assert s.update(5.0, 0.2, 0.0) == (0.0, 0.0)
    x, y = s.update(5.0, 0.2, 0.1)
    assert x == pytest.approx(1.0) and y == pytest.approx(0.04)
    for t in range(2, 10):
        x, y = s.update(5.0, 0.2, t * 0.1)
    assert (x, y) == pytest.approx((5.0, 0.2))
    assert s.update(0.0, 0.0, 1.05) == (0.0, 0.0)
    assert s.update(5.0, 0.0, 10.0)[0] == pytest.approx(1.0)  # 긴 공백 뒤에도 dt 상한(0.1s)


# ───────────── 세션: 중립·활성·control_active ─────────────

def test_neutral_required_before_motion_and_control_active_false_stops():
    h = H()
    ok(h.next(True, 0.6, 0.0), False, "neutral_required", "awaiting_neutral")
    r = h.neutral()
    ok(r, True, "ok", "idle")
    r = h.next(True, 0.6, -0.3)
    ok(r, True, "ok", "active")
    assert r.velocity == pytest.approx((3.0, 0.0, -0.06))
    r = h.next(False, 0.6, -0.3)  # control_active=false 이면 값이 남아 있어도 정지
    ok(r, True, "ok", "idle")
    assert r.velocity == STOP


def test_forward_zero_turn_zero_and_combined():
    h = H()
    h.neutral()
    assert h.next(True, 0.0, 0.5).velocity == pytest.approx((0.0, 0.0, 0.1))
    assert h.next(True, 0.5, 0.0).velocity == pytest.approx((2.5, 0.0, 0.0))
    assert h.next(True, -0.5, 0.5).velocity == pytest.approx((-1.5, 0.0, 0.1))
    r = h.next(True, 0.0, 0.0)  # active 중립 = 속도 0
    assert r.velocity == STOP and r.ack["state"] == "active"


# ───────────── sequence ─────────────

def test_increasing_duplicate_reordered_and_gap():
    h = H()
    ok(h.send(cmd(10, False)), True, "ok", "idle")
    ok(h.send(cmd(11, True, 0.5)), True, "ok", "active")
    r = h.send(cmd(11, True, 1.0))  # 중복(재전송)
    ok(r, False, "out_of_order")
    assert r.velocity is None and h.s.target == pytest.approx((2.5, 0.0, 0.0))
    ok(h.send(cmd(9, True, -1.0)), False, "out_of_order")  # 늦게 온 옛 명령은 최신을 덮지 못함
    ok(h.send(cmd(20, True, 0.2)), True, "ok", "active")  # 유실(12~19)은 허용
    assert h.s.target == pytest.approx((1.0, 0.0, 0.0))


def test_sequence_limits():
    h = H()
    ok(h.send(cmd(V2_MAX_SEQUENCE - 1, False)), True)
    ok(h.send(cmd(V2_MAX_SEQUENCE, False)), True)
    ok(h.send(cmd(V2_MAX_SEQUENCE + 1, False)), False, "invalid_packet")


def test_sequence_shared_by_events_and_commands():
    h = H()
    h.send(cmd(1, False))
    h.send(ev("emergency_stop", 2))
    ok(h.send(cmd(2, False)), False, "out_of_order")
    ok(h.send(cmd(3, False)), True, "ok", "emergency_stop")


# ───────────── watchdog·lease·재연결 ─────────────

def test_watchdog_stops_after_timeout_and_requires_new_neutral():
    h = H()
    h.neutral()
    h.next(True, 1.0)
    assert h.tick(0.24) == []
    events = h.tick(0.02)
    assert events and "watchdog" in events[0]
    assert h.s.target == STOP and h.s.state == "timeout"
    ok(h.next(True, 1.0, dt=0.01), False, "neutral_required", "timeout")
    ok(h.neutral(), True, "ok", "idle")


def test_expiry_is_checked_before_late_packet_is_applied():
    """VR 패치와 같은 수정: 0.26s 뒤 도착한 활성 명령이 만료 구간을 지우지 못한다."""
    h = H()
    h.neutral()
    h.next(True, 1.0)
    r = h.next(True, 1.0, dt=0.26)
    ok(r, False, "neutral_required", "timeout")
    assert r.velocity == STOP and "watchdog" in r.event


def test_invalid_packets_do_not_refresh_watchdog():
    h = H()
    h.neutral()
    h.next(True, 1.0)
    for _ in range(4):  # 0.20s 동안 잘못된 패킷만: 아직 만료 전
        h.send(b"{broken", dt=0.05)
    assert h.s.state == "active"
    for _ in range(2):  # 0.30s: 잘못된 패킷은 watchdog 을 갱신하지 않으므로 만료
        h.send(b"{broken", dt=0.05)
    assert h.s.state == "timeout" and h.s.target == STOP


def test_lease_expiry_and_reacquire_same_uuid_with_higher_sequence():
    h = H()
    h.neutral()
    h.tick(1.01)
    assert h.s.owner is None
    ok(h.send(cmd(1, False)), False, "out_of_order")  # 같은 UUID 의 옛 번호
    ok(h.send(cmd(h.seq + 1, False)), True, "ok", "idle")
    assert h.s.owner == CID


def test_disconnect_releases_and_retires_session():
    h = H()
    h.neutral()
    h.next(True, 0.5)
    h.seq += 1
    r = h.send(ev("disconnect", h.seq))
    ok(r, True)
    assert r.velocity == STOP and h.s.owner is None
    ok(h.next(False), False, "not_owner")  # 종료된 세션의 지연 패킷
    ok(h.send(cmd(1, False, client=OTHER)), True, "ok", "idle")  # 새 앱 실행(새 UUID)


def test_app_restart_new_uuid_must_wait_for_lease():
    h = H()
    h.neutral()
    ok(h.send(cmd(1, False, client=OTHER)), False, "not_owner")
    h.tick(1.01)
    ok(h.send(cmd(1, False, client=OTHER)), True, "ok", "idle")
    ok(h.send(cmd(999, False, client=CID)), False, "not_owner")  # 교체된 세션은 retire


def test_reconnect_from_new_source_port_waits_for_lease():
    h = H()
    h.neutral()
    ok(h.next(False, addr=("192.0.2.11", 50001)), False, "not_owner")
    h.tick(1.01)
    ok(h.next(False, addr=("192.0.2.11", 50001)), True, "ok", "idle")


# ───────────── 비상정지·중재기·fault ─────────────

def test_emergency_latch_and_clear():
    h = H()
    h.neutral()
    h.next(True, 1.0)
    h.seq += 1
    r = h.send(ev("emergency_stop", h.seq))
    assert r.velocity == STOP and r.estop_change is True and r.ack["emergency_stop"] is True
    ok(h.next(True, 1.0), False, "emergency_stop", "emergency_stop")
    ok(h.neutral(), True, "ok", "emergency_stop")
    h.seq += 1
    r = h.send(ev("clear_emergency", h.seq))
    assert r.estop_change is False and h.s.state == "awaiting_neutral"


def test_estop_from_non_owner_is_latched_without_stealing_ownership():
    h = H()
    h.neutral()
    h.next(True, 1.0)
    r = h.send(ev("emergency_stop", 1, client=OTHER), addr=("192.0.2.12", 40000))
    ok(r, True, "ok", "emergency_stop")
    assert h.s.owner == CID and h.s.target == STOP


def test_motion_not_permitted_by_arbiter_rejects_with_not_owner_but_keeps_neutral():
    h = H()
    h.neutral()
    h.next(True, 1.0)
    assert h.s.set_motion_permitted(False, "legacy 경로가 이동 중") is True
    assert h.s.target == STOP
    h.neutral()
    ok(h.next(True, 1.0), False, "not_owner")
    h.s.set_motion_permitted(True)
    ok(h.next(True, 1.0), True, "ok", "active")


def test_preempt_requires_new_neutral():
    h = H()
    h.neutral()
    h.next(True, 1.0)
    h.s.preempt()
    ok(h.next(True, 1.0), False, "neutral_required")


def test_external_estop_mirrors_robot_latch():
    h = H()
    h.neutral()
    h.s.set_external_estop(True)
    ok(h.next(True, 1.0), False, "emergency_stop")
    h.s.set_external_estop(False)
    assert h.s.state == "awaiting_neutral"


def test_fault_latch():
    h = H()
    h.neutral()
    h.s.enter_fault("x")
    ok(h.next(True, 1.0), False, "fault", "fault")




# ───────────── Unity 세션 로직과의 상호작용 ─────────────



# ───────────── 실제 UDP ─────────────

@pytest.fixture
def server():
    outputs = []
    lock = threading.Lock()

    def on_output(v, reason):
        with lock:
            outputs.append((time.monotonic(), v, reason))

    s = V2UdpServer(V2Session(5.0, 3.0, 0.2, 0.25, 1.0), on_output, "127.0.0.1", 0)
    s.start()
    s.outputs = outputs
    yield s
    s.shutdown()




def test_udp_flood_of_garbage_does_not_kill_receiver(server):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for i in range(2000):
        sock.sendto(os.urandom(i % 3000 + 1), ("127.0.0.1", server.bound_port))
    time.sleep(0.5)  # 커널 수신 버퍼가 비워질 시간(버퍼 초과분은 커널이 버림)
    sock.settimeout(0.3)
    acks = []
    for attempt in range(5):
        sock.sendto(cmd(1 + attempt, False), ("127.0.0.1", server.bound_port))
        try:
            while True:
                a = json.loads(sock.recv(4096))
                acks.append(a)
                if a["client_id"] == CID:
                    break
        except socket.timeout:
            continue
        break
    assert any(a["client_id"] == CID and a["accepted"] for a in acks), acks[-3:]
    assert server._rx_thread.is_alive()
    assert server.stats.reasons["invalid_packet"] >= 100
    sock.close()


def test_rapid_input_burst_keeps_latest(server):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = ("127.0.0.1", server.bound_port)
    sock.sendto(cmd(1, False), addr)
    for i in range(2, 202):
        sock.sendto(cmd(i, True, (i % 20) / 20.0), addr)
    sock.sendto(cmd(202, True, 0.4, 0.5), addr)
    time.sleep(0.2)
    assert server.session.target == pytest.approx((2.0, 0.0, 0.1))
    assert server.stats.packets >= 150  # UDP 손실 허용(로컬 버퍼)
    sock.close()


def test_second_server_on_same_port_fails_visibly(server):
    other = V2UdpServer(V2Session(5, 3, 0.2, 0.25, 1.0), lambda v, r: None, "127.0.0.1", server.bound_port)
    with pytest.raises(OSError):
        other.start()






def test_reverse_scale_doubles_reverse_but_keeps_cap():
    from puppy_vr_bridge.protocol_v2 import normalized_to_velocity
    # 이전 동작: 속도 50 % 끝까지 후진 = -0.5 × 5 = -2.5
    assert normalized_to_velocity(-0.5, 0.0, 25.0, 5.0, 0.28)[0] == -2.5
    # 환산 10: -0.5 × 10 = -5.0, 끝까지 당겨도 상한 -5.0
    assert normalized_to_velocity(-0.5, 0.0, 25.0, 5.0, 0.28, 10.0)[0] == -5.0
    assert normalized_to_velocity(-1.0, 0.0, 25.0, 5.0, 0.28, 10.0)[0] == -5.0
    assert normalized_to_velocity(-0.25, 0.0, 25.0, 5.0, 0.28, 10.0)[0] == -2.5
    # 전진은 바뀌지 않는다
    assert normalized_to_velocity(0.5, 0.0, 25.0, 5.0, 0.28, 10.0)[0] == 12.5


def test_session_uses_reverse_scale():
    s = V2Session(25.0, 5.0, 0.28, 0.4, 1.0, reverse_scale=10.0)
    assert s._reverse_scale == 10.0


def test_start_hold_delays_only_start():
    h = StartHold(0.2)
    assert h.update(0.0, 0.0, 0.00) == (0.0, 0.0)
    assert h.update(3.4, 0.0, 0.02) == (0.0, 0.0) and h.pending   # 출발 대기
    assert h.update(8.1, 0.0, 0.15) == (0.0, 0.0)
    assert h.update(10.9, 0.0, 0.22) == (10.9, 0.0) and not h.pending  # 0.2 s 뒤 그때 값
    assert h.update(12.5, 0.0, 0.30) == (12.5, 0.0)              # 걷는 중 변경은 바로
    assert h.update(-5.0, 0.0, 0.35) == (-5.0, 0.0)              # 0 을 거치지 않은 방향 전환도 바로
    assert h.update(0.0, 0.0, 0.40) == (0.0, 0.0)                # 정지는 즉시
    assert h.update(0.0, 0.1, 0.45) == (0.0, 0.0)                # 다시 출발하면 다시 대기
    assert h.update(0.0, 0.1, 0.66) == (0.0, 0.1)


def test_start_hold_zero_is_off():
    h = StartHold(0.0)
    assert h.update(3.4, 0.0, 0.0) == (3.4, 0.0) and not h.pending


def test_turn_x_deadband_keeps_turn_in_place():
    from puppy_vr_bridge.protocol_v2 import normalized_to_velocity
    # 끔: 좌회전 + 엄지 살짝 위(0.05 × 25 = 1.25 cm/s) → 그대로
    assert normalized_to_velocity(0.05, 0.5, 25.0, 5.0, 0.28, 10.0)[0] == 1.25
    # 켬(4 cm/s): 회전 중 작은 전진·후진은 0
    x, _, yaw = normalized_to_velocity(0.05, 0.5, 25.0, 5.0, 0.28, 10.0, 4.0)
    assert x == 0.0 and yaw == 0.14
    assert normalized_to_velocity(-0.3, -0.5, 25.0, 5.0, 0.28, 10.0, 4.0)[0] == 0.0   # -3 cm/s 후진 + 우회전 → 제자리
    # 크게 밀며 돌기(곡선)는 그대로
    assert normalized_to_velocity(0.5, 0.3, 25.0, 5.0, 0.28, 10.0, 4.0)[0] == 12.5
    # 회전 없으면 작은 전진도 그대로(정밀 접근)
    assert normalized_to_velocity(0.05, 0.0, 25.0, 5.0, 0.28, 10.0, 4.0)[0] == 1.25


def test_session_passes_turn_x_deadband():
    s = V2Session(25.0, 5.0, 0.28, 0.4, 1.0, reverse_scale=10.0, turn_x_deadband=4.0)
    assert s._turn_x_deadband == 4.0



# ───────────── 경로 지정 주행 메시지(2026-11-05) ─────────────

def pathp(seq, points, client=CID, **override):
    d = {"protocol": 2, "type": "path", "client_id": client, "sequence": seq, "points": points}
    d.update(override)
    return json.dumps(d).encode()


def test_path_packet_parse_and_limits():
    from puppy_vr_bridge.protocol_v2 import MAX_PATH_POINTS, parse_v2_packet
    p = parse_v2_packet(pathp(1, [[1, 0], [1.5, -0.25]]))
    assert p.action == "path" and p.points == ((1.0, 0.0), (1.5, -0.25))
    assert parse_v2_packet(ev("path_cancel", 2)).action == "path_cancel"
    bad = [[], [[1]], [[1, 2, 3]], "x", [[True, 0]], [["1", 0]], [[101, 0]],
           [[0, 0]] * (MAX_PATH_POINTS + 1)]
    for pts in bad:
        with pytest.raises(V2PacketError) as e:
            parse_v2_packet(pathp(1, pts))
        assert e.value.reason == "invalid_packet", pts
    with pytest.raises(V2PacketError):
        parse_v2_packet(pathp(1, [[1, 0]], extra=1))
    with pytest.raises(V2PacketError):   # path_cancel 에 points 가 있으면 거부
        parse_v2_packet(json.dumps({"protocol": 2, "type": "path_cancel", "client_id": CID, "sequence": 1,
                                    "points": [[0, 0]]}).encode())
    with pytest.raises(V2PacketError):
        parse_v2_packet(b'{"protocol":2,"type":"path","client_id":"%s","sequence":1,"points":[[NaN,0]]}' % CID.encode())


def test_path_session_rules():
    h = H()
    r = h.send(pathp(1, [[1, 0]]))
    ok(r, False, "neutral_required")
    assert r.path is None and r.velocity is None
    h.seq = 1
    ok(h.neutral(), True, "ok", "idle")
    h.seq += 1
    r = h.send(pathp(h.seq, [[1, 0], [1, 1]]))
    ok(r, True, "ok", "idle")
    assert r.path == ((1.0, 0.0), (1.0, 1.0)) and r.velocity is None     # 이동 목표는 그대로(0)
    h.seq += 1
    r = h.send(ev("path_cancel", h.seq))
    ok(r, True)
    assert r.path == ()
    r = h.send(pathp(h.seq + 1, [[1, 0]], client=OTHER), addr=("192.0.2.99", 1))
    ok(r, False, "not_owner")
    h.seq += 1
    ok(h.send(ev("emergency_stop", h.seq)), True)
    h.seq += 1
    r = h.send(pathp(h.seq, [[1, 0]]))
    ok(r, False, "emergency_stop")
    assert r.path is None


def test_path_callback_from_udp_server():
    got = []
    session = V2Session(5.0, 3.0, 0.2, 0.25, 1.0)
    srv = V2UdpServer(session, lambda v, r: None, "127.0.0.1", 0, on_path=lambda p, o: got.append((p, o)))
    srv.start()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1.0)
        s.sendto(cmd(1, active=False), ("127.0.0.1", srv.bound_port)); s.recv(2048)
        s.sendto(pathp(2, [[0.5, 0.0]]), ("127.0.0.1", srv.bound_port))
        ack = json.loads(s.recv(2048))
        assert ack["accepted"] is True
        deadline = time.time() + 1.0
        while not got and time.time() < deadline:
            time.sleep(0.01)
        assert got == [(((0.5, 0.0),), CID)]
        s.close()
    finally:
        srv.shutdown()
