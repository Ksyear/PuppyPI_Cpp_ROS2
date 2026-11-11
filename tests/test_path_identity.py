"""경로 주행 실행 식별자·장애물 판단 이전(2026-11-11 MR 팀 작업 지시서 6절) 연결 시험. ROS·모터 없이 실행.

실제 V2Session·V2UdpServer·PathFollower 를 쓰고, 수신기(receiver_node)가 하는 연결만 아래 Rig 에 옮겼다
(_on_output·_on_path·_tick 의 경로 취소 규칙). 출력은 가짜(기록만), 로봇 운동은 단순 적분, 시계는 가짜.
receiver_node 자체는 ROS 2 컨테이너에서 따로 확인한다.
"""
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "puppy_vr_bridge"))
from puppy_vr_bridge.path_follower import PathFollower, wrap  # noqa: E402
from puppy_vr_bridge.protocol_v2 import V2Session  # noqa: E402
from puppy_vr_bridge.v2_server import V2UdpServer  # noqa: E402

CID = "a92e1842-7ef1-4d1f-bc14-2a04dcd0da87"
OTHER = "0f0e0d0c-0b0a-4908-8706-050403020100"
ADDR = ("192.0.2.10", 50000)


def cmd(seq, active=False, fwd=0.0, turn=0.0, client=CID):
    return json.dumps({"protocol": 2, "type": "command", "client_id": client, "sequence": seq,
                       "control_active": active, "forward": fwd, "turn": turn}).encode()


def ev(kind, seq, client=CID):
    return json.dumps({"protocol": 2, "type": kind, "client_id": client, "sequence": seq}).encode()


def pathp(seq, points, client=CID):
    return json.dumps({"protocol": 2, "type": "path", "client_id": client, "sequence": seq,
                       "points": points}).encode()


class Rig:
    def __init__(self, defer_path=False, scan_fn=lambda t, pose: [], scan_ok=lambda t: True):
        self.now = 100.0
        self.session = V2Session(25.0, 5.0, 0.28, 0.4, 1.0)
        self.f = PathFollower()
        self.defer, self.pending = defer_path, []
        self.events = []          # ("ack", ack) / ("path", args) — 순서 확인용
        self.target = (0.0, 0.0)
        self.srv = V2UdpServer(self.session, self._on_output, "127.0.0.1", 0, on_path=self._on_path,
                               clock=lambda: self.now)
        self.srv._send = lambda addr, ack: self.events.append(("ack", ack))   # 소켓 대신 기록
        self.seq = 0
        self.pos = (0.0, 0.0, 0.0)
        self.pose = self.scan = None
        self.pose_t = self.scan_t = -10.0
        self.scan_fn, self.scan_ok = scan_fn, scan_ok
        self.log = []
        self._key = None
        self.neutral()

    # ── receiver_node 와 같은 연결 ──
    def _on_output(self, velocity, reason):
        x, _y, yaw = velocity
        if self.f.active:
            if x == 0.0 and yaw == 0.0 and reason == "command":
                return
            self.f.cancel("수동 조작" if (x or yaw) else reason)
        self.target = (x, yaw)

    def _on_path(self, path, owner, cid=None, seq=None):
        self.events.append(("path", (path, owner, cid, seq)))
        if self.defer:
            self.pending.append((path, owner, cid, seq))
        else:
            self.apply(path, owner, cid, seq)

    def apply(self, path, owner, cid, seq):
        if path == ():
            self.f.cancel("VR 취소")
        else:
            self.f.set_path(path, owner, self.now, cid, seq)
            self.target = (0.0, 0.0)

    def flush(self):
        while self.pending:
            self.apply(*self.pending.pop(0))

    # ── VR 쪽 ──
    def send(self, data, addr=ADDR):
        n = len(self.events)
        self.srv._handle(data, addr)
        acks = [e[1] for e in self.events[n:] if e[0] == "ack"]
        return acks[-1]

    def nseq(self):
        self.seq += 1
        return self.seq

    def neutral(self):
        return self.send(cmd(self.nseq()))

    def path(self, pts):
        return self.send(pathp(self.nseq(), pts))

    def status(self):
        return self.f.status(self.now, self.pose)

    def ident(self):
        return self.f.request_client_id, self.f.request_sequence

    # ── 시간 진행: 50 Hz 수신기 tick, 25 Hz 중립 heartbeat, 10 Hz 위치·LiDAR ──
    def run(self, T, heartbeat=True, stop_when_idle=True, dt=0.02):
        end, k = self.now + T, 0
        while self.now < end - 1e-9:
            self.now += dt
            k += 1
            t = self.now
            if heartbeat and k % 2 == 0:
                self.neutral()
            self.srv.check_timeouts()
            if k % 5 == 0:
                self.pose, self.pose_t = self.pos, t
                if self.scan_ok(t):
                    self.scan, self.scan_t = self.scan_fn(t, self.pos), t
            st, owner, permitted = self.session.state, self.session.owner, self.session.motion_permitted
            if self.f.active:
                if st not in ("idle", "active"):
                    self.f.cancel("세션 %s" % st)
                elif not permitted:
                    self.f.cancel("이동 차단(중재기)")
                elif owner != self.f.owner:
                    self.f.cancel("조종자 바뀜")
                self.target = (self.f.update(t, self.pose, self.pose_t, self.scan, self.scan_t)
                               if self.f.active else (0.0, 0.0))
            key = (self.f.state, self.f.index, self.f.waiting, self.f.request_client_id, self.f.request_sequence)
            if key != self._key:                       # receiver_node: 상태가 바뀐 tick 에 끝난 경로의 목표를 0 으로
                self._key = key
                if not self.f.active and self.f.state in ("done", "cancelled"):
                    self.target = (0.0, 0.0)
            vx, w = self.target
            self.log.append((t, vx, w, self.f.state))
            x, y, th = self.pos
            self.pos = (x + vx / 100.0 * math.cos(th) * dt, y + vx / 100.0 * math.sin(th) * dt, wrap(th + w * dt))
            if stop_when_idle and not self.f.active and self.f.state != "idle":
                break


def front_points(dist_m):
    return [(dist_m, 0.0), (dist_m + 0.02, 0.01)]


def test_repeated_front_noise_follows_path_without_robot_stop():
    noise = lambda t, pose: front_points(0.25) if (t % 0.8) < 0.1 else []   # 0.8 s 마다 0.1 s
    clean, noisy = Rig(), Rig(scan_fn=noise)
    for r in (clean, noisy):
        assert r.path([[1.2, 0.0]])["accepted"]
        r.run(60.0)
    assert noisy.f.state == "done" and noisy.f.reason is None
    assert not any(s == "blocked" for *_, s in noisy.log)
    assert all(vx > 0 for _, vx, _, s in noisy.log if s == "drive")
    assert [r[1:] for r in noisy.log] == [r[1:] for r in clean.log]        # 잡음 없을 때와 같은 출력
    assert not noisy.session.emergency_stop and noisy.session.state == "idle"


def test_continuous_obstacle_robot_keeps_going_until_mr_path_cancel():
    r = Rig(scan_fn=lambda t, pose: front_points(0.25))
    r.path([[1.5, 0.0]])
    p_seq = r.seq
    r.run(3.0)
    assert r.f.state == "drive" and r.log[-1][1] > 0                       # 스스로 서지 않음
    assert r.send(ev("path_cancel", r.nseq()))["accepted"]
    r.run(0.1, stop_when_idle=False)
    assert r.f.state == "cancelled" and r.f.reason == "VR 취소"
    assert all(vx == 0.0 and w == 0.0 for _, vx, w, _ in r.log[-5:])
    assert r.ident() == (CID, p_seq)                                        # 취소 패킷 sequence 로 덮지 않음
    assert not r.session.emergency_stop


def test_second_path_in_same_session_runs_with_new_sequence():
    r = Rig()
    r.path([[0.5, 0.0]])
    s1 = r.seq
    r.run(30.0)
    assert r.f.state == "done" and r.ident() == (CID, s1)
    owner = r.session.owner
    r.path([[0.5, 0.5]])
    s2 = r.seq
    assert r.ident() == (CID, s2) and s2 > s1 and r.f.state == "rotate"
    r.run(60.0)
    assert r.f.state == "done" and r.ident() == (CID, s2) and r.session.owner == owner


def test_short_path_done_before_first_query_has_this_request_id():
    r = Rig()
    r.path([[0.0, 0.0], [0.12, 0.0]])
    p_seq = r.seq
    r.run(5.0)
    st = r.status()                                                          # 첫 조회가 완료 뒤에 와도
    assert st["state"] == "done" and st["index"] == st["count"] == 2
    assert (st["request_client_id"], st["request_sequence"]) == (CID, p_seq)
    assert r.log[-1][0] - 100.0 < 1.0


def test_same_points_resent_with_delayed_callback():
    r = Rig(defer_path=True)
    pts = [[0.0, 0.0], [0.12, 0.0]]
    r.path(pts)
    s_old = r.seq
    r.flush()
    r.run(5.0)
    assert r.f.state == "done" and r.ident() == (CID, s_old)
    n = len(r.events)
    ack = r.path(pts)                                                        # 같은 좌표 재요청
    s_new = r.seq
    assert ack["accepted"] and ack["sequence"] == s_new
    kinds = [e[0] for e in r.events[n:]]
    assert kinds == ["ack", "path"]                                          # ACK 가 먼저, 적용 콜백은 그 뒤
    st = r.status()                                                          # 콜백 적용 전: 이전 실행 그대로
    assert st["state"] == "done" and st["request_sequence"] == s_old != s_new
    r.flush()
    st = r.status()
    assert st["state"] == "rotate" and st["request_sequence"] == s_new and st["index"] == 0
    r.run(5.0)
    assert r.f.state == "done" and r.ident() == (CID, s_new)


def test_lost_path_ack_identified_by_status_without_resend():
    r = Rig()
    r.send(pathp(r.nseq(), [[0.6, 0.0]]))                                    # ACK 는 받지 못했다고 가정
    p_seq = r.seq
    r.run(1.0, stop_when_idle=False)                                         # 중립 heartbeat 만 계속
    paths_sent = sum(1 for e in r.events if e[0] == "path")
    assert paths_sent == 1 and r.status()["request_sequence"] == p_seq and r.f.active


def test_identity_update_rules_for_heartbeat_cancel_and_rejected():
    r = Rig()
    assert r.ident() == (None, None)
    r.path([[1.0, 0.0]])
    p = r.seq
    r.run(0.5, stop_when_idle=False)                                         # 중립 heartbeat 다수
    assert r.ident() == (CID, p)
    assert r.send(pathp(p, [[2.0, 0.0]]))["reason"] == "out_of_order"        # 중복
    assert r.send(pathp(p - 1, [[2.0, 0.0]]))["reason"] == "out_of_order"    # 역순
    assert r.send(pathp(r.seq + 5, [[2.0, 0.0]], client=OTHER), addr=("192.0.2.11", 1))["reason"] == "not_owner"
    assert r.ident() == (CID, p) and r.f.points == [(1.0, 0.0)]
    r.send(ev("path_cancel", r.nseq()))
    assert r.f.state == "cancelled" and r.ident() == (CID, p)
    r.send(ev("emergency_stop", r.nseq()))
    assert r.send(pathp(r.nseq(), [[3.0, 0.0]]))["reason"] == "emergency_stop"
    assert r.ident() == (CID, p)


def test_path_before_neutral_is_rejected_and_identity_stays_null():
    r = Rig()
    r.session = V2Session(25.0, 5.0, 0.28, 0.4, 1.0)
    r.srv.session = r.session
    assert r.send(pathp(1, [[1.0, 0.0]]))["reason"] == "neutral_required"
    st = r.status()
    assert st["request_client_id"] is None and st["request_sequence"] is None and st["state"] == "idle"


def test_scan_loss_stops_with_existing_timeout_and_resumes_without_reset():
    r = Rig(scan_ok=lambda t: not (102.0 <= t < 104.0))
    r.path([[2.0, 0.0]])
    r.run(60.0)
    lost = [e for e in r.log if 102.75 < e[0] < 104.0]
    assert lost and all(vx == 0.0 and w == 0.0 for _, vx, w, _ in lost)
    assert r.f.state == "done" and r.f.reason is None and not r.session.emergency_stop


def _driving_rig():
    r = Rig()
    r.path([[3.0, 0.0]])
    p = r.seq
    r.run(2.0)
    assert r.f.state == "drive" and r.target[0] > 0
    return r, p


def test_existing_stop_protections_still_cancel_path():
    cases = {
        "estop": lambda r: r.send(ev("emergency_stop", r.nseq())),
        "joystick": lambda r: r.send(cmd(r.nseq(), active=True, fwd=0.5)),
        "disconnect": lambda r: r.send(ev("disconnect", r.nseq())),
        "arbiter": lambda r: r.srv.set_motion_permitted(False, "시험 차단"),
        "watchdog": lambda r: r.run(0.6, heartbeat=False, stop_when_idle=False),
    }
    for name, act in cases.items():
        r, p = _driving_rig()
        act(r)
        r.run(0.1, heartbeat=False, stop_when_idle=False)
        assert r.f.state == "cancelled", name
        if name != "joystick":
            assert r.log[-1][1:3] == (0.0, 0.0), name
        assert r.ident() == (CID, p), name
    r, _ = _driving_rig()
    r.send(ev("emergency_stop", r.nseq()))
    assert r.session.emergency_stop and r.session.state == "emergency_stop"    # 비상정지는 경로 취소와 별개 상태
