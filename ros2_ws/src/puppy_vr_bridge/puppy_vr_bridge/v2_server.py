"""Protocol v2 UDP 서버 (ROS 비의존). ROS2 노드와 단독 시험 실행기가 함께 쓴다.

- 데이터그램 1개 = UTF-8 JSON 객체 1개, ACK 는 보낸 주소로 같은 소켓에서 응답
- 수신 스레드 하나가 패킷 처리와 watchdog 검사를 같은 lock 아래에서 수행한다
- 별도 감시 스레드가 수신 스레드 정지·지연을 감지하면 정지 출력을 낸다
- 고주파 패킷을 디스크에 모두 쓰지 않도록 통계는 메모리에 누적하고 요약만 노출한다
"""

import logging
import socket
import threading
import time
from collections import Counter, deque
from typing import Callable, Optional, Tuple

from .protocol_v2 import MAX_PACKET_BYTES, STOP, V2Session, encode_ack

LOGGER = logging.getLogger("puppy_vr_bridge.v2_server")

OutputCallback = Callable[[Tuple[float, float, float], str], None]


class RxStats:
    """수신 주기·거부 사유·순서 역전 등 요약 통계 (최근 N개 간격만 보관)."""

    def __init__(self, window: int = 400):
        self.packets = 0
        self.accepted = 0
        self.reasons = Counter()
        self.bytes = 0
        self._last_rx = None  # type: Optional[float]
        self._intervals = deque(maxlen=window)
        self._last_seq = {}  # client_id -> 마지막 관측 sequence(수락 여부 무관)
        self.gaps = 0  # 같은 세션에서 sequence 가 2 이상 건너뛴 횟수(유실 추정)
        self.missing = 0  # 건너뛴 번호 합(유실 패킷 수 추정)
        self.reordered = 0

    def observe(self, now: float, size: int, ack: dict):
        self.packets += 1
        self.bytes += size
        if self._last_rx is not None:
            self._intervals.append(now - self._last_rx)
        self._last_rx = now
        reason = ack.get("reason", "?")
        self.reasons[reason] += 1
        if ack.get("accepted"):
            self.accepted += 1
        cid, seq = ack.get("client_id"), ack.get("sequence", -1)
        if cid and isinstance(seq, int) and seq >= 0:
            last = self._last_seq.get(cid)
            if last is not None:
                if seq > last + 1:
                    self.gaps += 1
                    self.missing += seq - last - 1
                elif seq <= last:
                    self.reordered += 1
            if last is None or seq > last:
                self._last_seq[cid] = seq
            if len(self._last_seq) > 64:
                self._last_seq.pop(next(iter(self._last_seq)))

    def summary(self, now: float) -> dict:
        iv = sorted(self._intervals)
        def pct(p):
            if not iv:
                return None
            return iv[min(len(iv) - 1, int(round(p * (len(iv) - 1))))]
        mean = sum(iv) / len(iv) if iv else None
        return {
            "packets": self.packets,
            "accepted": self.accepted,
            "reasons": dict(self.reasons),
            "bytes": self.bytes,
            "rate_hz": (1.0 / mean) if mean else None,
            "interval_mean_s": mean,
            "interval_p50_s": pct(0.5),
            "interval_p95_s": pct(0.95),
            "interval_max_s": iv[-1] if iv else None,
            "sequence_gaps": self.gaps,
            "estimated_lost": self.missing,
            "reordered_or_duplicate": self.reordered,
            "last_rx_age_s": (now - self._last_rx) if self._last_rx is not None else None,
        }


class V2UdpServer:
    def __init__(
        self,
        session: V2Session,
        on_output: OutputCallback,
        bind_address: str = "0.0.0.0",
        port: int = 5001,
        poll_interval: float = 0.02,
        on_estop: Optional[Callable[[bool], None]] = None,
        on_state: Optional[Callable[[dict], None]] = None,
        stall_timeout: float = 0.5,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.session = session
        self._on_output = on_output
        self._on_estop = on_estop
        self._on_state = on_state
        self._bind = bind_address
        self._port = int(port)
        self._poll = float(poll_interval)
        self._stall_timeout = float(stall_timeout)
        self._clock = clock
        self.lock = threading.RLock()
        self.stats = RxStats()
        self._socket = None
        self._rx_thread = None
        self._monitor_thread = None
        self._running = threading.Event()
        self._loop_heartbeat = 0.0
        self._last_state_key = None
        self._last_invalid_log = 0.0
        self._last_output = STOP

    # ── 수명 ──

    def start(self):
        if self._running.is_set():
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # SO_REUSEADDR 금지: Linux 에서 두 서버가 같은 UDP 포트를 나눠 받게 된다.
        try:
            sock.bind((self._bind, self._port))
        except OSError:
            sock.close()
            raise
        sock.settimeout(self._poll)
        self._socket = sock
        self._running.set()
        self._emit(STOP, "서버 시작")
        self._loop_heartbeat = self._clock()
        self._rx_thread = threading.Thread(target=self._receive_loop, name="v2-udp-rx", daemon=True)
        self._rx_thread.start()
        self._monitor_thread = threading.Thread(target=self._monitor_loop, name="v2-udp-monitor", daemon=True)
        self._monitor_thread.start()
        LOGGER.info("Protocol v2 UDP 수신: %s:%d", self._bind, self.bound_port)
        self._notify_state()

    @property
    def bound_port(self) -> int:
        sock = self._socket
        return int(sock.getsockname()[1]) if sock is not None else self._port

    def shutdown(self):
        if not self._running.is_set():
            return
        self._running.clear()
        sock, self._socket = self._socket, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        for thread in (self._rx_thread, self._monitor_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=1.0)
        with self.lock:
            self.session.force_stop()
        self._emit(STOP, "서버 종료")

    # ── 외부 입력 (중재기) ──

    def set_motion_permitted(self, permitted: bool, reason: Optional[str] = None):
        with self.lock:
            changed = self.session.set_motion_permitted(permitted, reason)
        if changed:
            self._emit(STOP, "중재기: %s" % (reason or "VR 이동 차단"))
        self._notify_state()

    def preempt(self, reason: str):
        with self.lock:
            self.session.preempt()
        self._emit(STOP, "선점 정지: %s" % reason)
        self._notify_state()

    def set_external_estop(self, latched: bool):
        with self.lock:
            before = self.session.emergency_stop
            self.session.set_external_estop(latched)
            after = self.session.emergency_stop
        if before != after:
            self._emit(STOP, "외부 비상정지 %s" % ("latch" if after else "해제"))
            self._notify_state()

    def check_timeouts(self):
        """수신 스레드 밖(ROS 타이머 등)에서도 만료를 검사할 수 있게 공개한다."""
        with self.lock:
            events = self.session.check_timeouts(self._clock())
        if events:
            self._emit(STOP, "; ".join(events))
            for event in events:
                LOGGER.warning("%s; 로봇 정지", event)
            self._notify_state()
        return events

    def snapshot(self) -> dict:
        with self.lock:
            snap = self.session.snapshot()
        snap["stats"] = self.stats.summary(self._clock())
        snap["port"] = self.bound_port
        return snap

    # ── 내부 ──

    def _receive_loop(self):
        # 한 패킷의 예외가 이 스레드를 끝내면 watchdog 도 멈추므로 각각 보호한다.
        while self._running.is_set():
            self._loop_heartbeat = self._clock()
            sock = self._socket
            if sock is None:
                break
            try:
                data, address = sock.recvfrom(MAX_PACKET_BYTES + 1)
            except socket.timeout:
                self._guarded(self.check_timeouts)
                continue
            except OSError as exc:
                if not self._running.is_set():
                    break
                LOGGER.warning("v2 UDP 수신 오류: %s", exc)
                self._guarded(self.check_timeouts)
                time.sleep(0.01)
                continue
            try:
                self._handle(data, address)
            except Exception:
                LOGGER.exception("v2 패킷 처리 중 예기치 않은 오류")
                with self.lock:
                    self.session.force_stop()
                self._emit(STOP, "패킷 처리 오류")
            self._guarded(self.check_timeouts)

    def _monitor_loop(self):
        """수신 루프가 멈추면(교착·예외 종료) 정지를 낸다. 소켓·스레드 고장 방어."""
        stalled = False
        while self._running.is_set():
            time.sleep(min(0.1, self._stall_timeout / 3.0))
            if not self._running.is_set():
                break  # 정상 종료 중
            age = self._clock() - self._loop_heartbeat
            alive = self._rx_thread is not None and self._rx_thread.is_alive()
            if not alive:
                # 수신 스레드가 끝났으면 재시작 전까지 이동을 막는다(fault latch).
                if not stalled:
                    stalled = True
                    LOGGER.error("v2 수신 스레드 종료 감지; fault latch")
                    with self.lock:
                        self.session.enter_fault("수신 스레드 종료")
                    self._emit(STOP, "수신 스레드 종료")
                    self._notify_state()
            elif age > self._stall_timeout:
                # 일시 지연(CPU 부족 등): 정지 후 새 중립을 요구하고 계속 운영한다.
                if not stalled:
                    stalled = True
                    LOGGER.error("v2 수신 루프 %.3fs 지연; 안전 정지", age)
                    with self.lock:
                        self.session.force_stop()
                    self._emit(STOP, "수신 루프 지연")
                    self._notify_state()
            else:
                stalled = False

    def _guarded(self, fn):
        try:
            fn()
        except Exception:
            LOGGER.exception("watchdog 검사 오류")
            with self.lock:
                self.session.force_stop()
            self._emit(STOP, "watchdog 검사 오류")

    def _handle(self, data: bytes, address):
        estop_change = None
        with self.lock:
            now = self._clock()
            result = self.session.handle_datagram(data, address, now)
            ack = result.ack
            estop_change = result.estop_change
            self.stats.observe(now, len(data), ack)
            if result.velocity is not None:
                self._emit(result.velocity, result.event or "command")
        self._send(address, ack)
        if estop_change is not None and self._on_estop is not None:
            try:
                self._on_estop(estop_change)
            except Exception:
                LOGGER.exception("비상정지 전달 실패")
        if result.event and "만료" in result.event:
            LOGGER.warning("%s", result.event)
        if not ack["accepted"]:
            self._log_invalid(address, "%s (%s)" % (ack["reason"], result.event or "-"))
        elif result.event is not None:
            LOGGER.warning("%s", result.event)
        self._notify_state()

    def _emit(self, velocity, reason: str):
        self._last_output = velocity
        try:
            self._on_output(velocity, reason)
        except Exception:
            LOGGER.exception("출력 콜백 실패")

    def _send(self, address, ack: dict):
        sock = self._socket
        if sock is None:
            return
        try:
            sock.sendto(encode_ack(ack), address)
        except (OSError, ValueError):
            pass

    def _notify_state(self):
        callback = self._on_state
        if callback is None:
            return
        with self.lock:
            snap = self.session.snapshot()
        key = (snap["state"], snap["emergency_stop"], snap["owner"], snap["fault"],
               snap["motion_permitted"])
        if key == self._last_state_key:
            return
        self._last_state_key = key
        try:
            callback(snap)
        except Exception:
            LOGGER.exception("상태 알림 실패")

    def _log_invalid(self, address, reason: str):
        now = self._clock()
        if now - self._last_invalid_log >= 1.0:
            self._last_invalid_log = now
            LOGGER.warning("v2 패킷 거부 %s:%d: %s", address[0], address[1], reason)

