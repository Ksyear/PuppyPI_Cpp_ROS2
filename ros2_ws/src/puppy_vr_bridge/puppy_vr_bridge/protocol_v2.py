"""Quest Protocol v2 순수 로직 (ROS·소켓·스레드 비의존).

출처: 이 저장소의 이전 버전(Ksyear, Apache-2.0) ``puppypi_remote_camera/robot/remote_control.py``
(``parse_v2_packet``, ``V2Session``)를 ROS2 수신기에서 재사용하도록 옮겼다.
변경 사항(각 항목은 tests/test_protocol_v2.py 에 시험):

1. 패킷 적용 전 watchdog·lease 만료 검사 (VR 저장소
   ``patches/robot-watchdog-before-command.patch`` 와 같은 수정).
   만료 직후 도착한 패킷이 만료 구간을 지우고 이동을 이어가던 결함을 막는다.
2. ``accept_estop_from_any_client``: 소유자가 아닌 유효 클라이언트의
   ``emergency_stop`` 도 latch 한다(안전 정지 우선). 기본 True.
3. ``set_motion_permitted``: 로봇 측 제어권 중재기(puppy_control)가 다른 제어
   경로(기존 앱·조이스틱)에 이동 제어권을 준 동안 활성 명령을 ``not_owner`` 로
   거부한다. wire 형식(필드·reason 목록)은 바꾸지 않는다.
4. ``preempt``: 다른 경로의 정지 요청·비상정지로 VR 이동이 끊기면 새 중립을
   요구한다(``force_stop`` 과 같은 효과, 소유권 유지).

wire 형식은 VR 저장소 inrjin/PuppyPi_MR_Controller@c63e7c3 의
``V2Session.cs``/``V2AckParser.cs`` 가 기대하는 8필드 ACK 와 정확히 일치해야 한다.
"""

import json
import math
import uuid
from collections import OrderedDict
from dataclasses import dataclass
from typing import List, Optional, Tuple

MAX_PACKET_BYTES = 2048
STOP = (0.0, 0.0, 0.0)

PROTOCOL_V2 = 2
V2_MAX_SEQUENCE = 9007199254740991  # 2**53 - 1, C# long 이지만 JS/double 안전 정수로 제한
V2_COMMON_FIELDS = frozenset({"protocol", "type", "client_id", "sequence"})
V2_COMMAND_FIELDS = V2_COMMON_FIELDS | {"control_active", "forward", "turn"}
V2_EVENT_TYPES = frozenset({"emergency_stop", "clear_emergency", "disconnect"})
V2_STATES = (
    "idle",
    "active",
    "timeout",
    "emergency_stop",
    "awaiting_neutral",
    "fault",
)
V2_REASONS = (
    "ok",
    "invalid_packet",
    "unsupported_protocol",
    "out_of_order",
    "not_owner",
    "emergency_stop",
    "neutral_required",
    "fault",
)
_NIL_UUID = "00000000-0000-0000-0000-000000000000"


class PacketError(ValueError):
    """제어 datagram 을 안전하게 쓸 수 없을 때."""


class V2PacketError(PacketError):
    """거부 사유 코드와, 해석 가능했던 경우 ACK 에 되돌릴 식별자."""

    def __init__(self, reason: str, message: str, client_id=None, sequence=-1):
        super().__init__(message)
        self.reason = reason
        self.client_id = client_id
        self.sequence = sequence


@dataclass(frozen=True)
class V2Packet:
    action: str
    client_id: str  # ACK 에 그대로 되돌리는 수신 문자열
    session_id: str  # 비교용 소문자 UUID
    sequence: int
    control_active: bool = False
    forward: float = 0.0
    turn: float = 0.0

    @property
    def is_neutral(self) -> bool:
        """control_active=false 이거나 두 축이 0 이면 이동 목표가 0 인 중립."""
        return not self.control_active or (self.forward == 0.0 and self.turn == 0.0)


def _reject_json_constant(value):
    raise PacketError("JSON 상수 %s는 허용되지 않습니다" % value)


def _reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PacketError("중복 JSON 키: %s" % key)
        result[key] = value
    return result


def _decode_strict_json(data: bytes):
    if not data or len(data) > MAX_PACKET_BYTES:
        raise PacketError("패킷 크기가 올바르지 않습니다")
    try:
        return json.loads(
            data.decode("utf-8"),
            parse_constant=_reject_json_constant,
            object_pairs_hook=_reject_duplicate_keys,
        )
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        # JSONDecodeError, PacketError, 과도한 정수 자릿수는 모두 ValueError 이다.
        raise PacketError("JSON 디코딩 실패: %s" % exc)


def _canonical_uuid(value) -> Optional[str]:
    """하이픈 포함 36자 표준 UUID 만 허용하고 nil UUID 는 거부한다."""
    if type(value) is not str or len(value) != 36:
        return None
    try:
        text = str(uuid.UUID(value))
    except ValueError:
        return None
    if text != value.lower() or text == _NIL_UUID:
        return None
    return text


def _is_v2_sequence(value) -> bool:
    # bool 은 int 의 하위 타입이므로 type(...) is int 로 정확히 비교한다.
    return type(value) is int and 0 <= value <= V2_MAX_SEQUENCE


def _unit_axis(value) -> Optional[float]:
    """-1~1 유한한 JSON 숫자만 허용한다. bool·문자열·NaN·Inf 는 거부한다."""
    if type(value) is int:
        # 큰 정수를 float 로 바꾸면 OverflowError 가 나므로 범위를 먼저 비교한다.
        return float(value) if -1 <= value <= 1 else None
    if type(value) is float and math.isfinite(value) and -1.0 <= value <= 1.0:
        return value
    return None


def parse_v2_packet(data: bytes) -> V2Packet:
    """엄격한 v2 datagram 하나를 해석한다. 누락·추가·중복·자료형 오류는 거부."""
    try:
        payload = _decode_strict_json(data)
    except PacketError as exc:
        raise V2PacketError("invalid_packet", str(exc))
    if not isinstance(payload, dict):
        raise V2PacketError("invalid_packet", "JSON 객체만 허용됩니다")

    session_id = _canonical_uuid(payload.get("client_id"))
    echo_id = payload["client_id"] if session_id is not None else None
    raw_sequence = payload.get("sequence")
    echo_sequence = raw_sequence if _is_v2_sequence(raw_sequence) else -1

    def reject(reason, message):
        return V2PacketError(reason, message, echo_id, echo_sequence)

    protocol = payload.get("protocol")
    if type(protocol) is not int:
        raise reject("invalid_packet", "protocol은 정수여야 합니다")
    if protocol != PROTOCOL_V2:
        raise reject("unsupported_protocol", "protocol %d은 지원하지 않습니다" % protocol)

    action = payload.get("type")
    if type(action) is not str:
        raise reject("invalid_packet", "type은 문자열이어야 합니다")
    if action == "command":
        required = V2_COMMAND_FIELDS
    elif action in V2_EVENT_TYPES:
        required = V2_COMMON_FIELDS
    else:
        raise reject("invalid_packet", "알 수 없는 type")

    fields = set(payload)
    if fields != required:
        raise reject(
            "invalid_packet",
            "필드 불일치: 누락=%s 추가=%s"
            % (sorted(required - fields), sorted(fields - required)),
        )
    if session_id is None:
        raise reject("invalid_packet", "client_id는 표준 UUID 문자열이어야 합니다")
    if echo_sequence == -1:
        raise reject("invalid_packet", "sequence는 0~%d 정수여야 합니다" % V2_MAX_SEQUENCE)

    if action != "command":
        return V2Packet(action, echo_id, session_id, raw_sequence)

    control_active = payload["control_active"]
    if type(control_active) is not bool:
        raise reject("invalid_packet", "control_active는 불리언이어야 합니다")
    forward = _unit_axis(payload["forward"])
    turn = _unit_axis(payload["turn"])
    if forward is None or turn is None:
        raise reject("invalid_packet", "forward/turn은 -1~1 유한한 숫자여야 합니다")
    return V2Packet(action, echo_id, session_id, raw_sequence, control_active, forward, turn)


def clamp_velocity(
    x: float,
    yaw_rate: float,
    max_forward: float,
    max_reverse: float,
    max_yaw_rate: float,
) -> Tuple[float, float, float]:
    """로봇 측 최종 상한. y 는 항상 0 (PuppyPi 보행 엔진은 횡이동 미지원)."""
    if not math.isfinite(x) or not math.isfinite(yaw_rate):
        raise ValueError("속도 값은 유한해야 합니다")
    limited_x = max(-max_reverse, min(max_forward, x))
    limited_yaw = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))
    return float(limited_x), 0.0, float(limited_yaw)


def normalized_to_velocity(
    forward: float,
    turn: float,
    max_forward: float,
    max_reverse: float,
    max_yaw_rate: float,
    reverse_scale: Optional[float] = None,
) -> Tuple[float, float, float]:
    """forward/turn(-1~1) → (x cm/s, y cm/s, yaw_rate rad/s).

    forward>=0: x = forward*max_forward, forward<0: x = forward*reverse_scale
    (없으면 max_reverse) 뒤 -max_reverse 로 제한. yaw_rate = turn*max_yaw_rate.
    turn>0 은 VR 에서 '왼쪽으로 기울임'이고 ROS REP-103 의 +yaw(반시계, 좌회전)와 같은 부호다.
    """
    if forward >= 0.0:
        scale = max_forward
    else:
        scale = max_reverse if reverse_scale is None else reverse_scale
    return clamp_velocity(
        forward * scale, turn * max_yaw_rate, max_forward, max_reverse, max_yaw_rate
    )


@dataclass(frozen=True)
class V2Result:
    ack: dict
    velocity: Optional[Tuple[float, float, float]]  # 지금 출력할 목표. None 이면 출력 없음
    event: Optional[str] = None  # 상태 전이 로그
    estop_change: Optional[bool] = None  # True=latch, False=해제, None=변화 없음


class V2Session:
    """v2 제어기 상태. 호출자가 monotonic 시각을 넘기고 모든 I/O 를 담당한다.

    소유권(단일 조종자), sequence, 중립 확인, 비상정지 latch, watchdog, lease 를
    관리한다. 가짜 시계로 시험한다.
    """

    def __init__(
        self,
        max_forward: float,
        max_reverse: float,
        max_yaw_rate: float,
        command_timeout: float,
        lease_timeout: float,
        retired_limit: int = 1024,
        accept_estop_from_any_client: bool = True,
        reverse_scale: Optional[float] = None,
    ):
        if lease_timeout <= command_timeout:
            raise ValueError("lease_timeout 은 command_timeout 보다 커야 합니다")
        self._max_forward = max_forward
        self._max_reverse = max_reverse
        self._reverse_scale = reverse_scale  # None = max_reverse (이전 동작)
        self._max_yaw = max_yaw_rate
        self._command_timeout = command_timeout
        self._lease_timeout = lease_timeout
        self._retired_limit = retired_limit
        self._estop_any = accept_estop_from_any_client

        self._owner_id = None  # type: Optional[str]
        self._owner_address = None
        self._last_sequence = -1
        self._last_owner_activity = 0.0
        self._last_command_rx = None  # type: Optional[float]
        self._neutral_confirmed = False
        self._timed_out = False
        self._active = False
        self._emergency = False
        self._fault = None  # type: Optional[str]
        self._target = STOP
        self._last_reason = "ok"
        self._motion_permitted = True
        self._blocked_reason = None  # type: Optional[str]
        # lease 만료로 풀린 직전 조종자. 다른 UUID 가 인수하기 전까지만 이어서
        # 재획득할 수 있고, 이때도 마지막 sequence 보다 큰 번호가 필요하다.
        self._previous_owner = None  # type: Optional[Tuple[str, int]]
        # disconnect 했거나 다른 UUID 에게 소유권이 넘어간 세션. 지연 패킷으로
        # 소유권이 되돌아가지 않도록 프로세스 실행 동안 거부한다.
        self._retired = OrderedDict()  # type: OrderedDict

    # ── 조회 ──

    @property
    def state(self) -> str:
        if self._fault is not None:
            return "fault"
        if self._emergency:
            return "emergency_stop"
        if self._timed_out:
            return "timeout"
        if not self._neutral_confirmed:
            return "awaiting_neutral"
        return "active" if self._active else "idle"

    @property
    def emergency_stop(self) -> bool:
        return self._emergency

    @property
    def owner(self) -> Optional[str]:
        return self._owner_id

    @property
    def target(self) -> Tuple[float, float, float]:
        return self._target

    def snapshot(self) -> dict:
        return {
            "state": self.state,
            "emergency_stop": self._emergency,
            "owner": self._owner_id,
            "owner_address": (
                "%s:%d" % self._owner_address if self._owner_address else None
            ),
            "last_sequence": self._last_sequence,
            "last_reason": self._last_reason,
            "fault": self._fault,
            "motion_permitted": self._motion_permitted,
            "blocked_reason": self._blocked_reason,
            "x_cm_s": self._target[0],
            "y_cm_s": self._target[1],
            "yaw_rate_rad_s": self._target[2],
        }

    # ── 외부(중재기) 입력 ──

    def set_motion_permitted(self, permitted: bool, reason: Optional[str] = None) -> bool:
        """다른 제어 경로가 이동 제어권을 가진 동안 False. 목표가 바뀌면 True 반환."""
        self._motion_permitted = bool(permitted)
        self._blocked_reason = None if permitted else reason
        if not permitted and self._target != STOP:
            self._reset_motion()
            return True
        return False

    def preempt(self) -> None:
        """다른 경로의 정지 요청. 소유권은 유지하고 새 중립을 요구한다."""
        self._reset_motion()

    def set_external_estop(self, latched: bool) -> None:
        """로봇 전체 비상정지 상태를 세션 latch 에 반영한다(VR 이 아닌 경로의 latch 포함)."""
        if latched and not self._emergency:
            self._emergency = True
            self._reset_motion()
        elif not latched and self._emergency:
            self._emergency = False
            self._reset_motion()

    # ── 패킷 처리 ──

    def handle_datagram(self, data: bytes, address, now: float) -> V2Result:
        try:
            packet = parse_v2_packet(data)
        except V2PacketError as exc:
            expired = self.check_timeouts(now)
            return self._reject(
                exc.client_id,
                exc.sequence,
                exc.reason,
                STOP if expired else None,
                "; ".join(expired + [str(exc)]),
            )
        return self.handle_packet(packet, address, now)

    def handle_packet(self, packet: V2Packet, address, now: float) -> V2Result:
        # 만료 검사는 sequence·소유권·command 갱신보다 반드시 먼저 한다.
        expired = self.check_timeouts(now)
        result = self._handle_packet_current(packet, address, now)
        if expired:
            event = "; ".join(expired + ([result.event] if result.event else []))
            return V2Result(
                result.ack,
                STOP if result.velocity is None else result.velocity,
                event,
                result.estop_change,
            )
        return result

    def _handle_packet_current(self, packet: V2Packet, address, now: float) -> V2Result:
        sid = packet.session_id
        is_owner = self._owner_id is not None and sid == self._owner_id and address == self._owner_address

        # 안전 정지 우선: 소유자가 아닌 유효 클라이언트의 비상정지도 latch 한다.
        # 소유권·sequence 흐름은 바꾸지 않는다(다른 조종자의 번호 흐름 보호).
        if (
            packet.action == "emergency_stop"
            and self._estop_any
            and not is_owner
            and (self._owner_id is not None or sid in self._retired)
        ):
            was_latched = self._emergency
            self._emergency = True
            self._reset_motion()
            return V2Result(
                self._ack(packet.client_id, packet.sequence, True, "ok"),
                STOP,
                None if was_latched else "비상정지 latch (비소유자 %s)" % sid,
                None if was_latched else True,
            )

        if sid in self._retired:
            return self._reject(packet.client_id, packet.sequence, "not_owner", None,
                                "종료되었거나 교체된 세션")

        claimed = None
        if self._owner_id is None:
            if packet.action == "disconnect":
                self._retire(sid)
                if self._previous_owner is not None and self._previous_owner[0] == sid:
                    self._previous_owner = None
                return self._accept(packet, None, None)
            baseline = -1
            if self._previous_owner is not None and self._previous_owner[0] == sid:
                baseline = self._previous_owner[1]
            if packet.sequence <= baseline:
                return self._reject(packet.client_id, packet.sequence, "out_of_order",
                                    None, "sequence 역전 또는 중복")
            if self._previous_owner is not None and self._previous_owner[0] != sid:
                self._retire(self._previous_owner[0])
            self._previous_owner = None
            self._owner_id = sid
            self._owner_address = address
            self._last_sequence = baseline
            self._reset_motion()
            claimed = "조종자 획득: %s %s:%d" % (sid, address[0], address[1])
        elif not is_owner:
            return self._reject(packet.client_id, packet.sequence, "not_owner", None,
                                "다른 조종자가 제어 중")

        if packet.sequence <= self._last_sequence:
            return self._reject(packet.client_id, packet.sequence, "out_of_order", None,
                                "sequence 역전 또는 중복")
        self._last_sequence = packet.sequence
        self._last_owner_activity = now

        result = self._apply(packet, now)
        if claimed is not None:
            event = claimed if result.event is None else "%s; %s" % (claimed, result.event)
            result = V2Result(result.ack, result.velocity, event, result.estop_change)
        return result

    def _apply(self, packet: V2Packet, now: float) -> V2Result:
        if packet.action == "emergency_stop":
            was_latched = self._emergency
            self._emergency = True
            self._reset_motion()
            return self._accept(packet, STOP, None if was_latched else "비상정지 latch",
                                None if was_latched else True)
        if packet.action == "clear_emergency":
            was_latched = self._emergency
            self._emergency = False
            self._reset_motion()
            return self._accept(
                packet,
                STOP,
                "비상정지 해제; 새 중립 대기" if was_latched else None,
                False if was_latched else None,
            )
        if packet.action == "disconnect":
            self._retire(self._owner_id)
            self._release_owner()
            return self._accept(packet, STOP, "조종자 disconnect; 소유권 해제")

        if packet.is_neutral:
            event = None
            if self._fault is None and not self._emergency:
                if not self._neutral_confirmed:
                    event = "중립 확인"
                self._neutral_confirmed = True
                self._timed_out = False
                self._active = packet.control_active
                self._last_command_rx = now
            self._target = STOP
            return self._accept(packet, STOP, event)

        if self._fault is not None:
            return self._reject(packet.client_id, packet.sequence, "fault", STOP, None)
        if self._emergency:
            return self._reject(packet.client_id, packet.sequence, "emergency_stop", STOP, None)
        if not self._neutral_confirmed:
            return self._reject(packet.client_id, packet.sequence, "neutral_required", STOP, None)
        if not self._motion_permitted:
            # 다른 제어 경로가 이동 중. 중립은 위에서 계속 받아 watchdog 을 유지한다.
            self._target = STOP
            self._active = False
            self._last_command_rx = now
            return self._reject(packet.client_id, packet.sequence, "not_owner", STOP,
                                self._blocked_reason or "다른 제어 경로가 이동 제어권 보유")
        self._active = True
        self._target = normalized_to_velocity(
            packet.forward, packet.turn, self._max_forward, self._max_reverse, self._max_yaw,
            self._reverse_scale,
        )
        self._last_command_rx = now
        return self._accept(packet, self._target, None)

    # ── 시간 기반 정지 ──

    def check_timeouts(self, now: float) -> List[str]:
        """watchdog/lease 만료 시 정지해야 할 이유 목록."""
        events = []
        if (
            self._last_command_rx is not None
            and now - self._last_command_rx >= self._command_timeout
        ):
            self._reset_motion()
            self._timed_out = True
            events.append("%.3f초 command watchdog 만료" % self._command_timeout)
        if (
            self._owner_id is not None
            and now - self._last_owner_activity >= self._lease_timeout
        ):
            self._previous_owner = (self._owner_id, self._last_sequence)
            self._release_owner()
            events.append("%.3f초 조종자 lease 만료" % self._lease_timeout)
        return events

    def force_stop(self):
        """외부 정지(종료 등). 소유권은 유지하고 새 중립을 요구한다."""
        self._reset_motion()

    def enter_fault(self, reason: str):
        """재시작 전까지 이동 명령을 거부하는 fault latch."""
        self._fault = reason
        self._reset_motion()

    # ── 내부 ──

    def _reset_motion(self):
        self._target = STOP
        self._last_command_rx = None
        self._neutral_confirmed = False
        self._timed_out = False
        self._active = False

    def _release_owner(self):
        self._owner_id = None
        self._owner_address = None
        self._last_sequence = -1
        self._last_owner_activity = 0.0
        self._reset_motion()

    def _retire(self, session_id: str):
        self._retired[session_id] = True
        self._retired.move_to_end(session_id)
        while len(self._retired) > self._retired_limit:
            self._retired.popitem(last=False)

    def _ack(self, client_id, sequence, accepted, reason) -> dict:
        self._last_reason = reason
        return {
            "protocol": PROTOCOL_V2,
            "type": "ack",
            "client_id": client_id,
            "sequence": sequence,
            "accepted": accepted,
            "reason": reason,
            "emergency_stop": self._emergency,
            "state": self.state,
        }

    def _accept(self, packet, velocity, event, estop_change=None) -> V2Result:
        return V2Result(self._ack(packet.client_id, packet.sequence, True, "ok"),
                        velocity, event, estop_change)

    def _reject(self, client_id, sequence, reason, velocity, detail) -> V2Result:
        return V2Result(self._ack(client_id, sequence, False, reason), velocity, detail)


def encode_ack(ack: dict) -> bytes:
    """VR V2AckParser 가 받는 평평한 8필드 JSON. NaN 금지."""
    return json.dumps(ack, separators=(",", ":"), allow_nan=False).encode("utf-8")


class SlewLimiter:
    """목표 속도 가감속 제한. 정확한 정지(0,0)는 제한 없이 즉시 통과시킨다."""

    def __init__(self, max_linear_accel: float, max_yaw_accel: float, max_dt: float = 0.1):
        self._a_lin = max_linear_accel  # cm/s^2, 0 이하면 제한 없음
        self._a_yaw = max_yaw_accel  # rad/s^2
        # 오래 쉬었다가 처음 갱신할 때 큰 dt 로 한 번에 목표까지 뛰지 않도록 상한을 둔다.
        self._max_dt = max_dt
        self._out = (0.0, 0.0)
        self._last = None  # type: Optional[float]

    @property
    def output(self) -> Tuple[float, float]:
        return self._out

    def reset(self):
        self._out = (0.0, 0.0)
        self._last = None

    def update(self, target_x: float, target_yaw: float, now: float) -> Tuple[float, float]:
        if target_x == 0.0 and target_yaw == 0.0:
            self._out = (0.0, 0.0)
            self._last = now
            return self._out
        dt = 0.0 if self._last is None else min(self._max_dt, max(0.0, now - self._last))
        self._last = now
        x, yaw = self._out
        x = _approach(x, target_x, self._a_lin * dt if self._a_lin > 0 else None)
        yaw = _approach(yaw, target_yaw, self._a_yaw * dt if self._a_yaw > 0 else None)
        self._out = (x, yaw)
        return self._out


class StartHold:
    """정지 상태에서 출발할 때만 hold_sec 동안 0 을 유지한 뒤, 그때의 목표를 내보낸다.

    보행 엔진은 속도를 보행 주기 경계에서만 바꾼다. 엄지·손을 미는 0.3~0.4 s 동안 입력이
    작은 값부터 올라가므로, 첫 명령(예: 3 cm/s)이 한 주기 동안 고정되어 출발이 머뭇거린다.
    잠깐 기다렸다가 출발하면 첫 주기부터 목표에 가까운 속도로 걷는다.
    정지(0,0)는 기다리지 않고 즉시 통과한다. hold_sec <= 0 이면 끈다.
    """

    def __init__(self, hold_sec: float):
        self._hold = max(0.0, float(hold_sec))
        self._since = None  # type: Optional[float]
        self._moving = False

    @property
    def pending(self) -> bool:
        return self._since is not None and not self._moving

    def reset(self):
        self._since = None
        self._moving = False

    def update(self, target_x: float, target_yaw: float, now: float) -> Tuple[float, float]:
        if target_x == 0.0 and target_yaw == 0.0:
            self.reset()
            return (0.0, 0.0)
        if self._moving or self._hold <= 0.0:
            self._moving = True
            return (target_x, target_yaw)
        if self._since is None:
            self._since = now
        if now - self._since >= self._hold:
            self._moving = True
            return (target_x, target_yaw)
        return (0.0, 0.0)


def _approach(current: float, target: float, max_step: Optional[float]) -> float:
    if max_step is None:
        return target
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)
