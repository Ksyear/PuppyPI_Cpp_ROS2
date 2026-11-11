# PuppyPi 원격 조종 — ROS 2 로봇 쪽 구성요소

Robot-side ROS 2 (Humble) components for teleoperating a PuppyPi quadruped from a VR headset or phone:
a UDP "Protocol v2" receiver with safety rules, an LD06 LiDAR driver with odometry-free mapping, and a small
media server for camera/map streams. The manufacturer's motion stack is **not** included (see below).
Licensed under Apache-2.0.

## 들어 있는 것

| 위치 | 내용 |
|---|---|
| `ros2_ws/src/puppy_vr_bridge/` | Protocol v2 수신기(UDP 5001), 경로 지정 주행(`path_follower.py`), 카메라 UDP 송신기(5006, 받는 앱이 있을 때만 실행) |
| `ros2_ws/src/puppy_lidar/` | LDROBOT LD06 드라이버(`/dev/ttyAMA0`, 230400 baud) → `/scan`, 오도메트리 없는 2D 점유격자 지도작성 |
| `ros2_ws/src/puppy_bringup/` | PuppyPi Media v1 서버(포트 8090), USB 카메라 모드 선택기 `puppypi-camera-detect` |
| `vr_protocol_v2/` | Protocol v2 JSON 스키마(command·event·ack·path), ROS 없이 수신기를 띄우는 실행기, 단위 변환표 |
| `config/` | 로봇 설정 예(`/etc/puppypi/` 에 둔다): `vr_bridge.yaml`, `lidar_mapper.yaml`, `camera.env`, `ros2.env` |
| `vr_unity_addon/` | Unity VR 앱에서 로봇 카메라·LiDAR 지도를 패널로 보는 스크립트 2개 |
| `tests/` | ROS 없이 도는 pytest 시험 |

## 들어 있지 않은 것

- **보행 제어 노드와 보행 엔진.** 실제로 다리를 움직이는 코드, 서보 보정값, 동작 그룹 파일, 프리셋 자세 값은 로봇
  제조사 소프트웨어에 있다. 이 저장소에는 그 코드·값·파일과 그것을 옮긴 코드가 없다.
- 역공학 자료, OS 이미지, 네트워크·계정 설정.
- 휴대폰 조종 앱, 맥북 조종기, VR 앱(별도 저장소), 로봇 검색 응답기(UDP 5002). 앱에는 로봇 주소를 직접 입력한다.

그래서 이 저장소만으로는 로봇이 걷지 않는다. 아래 인터페이스를 맞춘 보행 제어 노드가 따로 있어야 한다.

## 구성

```text
VR 앱 / 휴대폰 앱 ──UDP 5001 JSON (Protocol v2, 20 Hz)──▶ vr_protocol_v2_receiver
        ▲                                                  │  /puppy_control/vr/cmd_vel  (TwistStamped, m/s, rad/s)
        └────────────── ACK (같은 소켓, 즉시) ──────────────┤
                                                           ▼
                                   보행 제어 노드(이 저장소에 없음) ──▶ 보행 엔진 ──▶ 서보
                                                           │  /puppy_control/arbiter_state (String, JSON)
                                                           └──▶ 수신기가 이동 허용 여부·비상정지를 따른다

USB 카메라 ─▶ usb_cam ─▶ web_video_server(8080, MJPEG) ─▶ VR 패널·브라우저
LD06 ─▶ ld06_driver(/scan) ─▶ lidar_mapper(/map, /lidar_map/image) ─▶ web_video_server
media_server(8090): 영상·지도 주소 안내, 지도 좌표 정보, 상태
```

### 보행 제어 노드가 맞춰야 할 인터페이스

| 방향 | 토픽 | 형식 |
|---|---|---|
| 수신기 → | `/puppy_control/vr/cmd_vel` | `geometry_msgs/TwistStamped`. `linear.x` m/s(+전진), `angular.z` rad/s(+좌회전, REP-103). 20 Hz, 정지는 즉시 0 |
| → 수신기 | `/puppy_control/arbiter_state` | `std_msgs/String` JSON. 수신기가 읽는 키: `vr_permitted`(bool), `vr_block_reason`, `vr_preempt_seq`, `last_preempt_reason`, `estop`(bool). 2 s 넘게 안 오면 이동 금지 |
| 수신기 → | `/puppy_control/estop_request` | `std_msgs/String` JSON `{"source": "vr", "latch": true/false}` |

QoS: `arbiter_state` 는 RELIABLE + TRANSIENT_LOCAL, depth 1.

## Protocol v2 요약

- UDP 5001, 데이터그램 하나 = JSON 객체 하나. 스키마: `vr_protocol_v2/schemas/`.
- 명령: `{"protocol":2,"type":"command","client_id":"<UUID>","sequence":n,"control_active":bool,"forward":-1~1,"turn":-1~1}`
- 이벤트: `type` = `emergency_stop` · `clear_emergency` · `disconnect` (공통 필드 4개).
- 응답(ACK): 필드 8개 `protocol, type, client_id, sequence, accepted, reason, emergency_stop, state`.
- 안전 규칙: 조종자 한 명(첫 클라이언트가 소유), sequence 증가만 수락, 연결 후 중립(`control_active:false`)을 받아야 출발,
  명령이 0.4 s 끊기면 정지(`command_timeout_sec`), 1.0 s 끊기면 소유권 해제(`client_lease_timeout_sec`),
  비상정지는 누가 보내도 걸리고 해제는 명시적으로만.
- 속도 변환(`config/vr_bridge.yaml`):

| 입력 | 계산 | 기본값 |
|---|---|---|
| `forward` > 0 | `forward × max_forward_cm_s` | 25 cm/s |
| `forward` < 0 | `forward × reverse_scale_cm_s` 뒤 `-max_reverse_cm_s` 로 제한 | ×10, 상한 5 cm/s |
| `turn` | `turn × max_yaw_rate_rad_s` | 0.28 rad/s |
| 출발 대기 | 정지 상태에서 출발할 때만 `start_hold_sec` 뒤 출력. 정지·감속·방향 전환은 바로 | 0.2 s |
| 회전 중 작은 전진·후진 | 회전 입력이 있고 \|x\| < `turn_x_deadband_cm_s` 이면 x = 0(제자리 회전) | 4 cm/s |

출발 대기를 두는 이유: 보행 엔진은 속도를 걸음 주기 단위로 바꾼다. 엄지나 손을 미는 동안 입력이 작은 값부터
올라가므로, 처음 받은 작은 속도로 한 주기를 걷고 나서야 빨라진다. 잠깐 기다렸다 출발하면 첫 주기부터 목표에
가까운 속도로 걷는다. 대신 출발 반응이 그만큼 늦어진다. 0 이면 끈다.

회전 중 작은 전진·후진을 무시하는 이유: 조이스틱을 좌우로 밀 때 손이 조금 위아래로 가면 '느린 전진 + 회전' 이
된다. 실기에서 이 조합으로 넘어지는 일이 관찰되어, 회전 중 작은 x 는 0 으로 바꿔 제자리 회전으로 돌게 했다.

## 경로 지정 주행 (Protocol v2 확장)

VR 앱이 LiDAR 지도 위에 찍은 경유점을 보내면 로봇이 차례로 따라간다. 조이스틱 직접 조종과 같은 수신기·같은
안전 규칙을 쓴다. 스키마: `vr_protocol_v2/schemas/path_v2.schema.json`.

- 경로 주행 중에도 VR 은 중립 명령(`forward:0, turn:0`)을 계속 보내 연결을 유지한다. 중립 명령은 경로를 취소하지 않는다.
- 경로: `{"protocol":2,"type":"path","client_id":"<UUID>","sequence":n,"points":[[x,y],...]}`
  - 지도 좌표(m, `/lidar_map/pose` 와 같은 좌표계), 점 1~20개, 각 좌표 절댓값 100 이하.
  - 조종자만, 연결 뒤 중립을 받은 뒤에만 수락. 새 경로는 이전 경로를 바꾼다. ACK 는 명령과 같은 8필드.
- 취소: `{"protocol":2,"type":"path_cancel","client_id":"<UUID>","sequence":n}`
- 움직임: 경유점 방향으로 제자리 회전 → 직진. 직진 중에는 작은 방향 보정만 한다(느린 전진 + 큰 회전 조합을 만들지 않음).
- 장애물: **로봇은 LiDAR 점만으로 멈추지 않는다.** 장애물 판단·경로 취소·우회는 VR 앱이 맡고, 장애물을 확정하면
  `path_cancel` 을 보낸다(2026-11-11 변경. 이전에는 몸 앞 점이 보이면 멈추고 1 s 비면 다시 갔다).
- 멈춤(경로 유지): 로봇 위치(`/lidar_map/pose`)나 `/scan` 이 0.6 s 넘게 끊길 때. 제자리 회전도 멈춘다.
  다시 들어오면 별도 해제 없이 이어 간다.
- 취소: 0 이 아닌 조이스틱 명령, 명령 끊김(watchdog·lease), 비상정지, 조종자 교체, 보행 제어 노드의 VR 이동 차단,
  `path_cancel`, 로봇 위치가 한 번에 0.25 m 또는 25° 넘게 바뀔 때(지도 초기화·위치 추정 튐).
- 속도: 기본 12.5 cm/s, 제자리 회전 0.14 rad/s. 설정은 `config/vr_bridge.yaml` 의 `path.*`.
- 상태: `http://<로봇>:8090/api/path` 의 `path`
  - `state` = idle·rotate·drive·done·cancelled (`blocked` 는 더 이상 나오지 않는다. `blocked_count` 는 형식 호환용으로 늘 0)
  - `request_client_id`·`request_sequence`: 실제로 적용한 `path` 패킷의 `client_id`·`sequence`. 적용 전·식별자 없는 경우 둘 다 `null`.
    ACK 시점이 아니라 경로를 실제로 적용할 때 바뀌고, done·cancelled 뒤에도 유지되며, 중립 명령·`path_cancel`·거부된
    요청으로는 바뀌지 않는다. 짧은 경로가 첫 조회 전에 끝나도 VR 이 "이번 요청의 완료" 인지 구분할 수 있다.
  - 그 밖에 `waiting`(pose·scan 끊김), `reason`, `points`, `index`, `count`(done 이면 index = count), `remaining_m`, `elapsed_s`.

한계:
- 위치는 LiDAR 스캔 맞추기로만 추정한다(오도메트리 없음). 특징이 적은 넓은 곳·긴 복도에서는 흔들릴 수 있다.
- LD06 은 한 평면(설치 높이)만 본다. 그보다 낮거나 높은 물체는 장애물로 잡지 못한다.
- 로봇 쪽 장애물 정지가 없으므로, VR 앱이 장애물 판단을 하지 않거나 `path_cancel` 을 보내지 못하면 로봇은 경로를 계속 간다.
  통신 자체가 끊기면 watchdog(0.4 s)이 멈춘다.

## PuppyPi Media v1

| 주소 | 내용 |
|---|---|
| `http://<로봇>:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed` | 카메라 MJPEG(web_video_server) |
| `http://<로봇>:8080/stream?topic=/lidar_map/image&type=ros_compressed` | LiDAR 지도 이미지 MJPEG |
| `http://<로봇>:8090/` | 보기 페이지(카메라 + 지도 + 상태) |
| `http://<로봇>:8090/api/media` | 위 주소·형식·주기 안내(JSON) |
| `http://<로봇>:8090/api/map` | 지도 이미지 좌표 정보(m ↔ 픽셀) |
| `http://<로봇>:8090/api/status` | 중재 상태·지도 위 로봇 자세 |
| `http://<로봇>:8090/api/path` | 경로 지정 주행 상태(위 절 참고) |

LiDAR 지도 이미지의 표시 방식은 `lidar_mapper` 의 `display_mode` 로 고른다.
`map`(노드 기본값)은 누적 지도를, `live` 는 최근 `live_window_sec` 초 안의 스캔과 로봇만 그린다.
`live` 에서도 위치 추정용 누적 지도는 내부에서 계속 쓰고 `/map` 으로 낸다. 설정 예: `config/lidar_mapper.yaml`.

## 요구 사항

- Raspberry Pi 4B, Ubuntu 22.04, ROS 2 Humble
- `puppy_lidar`: python3-serial, python3-numpy, python3-opencv
- 영상: `usb_cam`, `web_video_server`
- 시험(PC): Python 3.10 이상, pytest, numpy, opencv-python-headless

## 빌드와 실행

```bash
mkdir -p ~/ros2_ws && cp -r ros2_ws/src ~/ros2_ws/
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-select puppy_vr_bridge puppy_lidar puppy_bringup
source install/setup.bash

ros2 run puppy_vr_bridge vr_protocol_v2_receiver --ros-args --params-file /etc/puppypi/vr_bridge.yaml
ros2 run puppy_lidar ld06_driver
ros2 run puppy_lidar lidar_mapper --ros-args --params-file /etc/puppypi/lidar_mapper.yaml
ros2 run puppy_bringup media_server
```

ROS 없이 수신기만 확인: `python3 vr_protocol_v2/receiver/run_receiver.py --help`

## 시험

```bash
python3 -m pip install pytest numpy opencv-python-headless
python3 -m pytest tests
```

2026-11-11 기록: macOS(Python 3.13)에서 138개 통과. ROS 2 Humble 컨테이너(arm64)에서 세 패키지 빌드,
실제 수신기·미디어 서버를 가짜 위치·LiDAR·보행 제어 상태와 함께 띄워 경로 주행(잡음, 짧은 경로, 같은 좌표 재요청,
취소, 차단, 끊김, 비상정지)을 확인. 경로 지정 주행은 실제 로봇에서 이 공개본 그대로 시험한 것은 아니다.

## 안전

- 처음 실행·설정 변경 뒤에는 로봇을 받치거나 들어 올린 상태에서 먼저 확인한다.
- 사람 한 명은 늘 로봇 옆에서 전원 스위치를 맡는다. 소프트웨어 비상정지와 전원 차단은 다르다.
- 속도 상한(`max_*`)을 올리기 전에 로봇에서 관절 범위를 먼저 확인한다.

## 라이선스와 상표

Apache License 2.0 — `LICENSE`, `NOTICE` 참고.
PuppyPi 는 Hiwonder 의 제품명, LD06 은 LDROBOT 의 제품명, Meta Quest 는 Meta 의 상표다.
이 프로젝트는 이 회사들과 관계가 없고 승인을 받지 않았다.

코드와 문서 일부는 AI 도구(Claude)의 도움을 받아 작성했고, 사람이 검토했다.
