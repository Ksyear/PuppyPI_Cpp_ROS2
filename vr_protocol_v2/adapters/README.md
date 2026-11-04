# Protocol v2 → ROS 2 어댑터

실제 구현: `ros2_ws/src/puppy_vr_bridge/puppy_vr_bridge/receiver_node.py`, 설정: `config/vr_bridge.yaml`

| 단계 | 값 | 단위 |
|---|---|---|
| 앱 `forward`, `turn` | -1 ~ 1 | 단위 없음(앱이 속도 비율을 곱해 보낸다) |
| 수신기 변환 `x` (전진) | `forward × max_forward_cm_s` (25) | cm/s |
| 수신기 변환 `x` (후진) | `forward × reverse_scale_cm_s` (10) 뒤 `-max_reverse_cm_s` (5) 로 제한 | cm/s |
| 수신기 변환 `yaw` | `turn × max_yaw_rate_rad_s` (0.28) | rad/s, +좌회전(REP-103) |
| 출발 대기 | 정지 상태에서 출발할 때만 `start_hold_sec` (0.2) 뒤 출력. 정지는 즉시 | s |
| 가감속 제한 | 0 = 끔 | |
| ROS 2 출력 `/puppy_control/vr/cmd_vel` | `TwistStamped.linear.x = x/100`, `angular.z = yaw` | m/s, rad/s |

`/puppy_control/vr/cmd_vel` 을 받아 실제로 걷게 하는 보행 제어 노드는 이 저장소에 없다(README 참고).
보행 엔진 허용 범위(|x|≤35 cm/s, |yaw|≤51°/s)를 넘는 설정은 수신기가 시작을 거부한다.
