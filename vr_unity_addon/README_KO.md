# VR(Unity)에서 로봇 카메라·LiDAR 지도 보기 — 설치 (PuppyPi Media v1)

대상: Protocol v2 로 PuppyPi 를 조종하는 Unity VR 앱(별도 저장소, 이 저장소에 없음). Unity 6000.0, URP 17, PC + Quest Link 에서 작성.
규격: README 의 'PuppyPi Media v1'. 조종(Protocol v2) 코드는 바꾸지 않는다.

## 1. 파일 복사 (2개 추가)

| 이 폴더 | VR 앱 프로젝트 | |
|---|---|---|
| `Assets/PuppyPi/MjpegStreamReader.cs` | `Assets/PuppyPi/` | HTTP MJPEG 수신(백그라운드 스레드, Unity 비의존) |
| `Assets/PuppyPi/PuppyPiStreamView.cs` | `Assets/PuppyPi/` | 헤드셋 안 패널 2개 |

`PuppyPiStreamView` 는 VR 앱의 `PuppyPiV2Client`(로봇 주소)를 참조한다. 씬의 오브젝트에 붙이거나, 기존 스크립트에서 `AddComponent<PuppyPiStreamView>()` 로 붙인다.

## 2. 사용

1. 지금처럼 Play → 연결 패널에서 기기 검색 → 연결.
2. 연결되면 콘솔 앞에 **CAMERA**(가운데 위)와 **LIDAR MAP**(오른쪽) 패널이 뜬다. PC 화면 오른쪽 아래에도 작게 보인다.
3. 왼쪽 컨트롤러 **X** = 패널 켜기/끄기. (B 비상정지 · Y 해제는 그대로)
4. 패널 왼쪽 위 글자: `NO SIGNAL …` 이면 2 s 넘게 프레임 없음(로봇 카메라·LiDAR·Wi-Fi 확인), `NOT CONNECTED` 면 조종 연결 전.

## 3. 조정 (Inspector, `PuppyPiStreamView`)

| 항목 | 기본 | 설명 |
|---|---|---|
| maxCameraFps / maxMapFps | 15 / 2 | 메인 스레드 JPEG 해독 상한(VR 프레임 예산). 버벅이면 10 으로 |
| cameraPanelOffset / Width | (0, 0.20, 0.60) / 0.50 m | 콘솔(받침대) 기준 위치·폭 |
| mapPanelOffset / Width | (0.40, 0.14, 0.50) / 0.28 m | |
| panelMaterial | 없음 | 비우면 `Universal Render Pipeline/Unlit` 을 찾는다. **PC 빌드(.exe)** 로 낼 때는 이 칸에 URP Unlit 머티리얼을 넣거나 Graphics 설정 Always Included Shaders 에 추가(에디터 Play 는 그대로 됨) |

## 4. 확인한 것 / 안 한 것

- 수신기(`MjpegStreamReader`)를 .NET 8 로 컴파일해 **실제 로봇과 같은 web_video_server 3.1.0** 스트림에서 30 fps 수신·자동 재접속 확인.
- `PuppyPiStreamView` 는 Unity API 형식 검사만 했다. **Unity 에디터·Quest 에서 실행하지 않았다** — 첫 실행 때 패널 위치·크기·셰이더(분홍색이면 위 panelMaterial)를 확인할 것.
