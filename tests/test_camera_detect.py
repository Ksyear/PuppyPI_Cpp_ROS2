"""USB 카메라 선택기(puppypi-camera-detect) 시험: 가짜 by-id·가짜 v4l2-ctl 로 실행."""
import importlib.machinery
import importlib.util
import os
import stat
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(ROOT, "ros2_ws", "src", "puppy_bringup", "scripts", "puppypi-camera-detect")
loader = importlib.machinery.SourceFileLoader("camdetect", SCRIPT)
spec = importlib.util.spec_from_loader("camdetect", loader)
cd = importlib.util.module_from_spec(spec)
loader.exec_module(cd)

SAMPLE = """ioctl: VIDIOC_ENUM_FMT
\tType: Video Capture

\t[0]: 'MJPG' (Motion-JPEG, compressed)
\t\tSize: Discrete 1920x1080
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t\t\tInterval: Discrete 0.067s (15.000 fps)
\t\tSize: Discrete 2592x1944
\t\t\tInterval: Discrete 0.067s (15.000 fps)
\t\tSize: Discrete 1280x720
\t\t\tInterval: Discrete 0.017s (60.000 fps)
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t[1]: 'YUYV' (YUYV 4:2:2)
\t\tSize: Discrete 640x480
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t\tSize: Discrete 1920x1080
\t\t\tInterval: Discrete 0.200s (5.000 fps)
"""


# 커널 5.15 v4l2-ctl --list-ctrls 형식. brightness 는 usb_cam 0.8.1 이 강제한 50 이 남아 있는 상태.
CTRLS = """
User Controls

                     brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=50
                       contrast 0x00980901 (int)    : min=0 max=255 step=1 default=32 value=32
                     saturation 0x00980902 (int)    : min=0 max=128 step=1 default=64 value=64
        white_balance_automatic 0x0098090c (bool)   : default=1 value=0
                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
      white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=6 step=1 default=3 value=3

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=1
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=157 flags=inactive
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=0
"""
OLD_CTRLS = """
                     brightness (int)    : min=-64 max=64 step=1 default=0 value=50
                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=50
 white_balance_temperature_auto 0x0098090c (bool)   : default=1 value=1
                  exposure_auto 0x009a0901 (menu)   : min=0 max=3 default=3 value=3
"""


def test_parse_ctrls_new_and_old_names():
    c = cd.parse_ctrls(CTRLS)
    assert c["brightness"] == {"type": "int", "inactive": False, "min": 0, "max": 255, "step": 1, "default": 128, "value": 50}
    assert c["white_balance_temperature"]["inactive"] is True and c["auto_exposure"]["value"] == 1
    old = cd.parse_ctrls(OLD_CTRLS)
    assert set(old) == {"brightness", "white_balance_temperature_auto", "exposure_auto"}


def test_plan_controls_resets_defaults_and_auto():
    plan = {n: (v, why) for n, v, why in cd.plan_controls(cd.parse_ctrls(CTRLS), {})}
    # usb_cam 이 남긴 brightness 50 → 카메라 기본 128, 자동 노출(1→3)·자동 WB(0→1). 이미 같은 값·inactive 는 건드리지 않음
    assert plan == {"brightness": (128, "카메라 기본값"), "auto_exposure": (3, "자동 노출"),
                    "white_balance_automatic": (1, "자동 화이트밸런스")}
    old = {n: v for n, v, _ in cd.plan_controls(cd.parse_ctrls(OLD_CTRLS), {})}
    assert old == {"brightness": 0}  # 옛 이름 카메라: exposure_auto·WB 는 이미 자동


def test_plan_controls_env_overrides():
    env = {"CTRL_brightness": "160", "CTRL_exposure_dynamic_framerate": "1", "CTRL_gain": "500",
           "CTRL_nope": "1", "CTRL_contrast": "abc", "CTRL_white_balance_temperature": "5000", "CTRL_sharpness": ""}
    plan = {n: (v, why) for n, v, why in cd.plan_controls(cd.parse_ctrls(CTRLS), env)}
    assert plan["brightness"] == (160, "camera.env") and plan["exposure_dynamic_framerate"] == (1, "camera.env")
    assert "gain" not in plan and "nope" not in plan and "contrast" not in plan  # 범위 밖·없는 이름·정수 아님
    assert "white_balance_temperature" not in plan  # inactive(자동 WB 중)는 설정 불가


def test_parse_modes():
    modes = cd.parse_modes(SAMPLE)
    assert ("MJPG", 1920, 1080, 30.0) in modes and ("MJPG", 2592, 1944, 15.0) in modes
    assert ("YUYV", 640, 480, 30.0) in modes and len(modes) == 7  # MJPG 5 + YUYV 2


def test_choose_best_mjpg_within_limits():
    modes = cd.parse_modes(SAMPLE)
    assert cd.choose_mode(modes, 1920, 1080, 15, 30) == ("mjpeg2rgb", 1920, 1080, 30.0)
    assert cd.choose_mode(modes, 1280, 720, 15, 30) == ("mjpeg2rgb", 1280, 720, 30.0)  # 60fps 는 MAX_FPS 초과
    assert cd.choose_mode(modes, 4000, 3000, 15, 30) == ("mjpeg2rgb", 2592, 1944, 15.0)


def test_yuyv_fallback_and_none():
    yuyv_only = [m for m in cd.parse_modes(SAMPLE) if m[0] == "YUYV"]
    assert cd.choose_mode(yuyv_only, 1920, 1080, 15, 30) == ("yuyv2rgb", 640, 480, 30.0)
    assert cd.choose_mode([], 1920, 1080, 15, 30) is None


def _jpeg(y_hv, sof=0xC0):
    """SOI + DQT(안에 FF C0 바이트 포함) + SOFn(Y, Cb, Cr) + EOI. 순진한 FF C0 검색이면 DQT 에서 잘못 읽는다."""
    dqt = b"\x00" + bytes([0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00]) + bytes(58)
    seg = lambda m, body: bytes([0xFF, m]) + (len(body) + 2).to_bytes(2, "big") + body
    sofb = bytes([8, 0x02, 0xD0, 0x05, 0x00, 3, 1, y_hv, 0, 2, 0x11, 1, 3, 0x11, 1])
    return b"\xff\xd8" + seg(0xE0, b"JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00") + seg(0xDB, dqt) + seg(sof, sofb) + b"\xff\xd9"


def test_jpeg_sampling_walks_segments():
    assert cd.jpeg_sampling(_jpeg(0x21)) == (2, 1)
    assert cd.jpeg_sampling(_jpeg(0x22)) == (2, 2)
    assert cd.jpeg_sampling(_jpeg(0x11, sof=0xC2)) == (1, 1)  # progressive
    assert cd.jpeg_sampling(b"\x00garbage" + _jpeg(0x22)) == (2, 2)  # 앞에 쓰레기가 있어도 SOI 부터
    assert cd.jpeg_sampling(b"not a jpeg") is None
    assert cd.jpeg_sampling(b"\xff\xd8\xff\xda\x00\x02") is None  # SOF 없이 스캔 시작
    assert cd.AV_FORMAT_BY_SAMPLING[(2, 1)] == "YUVJ422P" and cd.AV_FORMAT_BY_SAMPLING[(2, 2)] == "YUVJ420P"


def _layout(tmp_path, names):
    byid = tmp_path / "by-id"
    byid.mkdir()
    for i, n in enumerate(names):
        (tmp_path / ("video%d" % (2 * i))).write_text("")
        os.symlink(str(tmp_path / ("video%d" % (2 * i))), str(byid / n))
    return byid


def test_find_device_priority(tmp_path):
    byid = _layout(tmp_path, ["usb-Sonix_Cam-video-index0", "usb-Shenzhen_NV50-HD220S_1-video-index0"])
    v0 = str(tmp_path / "video0")
    assert cd.find_device({}, str(byid), v0).endswith("NV50-HD220S_1-video-index0")
    assert cd.find_device({"CAMERA_NAME": "Sonix"}, str(byid), v0).endswith("Sonix_Cam-video-index0")
    assert cd.find_device({"CAMERA_NAME": "Nope"}, str(byid), v0).endswith("NV50-HD220S_1-video-index0")  # 이름 불일치 → 정렬상 첫 장치
    assert cd.find_device({"CAMERA_DEVICE": v0}, str(byid), v0) == v0
    assert cd.find_device({"CAMERA_DEVICE": "/nonexistent"}, str(byid), v0) is None


def test_no_camera(tmp_path):
    (tmp_path / "by-id").mkdir()
    assert cd.find_device({}, str(tmp_path / "by-id"), str(tmp_path / "video0")) is None


FAKE_V4L2 = """#!{py}
import os, sys
sys.stdout.write({ctrls!r} if "--list-ctrls" in sys.argv else {sample!r})
log = os.environ["FAKE_V4L2_LOG"]
open(log, "a").write(" ".join(sys.argv[1:]) + "\\n")
for a in sys.argv[1:]:
    if a.startswith("--stream-to=") and os.environ.get("FAKE_JPEG"):
        open(a.split("=", 1)[1], "wb").write(open(os.environ["FAKE_JPEG"], "rb").read() * 3)
"""


def _run(tmp_path, env_text="", v4l2=True, y_hv=0x21):
    env = dict(os.environ)
    (tmp_path / "camera.env").write_text(env_text)
    env.update(PUPPYPI_CAMERA_ENV=str(tmp_path / "camera.env"), PUPPYPI_V4L_BYID=str(tmp_path / "by-id"),
               PUPPYPI_VIDEO0=str(tmp_path / "video0"), FAKE_V4L2_LOG=str(tmp_path / "v4l2.log"))
    if v4l2:
        fake = tmp_path / "v4l2-ctl"
        fake.write_text(FAKE_V4L2.format(py=sys.executable, sample=SAMPLE, ctrls=CTRLS))
        fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
        env["PUPPYPI_V4L2CTL"] = str(fake)
        if y_hv is not None:
            (tmp_path / "frame.jpg").write_bytes(_jpeg(y_hv))
            env["FAKE_JPEG"] = str(tmp_path / "frame.jpg")
    else:
        env["PUPPYPI_V4L2CTL"] = "/nonexistent-v4l2-ctl"
    return subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True, env=env, timeout=10)


def test_end_to_end_launch_args(tmp_path):
    _layout(tmp_path, ["usb-Shenzhen_NV50-HD220S_1-video-index0"])
    real = os.path.realpath(str(tmp_path / "video0"))
    r = _run(tmp_path)
    assert r.returncode == 0, r.stderr
    # usb_cam 0.8.1 은 by-id 링크를 못 연다 → 실제 장치 경로, 기본 상한 1280x720, 4:2:2 → YUVJ422P
    assert r.stdout.strip() == ("use_camera:=true video_device:=%s image_width:=1280 image_height:=720 framerate:=30.0 "
                                "pixel_format:=mjpeg2rgb av_device_format:=YUVJ422P jpeg_quality:=80" % real), r.stdout
    calls = (tmp_path / "v4l2.log").read_text()
    assert "--set-fmt-video=width=1280,height=720,pixelformat=MJPG" in calls
    # 시작할 때 usb_cam 이 남긴 밝기를 카메라 기본값으로, 자동 노출·WB 켬
    assert "-c brightness=128" in calls and "-c auto_exposure=3" in calls and "-c white_balance_automatic=1" in calls
    assert "조정값(현재/기본/범위): brightness=50/128/0~255" in r.stderr
    r = _run(tmp_path, "MAX_WIDTH=1920\nMAX_HEIGHT=1080\n", y_hv=0x22)
    assert "image_width:=1920 image_height:=1080" in r.stdout and "av_device_format:=YUVJ420P " in r.stdout


def test_av_format_override_and_capture_failure(tmp_path):
    _layout(tmp_path, ["usb-Shenzhen_NV50-HD220S_1-video-index0"])
    (tmp_path / "v4l2.log").write_text("")
    r = _run(tmp_path, "AV_DEVICE_FORMAT=YUV420P\nJPEG_QUALITY=150\n")
    assert r.stdout.strip().endswith("av_device_format:=YUV420P jpeg_quality:=100")  # 품질은 10~100 으로 제한
    assert "--stream-to" not in (tmp_path / "v4l2.log").read_text()  # 직접 지정하면 프레임을 받지 않음
    r = _run(tmp_path, y_hv=None)  # 프레임을 못 받으면 기본 4:2:2
    assert r.returncode == 0 and "av_device_format:=YUVJ422P " in r.stdout


def test_camera_device_symlink_resolved(tmp_path):
    _layout(tmp_path, ["usb-Other_Cam-video-index0"])
    link = str(tmp_path / "by-id" / "usb-Other_Cam-video-index0")
    r = _run(tmp_path, "CAMERA_DEVICE=%s\n" % link)
    assert ("video_device:=%s " % os.path.realpath(str(tmp_path / "video0"))) in r.stdout


def test_end_to_end_no_camera_and_query_failure(tmp_path):
    (tmp_path / "by-id").mkdir()
    assert _run(tmp_path).stdout.strip() == "use_camera:=false"
    (tmp_path / "video0").write_text("")
    r = _run(tmp_path, v4l2=False)  # 모드 조회 실패 → 안전한 기본값
    assert "image_width:=1280 image_height:=720 framerate:=30.0 pixel_format:=mjpeg2rgb av_device_format:=YUVJ422P jpeg_quality:=80" in r.stdout
