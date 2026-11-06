#!/usr/bin/env python3
"""PuppyPi Media v1 서버(포트 8090): VR·맥북·브라우저가 카메라와 LiDAR 지도를 보는 입구. 규격: README 'PuppyPi Media v1'

  GET /              보기 페이지(카메라 + 지도 + 상태) — 맥북 Safari/Chrome, Quest 브라우저, 휴대폰
  GET /api/media     규격 설명(영상·지도 주소, 형식, 주기). 주소의 호스트는 요청한 Host 그대로
  GET /api/map       최신 지도 이미지의 좌표 정보(/lidar_map/info) + age_s
  GET /api/status    중재기 상태(소유자·비상정지·엔진) + 지도 로봇 자세
  GET /api/path      경로 지정 주행 상태(2026-11-05): state(idle|rotate|drive|blocked|done|cancelled), 경유점, 진행,
                     막힘 횟수, 로봇 지도 위치. 경로는 VR 이 UDP Protocol v2 `path` 로 보낸다
영상 자체는 web_video_server(8080)가 보낸다(카메라 JPEG 를 재인코딩 없이 전달). 이 서버는 카메라 토픽을 구독하지 않는다
(구독하면 로봇이 아무도 안 볼 때도 JPEG 를 만든다).
"""

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

MEDIA = "puppypi-media/1"
CAMERA_TOPIC = "/usb_cam/image_raw"
MAP_TOPIC = "/lidar_map/image"
STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def descriptor(host, video_port=8080, media_port=8090):
    v = "http://%s:%d" % (host, video_port)
    m = "http://%s:%d" % (host, media_port)
    q = lambda topic: "topic=%s&type=ros_compressed" % topic
    return {
        "media": MEDIA,
        "robot": {"name": socket.gethostname()},
        "camera": {"stream": "%s/stream?%s" % (v, q(CAMERA_TOPIC)), "snapshot": "%s/snapshot?%s" % (v, q(CAMERA_TOPIC)),
                   "format": "multipart/x-mixed-replace, image/jpeg parts", "nominal_fps": 30,
                   "note": "카메라 모드(기본 1280x720)·JPEG 품질(camera.env JPEG_QUALITY)에 따름"},
        "map": {"stream": "%s/stream?%s" % (v, q(MAP_TOPIC)), "snapshot": "%s/snapshot?%s" % (v, q(MAP_TOPIC)),
                "info": "%s/api/map" % m, "format": "multipart/x-mixed-replace, image/jpeg parts", "nominal_fps": 1,
                "pixel_from_map": "u = (x - left_m) * px_per_m ; v = height_px - (y - bottom_m) * px_per_m"},
        "status": "%s/api/status" % m,
        "viewer": "%s/" % m,
    }


VIEWER = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>PuppyPi 보기</title>
<style>
:root{--bg:#0f1216;--panel:#1a1f26;--line:#2b323c;--text:#e8ecf1;--dim:#9aa4b2;--ok:#3ecf8e;--bad:#ff5c5c}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px/1.4 -apple-system,"Apple SD Gothic Neo",sans-serif}
header{padding:10px 16px;border-bottom:1px solid var(--line);display:flex;gap:12px;flex-wrap:wrap;align-items:center}
h1{font-size:17px;margin:0}.pill{padding:2px 10px;border-radius:999px;border:1px solid var(--line);color:var(--dim)}
.ok{color:var(--ok);border-color:var(--ok)}.bad{color:var(--bad);border-color:var(--bad)}
main{display:grid;grid-template-columns:minmax(0,2fr) minmax(0,1fr);gap:16px;padding:16px}
@media(max-width:900px){main{grid-template-columns:1fr}}
.box{background:#000;border:1px solid var(--line);border-radius:8px;overflow:hidden;min-height:200px;display:flex;align-items:center;justify-content:center}
.box img{width:100%;height:auto;display:block}h2{font-size:13px;color:var(--dim);margin:0 0 6px}
</style></head><body>
<header><h1>PuppyPi 보기</h1><span id="own" class="pill">조종: …</span><span id="es" class="pill">비상정지: …</span>
<span id="pose" class="pill">지도 위치: …</span><span id="path" class="pill">경로: …</span></header>
<main><section><h2>카메라</h2><div class="box"><img id="cam" alt="카메라 연결 중…"></div></section>
<section><h2>LiDAR 지도 (빨강 = 로봇)</h2><div class="box"><img id="map" alt="지도 대기 중…"></div></section></main>
<script>
function pill(id,t,c){const e=document.getElementById(id);e.textContent=t;e.className='pill '+(c||'')}
function live(img,url,delay){img.src=url;img.onerror=()=>setTimeout(()=>{img.src=url+'&t='+Date.now()},delay)}
(async()=>{const d=await (await fetch('/api/media')).json();
live(document.getElementById('cam'),d.camera.stream,2000);live(document.getElementById('map'),d.map.stream,3000);
async function poll(){try{const s=await (await fetch('/api/status')).json();const a=s.arbiter||{};
pill('own','조종: '+(a.owner||'없음'));pill('es','비상정지: '+(a.estop?'ON':'off'),a.estop?'bad':'ok');
const r=s.map&&s.map.robot;pill('pose',r?('지도 위치: '+r.x_m.toFixed(2)+', '+r.y_m.toFixed(2)+' m, '+(r.yaw_rad*57.2958).toFixed(0)+'°'):'지도 위치: 없음')}catch(e){}
try{const q=await (await fetch('/api/path')).json();const t=q.path||{};pill('path','경로: '+(t.state||'-')+(t.count?(' '+t.index+'/'+t.count):'')+(t.state==='blocked'?' (장애물)':''),t.state==='blocked'?'bad':'')}catch(e){}
setTimeout(poll,1000)}poll()})();
</script></body></html>"""


class MediaServer(Node):
    def __init__(self):
        super().__init__("media_server")
        p = self.declare_parameter
        self.port = int(p("port", 8090).value)
        self.video_port = int(p("video_port", 8080).value)
        self.lock = threading.Lock()
        self.map_info, self.map_rx = None, 0.0
        self.arbiter, self.arbiter_rx = None, 0.0
        self.recv, self.recv_rx = None, 0.0
        self.create_subscription(String, "/lidar_map/info", self._on_map, 1)
        self.create_subscription(String, "/vr_protocol_v2_receiver/state", self._on_recv, STATE_QOS)
        self.create_subscription(String, "/puppy_control/arbiter_state", self._on_arb, STATE_QOS)
        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.port), self._handler())
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, name="media-http", daemon=True).start()
        self.get_logger().info("PuppyPi Media v1: http://<로봇IP>:%d/ (영상 %d)" % (self.port, self.video_port))

    def _on_map(self, msg):
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.map_info, self.map_rx = info, time.monotonic()

    def _on_arb(self, msg):
        try:
            a = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.arbiter, self.arbiter_rx = a, time.monotonic()

    def _on_recv(self, msg):
        try:
            r = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.recv, self.recv_rx = r, time.monotonic()

    def path_status(self):
        now = time.monotonic()
        with self.lock:
            r, rx = self.recv, self.recv_rx
        if not r:
            return None
        return {"media": MEDIA, "path": r.get("path"), "pose": r.get("pose"), "session_state": r.get("state"),
                "emergency_stop": r.get("emergency_stop"), "age_s": round(now - rx, 2)}

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            m = dict(self.map_info, age_s=round(now - self.map_rx, 2)) if self.map_info else None
            a = dict(self.arbiter, age_s=round(now - self.arbiter_rx, 2)) if self.arbiter else None
        return m, a

    def _handler(self):
        node = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body, ctype="application/json; charset=utf-8"):
                data = body if isinstance(body, bytes) else (
                    json.dumps(body, ensure_ascii=False).encode("utf-8") if not isinstance(body, str) else body.encode("utf-8"))
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                host = (self.headers.get("Host") or "127.0.0.1").rsplit(":", 1)[0].strip("[]")
                path = self.path.split("?", 1)[0]
                if path == "/":
                    return self._send(200, VIEWER, "text/html; charset=utf-8")
                if path == "/api/media":
                    return self._send(200, descriptor(host, node.video_port, node.port))
                m, a = node.snapshot()
                if path == "/api/map":
                    return self._send(200, m) if m else self._send(503, {"media": MEDIA, "error": "no map yet"})
                if path == "/api/path":
                    ps = node.path_status()
                    return self._send(200, ps) if ps else self._send(503, {"media": MEDIA, "error": "no receiver state yet"})
                if path == "/api/status":
                    return self._send(200, {"media": MEDIA, "arbiter": a,
                                            "map": {"robot": m["robot"], "age_s": m["age_s"]} if m else None})
                return self._send(404, {"error": "not found", "see": "/api/media"})
        return H

    def destroy_node(self):
        self.httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MediaServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
