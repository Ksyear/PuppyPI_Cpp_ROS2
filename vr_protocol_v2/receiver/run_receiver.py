#!/usr/bin/env python3
"""ROS 없이 Protocol v2 수신기를 실행한다(시험·현장 진단용).

실제 구현은 ros2_ws/src/puppy_vr_bridge/puppy_vr_bridge/ 에 있으며 이 스크립트는 그것을 그대로
불러 쓴다(코드 중복 없음). 출력 속도는 표준 출력에 JSON 줄로 기록하고 모터는 움직이지 않는다.

  python3 run_receiver.py --port 5001   (--discovery 는 검색 응답기 모듈이 있을 때만)
"""
import argparse
import json
import logging
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "ros2_ws", "src", "puppy_vr_bridge"))

from puppy_vr_bridge.protocol_v2 import V2Session  # noqa: E402
from puppy_vr_bridge.v2_server import V2UdpServer  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--max-forward", type=float, default=5.0)
    ap.add_argument("--max-reverse", type=float, default=3.0)
    ap.add_argument("--max-yaw", type=float, default=0.20)
    ap.add_argument("--command-timeout", type=float, default=0.25)
    ap.add_argument("--lease-timeout", type=float, default=1.0)
    ap.add_argument("--discovery", action="store_true")
    ap.add_argument("--duration", type=float, default=0.0, help="0 이면 Ctrl+C 까지")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    def on_output(v, reason):
        print(json.dumps({"t": round(time.monotonic(), 4), "x_cm_s": v[0], "yaw_rad_s": v[2], "why": reason},
                         ensure_ascii=False), flush=True)

    session = V2Session(args.max_forward, args.max_reverse, args.max_yaw, args.command_timeout, args.lease_timeout)
    server = V2UdpServer(session, on_output, args.bind, args.port)
    server.start()
    disc = None
    if args.discovery:
        from puppy_vr_bridge.discovery import DiscoveryServer  # 공개본에는 없다
        disc = DiscoveryServer(control_port=server.bound_port)
        disc.start()
    try:
        end = time.monotonic() + args.duration if args.duration > 0 else None
        while end is None or time.monotonic() < end:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print(json.dumps({"summary": server.snapshot()}, ensure_ascii=False, default=str), flush=True)
        if disc:
            disc.shutdown()
        server.shutdown()


if __name__ == "__main__":
    main()
