#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_udp_sender (ROS2, UDP 5006)
==================================
출처: Ksyear/Puppy-PI-Cpp src/puppy_vr_control_py/camera_udp_sender.py (팀 코드) 이식.
변경: 기본 영상 토픽을 원본 PuppyPi 와 같은 /usb_cam/image_raw/compressed 로,
SO_REUSEADDR 제거(중복 실행이 bind 오류로 드러나게), 종료 예외 처리.
주의: 2026-10-22 기준 VR 저장소(c63e7c3)에는 5006 수신 코드가 없다(미검증 경로).

로봇 카메라 영상(/image_raw/compressed, JPEG)을 UDP로 VR 헤드셋(Quest/Unity)에 전송.
C++판(src/puppy_vr_control/src/camera_udp_sender.cpp)과 패킷 구조·동작이 동일하다.

패킷 구조 (12바이트 헤더 + JPEG 조각, 정수는 네트워크 바이트순서=빅엔디언):
  [0..3]  uint32 frame_id     프레임 번호
  [4..5]  uint16 chunk_index  조각 순번 (0부터)
  [6..7]  uint16 chunk_count  전체 조각 수
  [8..11] uint32 frame_size   JPEG 전체 크기

수신 대상:
  - client_ip 지정 → 고정 전송
  - 비우면 자동 발견: Quest가 bind_port로 아무 패킷("hello")을 보내면
    그 발신 주소로 스트리밍 (client_timeout_sec 동안 없으면 중단)
"""

import socket
import struct
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage


class CameraUdpSender(Node):

    def __init__(self):
        super().__init__('camera_udp_sender')

        self.image_topic = self.declare_parameter('image_topic', '/usb_cam/image_raw/compressed').value
        self.client_ip = self.declare_parameter('client_ip', '').value
        self.client_port = self.declare_parameter('client_port', 5006).value
        self.bind_port = self.declare_parameter('bind_port', 5006).value
        self.max_fps = self.declare_parameter('max_fps', 15.0).value
        self.chunk_size = self.declare_parameter('chunk_size', 1400).value  # MTU(1500) 이하
        self.client_timeout_sec = self.declare_parameter('client_timeout_sec', 5.0).value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.bind_port))
        self.sock.settimeout(0.2)

        self.lock = threading.Lock()
        self.client_addr = None
        self.last_hello = 0.0
        self.timeout_logged = False

        self.frame_counter = 0
        self.last_send = 0.0
        self.running = True

        if self.client_ip:
            self.client_addr = (self.client_ip, self.client_port)
            self.get_logger().info(f'고정 목적지로 전송: {self.client_ip}:{self.client_port}')
        else:
            self.get_logger().info(
                f'자동 발견 모드: Quest가 UDP {self.bind_port} 포트로 패킷을 보내면 스트리밍 시작')
            self.hello_thread = threading.Thread(target=self.hello_loop, daemon=True)
            self.hello_thread.start()

        # 카메라 토픽은 best-effort(SensorDataQoS) — usb_cam 발행 QoS와 호환
        self.image_sub = self.create_subscription(
            CompressedImage, self.image_topic, self.image_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'구독: {self.image_topic} (최대 {self.max_fps} fps, 청크 {self.chunk_size} 바이트)')

    def hello_loop(self):
        """Quest가 보내는 hello 패킷의 발신 주소를 기억 (자동 발견 모드)."""
        while self.running:
            try:
                _, addr = self.sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            with self.lock:
                if addr != self.client_addr:
                    self.get_logger().info(f'클라이언트 발견: {addr[0]}:{addr[1]} → 스트리밍 시작')
                self.client_addr = addr
                self.last_hello = time.monotonic()

    def image_callback(self, msg):
        now = time.monotonic()
        # fps 제한 (Wi-Fi 대역폭 보호)
        if self.max_fps > 0 and now - self.last_send < 1.0 / self.max_fps:
            return

        with self.lock:
            dest = self.client_addr
            # 자동 발견 모드에서는 hello가 끊기면 전송 중단
            if not self.client_ip and dest is not None:
                if now - self.last_hello > self.client_timeout_sec:
                    if not self.timeout_logged:
                        self.get_logger().warn(
                            f'클라이언트 hello 끊김({self.client_timeout_sec}s) → 전송 중단')
                        self.timeout_logged = True
                    return
            self.timeout_logged = False
        if dest is None:
            return

        jpeg = bytes(msg.data)
        total = len(jpeg)
        if total == 0:
            return
        count = (total + self.chunk_size - 1) // self.chunk_size
        if count > 65535:
            return

        frame_id = self.frame_counter & 0xFFFFFFFF
        self.frame_counter += 1

        for i in range(count):
            offset = i * self.chunk_size
            payload = jpeg[offset:offset + self.chunk_size]
            # !IHHI = 빅엔디언 uint32, uint16, uint16, uint32 (C++판과 동일)
            header = struct.pack('!IHHI', frame_id, i, count, total)
            try:
                self.sock.sendto(header + payload, dest)
            except OSError:
                return
        self.last_send = now

    def shutdown(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CameraUdpSender()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
