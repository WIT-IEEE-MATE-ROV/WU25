#!/usr/bin/env python3
"""
Simple ROS2 -> HTTP telemetry bridge

Subscribes to ROS topics and forwards their data as JSON to an HTTP endpoint.

Configure via environment variables:
  TELEMETRY_ENDPOINT  HTTP endpoint to POST JSON payloads (default: http://localhost:8080/api/telemetry)

Run on Orange Pi (same environment as ROS2).
"""
import os
import time
import json
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, String

import requests


ENDPOINT = os.environ.get('TELEMETRY_ENDPOINT', 'http://localhost:8080/api/telemetry')

# Minimum seconds between HTTP posts per topic (rate limiting)
RATE_IMU = 0.05       # 20 Hz max
RATE_JOY = 0.1        # 10 Hz max
RATE_THRUSTERS = 0.05 # 20 Hz max
RATE_MODE = 0.2       # 5 Hz max


def post_json(payload: dict):
    try:
        requests.post(ENDPOINT, json=payload, timeout=2)
    except Exception as e:
        print('telemetry post failed:', e)


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('ros_to_http_bridge')

        self._last_imu = 0.0
        self._last_joy = 0.0
        self._last_thrusters = 0.0
        self._last_mode = 0.0

        self.create_subscription(QuaternionStamped, '/bno/quat', self.cb_quat, 10)
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.create_subscription(Float32MultiArray, '/thrusters/output', self.cb_thrusters, 10)
        self.create_subscription(String, '/thrusters/control_mode', self.cb_control_mode, 10)

        self.get_logger().info('TelemetryBridge ready, posting to ' + ENDPOINT)

    def cb_quat(self, msg: QuaternionStamped):
        now = time.time()
        if now - self._last_imu < RATE_IMU:
            return
        self._last_imu = now
        payload = {
            'topic': 'bno/quat',
            'ts': now,
            'quat': {
                'w': float(msg.quaternion.w),
                'x': float(msg.quaternion.x),
                'y': float(msg.quaternion.y),
                'z': float(msg.quaternion.z),
            }
        }
        threading.Thread(target=post_json, args=(payload,), daemon=True).start()

    def cb_joy(self, msg: Joy):
        now = time.time()
        if now - self._last_joy < RATE_JOY:
            return
        self._last_joy = now
        payload = {
            'topic': 'joy',
            'ts': now,
            'axes': [float(a) for a in msg.axes],
            'buttons': [int(b) for b in msg.buttons]
        }
        threading.Thread(target=post_json, args=(payload,), daemon=True).start()

    def cb_thrusters(self, msg: Float32MultiArray):
        now = time.time()
        if now - self._last_thrusters < RATE_THRUSTERS:
            return
        self._last_thrusters = now
        payload = {
            'topic': 'thrusters/output',
            'ts': now,
            'thrusters': [float(v) for v in msg.data]
        }
        threading.Thread(target=post_json, args=(payload,), daemon=True).start()

    def cb_control_mode(self, msg: String):
        now = time.time()
        if now - self._last_mode < RATE_MODE:
            return
        self._last_mode = now
        try:
            mode = json.loads(msg.data)
        except Exception:
            mode = {}
        payload = {'topic': 'thrusters/control_mode', 'ts': now, **mode}
        threading.Thread(target=post_json, args=(payload,), daemon=True).start()


def main():
    rclpy.init()
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
