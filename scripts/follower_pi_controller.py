#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException
from tf2_ros import LookupException, TransformListener

from pymavlink import mavutil


CONNECTION = 'tcp:127.0.0.1:5772'
TARGET_X = 0.0
TARGET_Y = 0.0
TARGET_Z = 1.0
KP_X = 0.8
KP_Y = 0.8
KP_Z = 0.8
KI_Z = 0.05
MAX_COMMAND = 0.35


class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_pi_controller')

        self.mav = mavutil.mavlink_connection(CONNECTION)
        self.mav.wait_heartbeat()

        mode = self.mav.mode_mapping()
        if mode and 'MANUAL' in mode:
            self.mav.set_mode(mode['MANUAL'])
        self.mav.arducopter_arm()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.integral_z = 0.0
        self.last_time = time.monotonic()
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Following leader at x=0.0, y=0.0, z=1.0 m')
    # Correct order: surge (x), sway (y), throttle (z)
    def send_control(self, x, y, z):
        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))
        z = max(-1.0, min(1.0, z))
        self.mav.mav.manual_control_send(
            self.mav.target_system,
            int(x * 1000),
            int(y * 1000),
            int(500 + z * 500),
            0,
            0,
        )

    def control_loop(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'bluerov2_2/camera_left',
                'leader_tag_frame',
                rclpy.time.Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.integral_z = 0.0
            self.send_control(0.0, 0.0, 0.0)
            return

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            self.integral_z = 0.0
            self.send_control(0.0, 0.0, 0.0)
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        z = transform.transform.translation.z

        error_x = x - TARGET_X
        error_y = y - TARGET_Y
        error_z = z - TARGET_Z
        self.integral_z += error_z * dt
        self.integral_z = max(-1.0, min(1.0, self.integral_z))

        command_x = KP_X * error_x
        command_y = KP_Y * error_y
        command_z = KP_Z * error_z + KI_Z * self.integral_z
        command_x = max(-MAX_COMMAND, min(MAX_COMMAND, command_x))
        command_y = max(-MAX_COMMAND, min(MAX_COMMAND, command_y))
        command_z = max(-MAX_COMMAND, min(MAX_COMMAND, command_z))

        self.send_control(command_x, command_y, command_z)
        self.get_logger().info(
            f'x={x:.2f} y={y:.2f} z={z:.2f}',
            throttle_duration_sec=0.5,
        )

    def stop(self):
        self.send_control(0.0, 0.0, 0.0)
        self.mav.arducopter_disarm()


def main():
    rclpy.init()
    node = FollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
