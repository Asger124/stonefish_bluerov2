#!/usr/bin/env python3

import time
import math

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
KP_Y = 0.0
KP_Z = 0.8
KI_Z = 0.05
MAX_COMMAND = 0.35
KP_R = 0.5
KD_R = 0.05
BEARING_DEADBAND = math.radians(6.0)
MAX_YAW_COMMAND = 0.1
MAX_YAW_ACCELERATION = 0.35
BEARING_RATE_LIMIT = 1.5
BEARING_RATE_FILTER = 0.25
TURN_RATE_THRESHOLD = math.radians(4.0)
TURN_RATE_FULL = math.radians(35.0)
TURN_BOOST_MAX = 0.50
TURN_RAMP_UP = 0.40 
TURN_RAMP_DOWN = 0.12
MAX_FORWARD_COMMAND = 0.60
TURN_BOOST_FILTER = 0.20


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
        self.last_bearing = None
        self.filtered_bearing_rate = 0.0
        self.last_yaw_command = 0.0
        self.filtered_turn_activity = 0.0
        self.turn_boost = 0.0
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Following leader at x=0.0, y=0.0, z=1.0 m')


    # Correct order: surge (x), sway (y), throttle (z), yaw (r)
    def send_control(self, x, y, z, r=0.0):
        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))
        z = max(-1.0, min(1.0, z))
        r = max(-1.0, min(1.0, r))
        self.mav.mav.manual_control_send(
            self.mav.target_system,
            int(x * 1000),
            int(y * 1000),
            int(500 + z * 500),
            int(r * 1000),
            0,
        )
    def halt(self):
        self.send_control(0.0, 0.0, 0.0, 0.0)

    def reset_motion_state(self):
        self.integral_z = 0.0
        self.last_bearing = None
        self.filtered_bearing_rate = 0.0
        self.last_yaw_command = 0.0
        self.filtered_turn_activity = 0.0
        self.turn_boost = 0.0

    def control_loop(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'bluerov2_2/camera_left',
                'leader_tag_frame',
                rclpy.time.Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.reset_motion_state()
            self.send_control(0.0, 0.0, 0.0, 0.0)
            return

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            self.reset_motion_state()
            self.send_control(0.0, 0.0, 0.0, 0.0)
            return

        cam_x = transform.transform.translation.x
        cam_y = transform.transform.translation.y
        cam_z = transform.transform.translation.z

        # Errors
        error_x = cam_z - TARGET_Z
        error_y = cam_x - TARGET_X
        error_z = TARGET_Y - cam_y

        # --- DEADBAND FILTER TO STOP STRAIGHT-LINE WOBBLING ---
        # Ignore tiny lateral drifts under 4cm to prevent thruster twitching & hunting
        if abs(error_y) < 0.04:
            error_y = 0.0
        # -----------------------------------------------------

        self.integral_z += error_z * dt
        self.integral_z = max(-1.0, min(1.0, self.integral_z))

        command_x = KP_X * error_x
        command_y = KP_Y * error_y  # (0.0 if KP_Y is 0)
        command_z = KP_Z * error_z + KI_Z * self.integral_z

        # Horizontal bearing from the camera.  Deadband around zero prevents
        # image noise from generating a yaw command while following straight.
        bearing = math.atan2(cam_x, max(cam_z, 0.05))
        if abs(bearing) < BEARING_DEADBAND:
            bearing_error = 0.0
        else:
            bearing_error = bearing - math.copysign(BEARING_DEADBAND, bearing)

        if self.last_bearing is None:
            bearing_rate = 0.0
        else:
            raw_bearing_rate = (bearing - self.last_bearing) / dt
            raw_bearing_rate = max(-BEARING_RATE_LIMIT,
                                    min(BEARING_RATE_LIMIT, raw_bearing_rate))
            bearing_rate = (
                BEARING_RATE_FILTER * raw_bearing_rate
                + (1.0 - BEARING_RATE_FILTER) * self.filtered_bearing_rate
            )
        self.last_bearing = bearing
        self.filtered_bearing_rate = bearing_rate

        # Surge boost: detect a sustained turn, then ramp the follower forward.
        # The boost is released slowly so a sharp 90-degree turn does not end
        # in an immediate stop. It is only applied to forward motion.
        turn_activity = (
            abs(bearing_rate) - TURN_RATE_THRESHOLD
        ) / (TURN_RATE_FULL - TURN_RATE_THRESHOLD)
        turn_activity = max(0.0, min(1.0, turn_activity))
        self.filtered_turn_activity = (
            TURN_BOOST_FILTER * turn_activity
            + (1.0 - TURN_BOOST_FILTER) * self.filtered_turn_activity
        )

        target_turn_boost = TURN_BOOST_MAX * self.filtered_turn_activity
        if target_turn_boost > self.turn_boost:
            self.turn_boost = min(
                target_turn_boost,
                self.turn_boost + TURN_RAMP_UP * dt,
            )
        else:
            self.turn_boost = max(
                target_turn_boost,
                self.turn_boost - TURN_RAMP_DOWN * dt,
            )

        # P tracks the leader's bearing; D is predictive and reacts to the
        # leader moving sideways before the follower has accumulated offset.
        command_r = (
            KP_R * bearing_error
            + KD_R * bearing_rate
        )
        command_r = max(-MAX_YAW_COMMAND,
                        min(MAX_YAW_COMMAND, command_r))

        # Slew-rate limit prevents thruster command reversals and wobble.
        yaw_delta = command_r - self.last_yaw_command
        max_delta = MAX_YAW_ACCELERATION * dt
        yaw_delta = max(-max_delta, min(max_delta, yaw_delta))
        command_r = self.last_yaw_command + yaw_delta
        command_r = max(-MAX_YAW_COMMAND,
                        min(MAX_YAW_COMMAND, command_r))
        self.last_yaw_command = command_r

        if error_x > 0.0:
            command_x += self.turn_boost

        forward_limit = (
            MAX_FORWARD_COMMAND
            if self.turn_boost > 0.01
            else MAX_COMMAND
        )
        command_x = max(-MAX_COMMAND, min(forward_limit, command_x))
        command_y = max(-MAX_COMMAND, min(MAX_COMMAND, command_y))
        command_z = max(-MAX_COMMAND, min(MAX_COMMAND, command_z))

        self.send_control(command_x, command_y, command_z, command_r)
        
        self.get_logger().info(
            f'cam_x={cam_x:.2f} bearing={math.degrees(bearing):.1f}deg '
            f'bearing_rate={math.degrees(bearing_rate):.1f}deg/s '
            f'turn_boost={self.turn_boost:.2f} yaw_cmd={command_r:.2f}',
            throttle_duration_sec=0.5,
        )

    def stop(self):
        self.send_control(0.0, 0.0, 0.0, 0.0)
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