#!/usr/bin/env python3
"""
Open-loop time-based square mission (Colleague's Diagnostic Script).
Adapted to support command-line connection arguments and safe control intervals.
"""
import argparse
import time
from pymavlink import mavutil

FORWARD_INPUT = 600
TURN_INPUT = 300

INITIAL_WAIT = 3.0
FORWARD_TIME = 5.0
TURN_TIME = 3.0       # Calibrate this time to reach ~90 degrees rotation
PAUSE_TIME = 3.0
FINAL_WAIT = 2.0

SEND_INTERVAL = 0.05  # 20 Hz control rate to keep safety timeout happy


def send_control(m, x=0, y=0, z=500, r=0):
    """Send one manual control command."""
    m.mav.manual_control_send(
        m.target_system, x, y, z, r, 0
    )


def run_phase(m, name, duration, x=0, y=0, z=500, r=0):
    """Repeat a control command for a specified duration."""
    print(f"{name}: {duration:.1f} seconds")
    start = time.monotonic()
    while time.monotonic() - start < duration:
        send_control(m, x, y, z, r)
        time.sleep(SEND_INTERVAL)
    # Briefly command neutral after every phase.
    send_control(m, 0, 0, 500, 0)
    time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description="Colleague's Open-Loop Square Mission")
    parser.add_argument("--connection", default="tcp:127.0.0.1:5762", help="MAVLink connection string")
    args = parser.parse_args()

    print(f"Connecting to autopilot at {args.connection}...")
    m = mavutil.mavlink_connection(args.connection)
    print("Waiting for heartbeat...")
    m.wait_heartbeat()
    print(f"Connected: system {m.target_system}, component {m.target_component}")

    # Request data stream explicitly
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )

    try:
        # Select MANUAL mode and arm.
        mode_mapping = m.mode_mapping()
        if "MANUAL" in mode_mapping:
            m.set_mode(mode_mapping["MANUAL"])
        time.sleep(1)

        print("Arming...")
        m.arducopter_arm()
        
        # Non-blocking arm verification loop
        start_arm = time.time()
        while time.time() - start_arm < 5:
            send_control(m, 0, 0, 500, 0)
            if m.motors_armed():
                break
            time.sleep(0.1)
        print("Armed")

        # Phase 1: initial neutral period
        run_phase(m, "Initial neutral", INITIAL_WAIT)

        # Execute 3 sides (Forward -> Pause -> Turn -> Pause)
        for i in range(1, 4):
            run_phase(m, f"Forward segment {i}", FORWARD_TIME, x=FORWARD_INPUT)
            run_phase(m, f"Pause before turn {i}", PAUSE_TIME)
            run_phase(m, f"Turn {i}", TURN_TIME, r=TURN_INPUT)
            run_phase(m, f"Pause after turn {i}", PAUSE_TIME)

        # Final 4th side and finish
        run_phase(m, "Final forward segment", FORWARD_TIME, x=FORWARD_INPUT)
        run_phase(m, "Final neutral", FINAL_WAIT)

    finally:
        # Always attempt to stop movement before disarming.
        send_control(m, 0, 0, 500, 0)
        time.sleep(0.5)
        m.arducopter_disarm()
        print("Experiment finished.")


if __name__ == "__main__":
    main()