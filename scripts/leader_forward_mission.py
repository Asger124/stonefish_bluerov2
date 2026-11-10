#!/usr/bin/env python3

import argparse
import time

from pymavlink import mavutil


def send_control(mav, forward):
    mav.mav.manual_control_send(
        mav.target_system,
        int(forward * 1000),
        0,
        500,
        0,
        0,
    )


def main():
    parser = argparse.ArgumentParser(description='Straight-forward leader mission')
    parser.add_argument('--connection', default='tcp:127.0.0.1:5762')
    parser.add_argument('--command', type=float, default=0.25)
    parser.add_argument('--duration', type=float, default=30.0)
    args = parser.parse_args()

    mav = mavutil.mavlink_connection(args.connection)
    print('Waiting for leader heartbeat...')
    mav.wait_heartbeat()
    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,
        1,
    )

    mode_mapping = mav.mode_mapping()
    if mode_mapping and 'MANUAL' in mode_mapping:
        mav.set_mode(mode_mapping['MANUAL'])
    mav.arducopter_arm()

    deadline = time.monotonic() + 5.0
    while not mav.motors_armed() and time.monotonic() < deadline:
        message = mav.recv_match(blocking=False)
        if message is not None and message.get_type() == 'STATUSTEXT':
            print(f'ArduSub: {message.text}')
        send_control(mav, 0.0)
        time.sleep(0.1)
    if mav.motors_armed():
        print('Leader armed')
    else:
        print('Leader did not confirm armed state; continuing like rectangle_mission.py')

    try:
        print(f'Leader moving forward for {args.duration:.1f} seconds')
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            send_control(mav, args.command)
            time.sleep(0.05)
    finally:
        send_control(mav, 0.0)
        time.sleep(0.5)
        mav.arducopter_disarm()


if __name__ == '__main__':
    main()
