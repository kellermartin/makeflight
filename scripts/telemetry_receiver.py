#!/usr/bin/env python3
"""UDP telemetry receiver for makeflight.

Listens for UDP packets on the configured port and prints decoded fields.
Optional live plotting uses matplotlib.
"""

import argparse
import socket
import struct
import time
from collections import deque

MAGIC = 0x4D46544C  # "MFTL"

STRUCT_FMT = (
    "<"  # little-endian
    "I"  # magic
    "B"  # version
    "B"  # motor_count
    "H"  # reserved
    "Q"  # timestamp_us
    "4f"  # orientation quaternion
    "3f"  # angular velocity
    "3f"  # position
    "3f"  # velocity
    "3f"  # setpoint velocity
    "3f"  # setpoint body rates
    "f"   # setpoint thrust
    "8f"  # motor outputs
    "B"   # esc_channel
    "H"   # esc_data
    "H"   # esc_raw
    "B"   # esc_crc_ok
    "B"   # esc_present
    "B"   # armed
)

STRUCT_SIZE = struct.calcsize(STRUCT_FMT)


def main() -> None:
    parser = argparse.ArgumentParser(description="makeflight UDP telemetry receiver")
    parser.add_argument("--port", type=int, default=14560)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--plot", action="store_true", help="Enable live plotting")
    parser.add_argument("--window", type=float, default=5.0, help="Seconds of data to display")
    parser.add_argument("--rate", type=float, default=20.0, help="Plot refresh rate (Hz)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))

    print(f"Listening on UDP {args.bind}:{args.port} (packet size {STRUCT_SIZE} bytes)")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except Exception as exc:  # pragma: no cover
            print(f"matplotlib not available: {exc}")
            return

        maxlen = max(10, int(args.window * 200))
        times = deque(maxlen=maxlen)
        rates_x = deque(maxlen=maxlen)
        rates_y = deque(maxlen=maxlen)
        rates_z = deque(maxlen=maxlen)
        set_rates_x = deque(maxlen=maxlen)
        set_rates_y = deque(maxlen=maxlen)
        set_rates_z = deque(maxlen=maxlen)
        err_x = deque(maxlen=maxlen)
        err_y = deque(maxlen=maxlen)
        err_z = deque(maxlen=maxlen)
        motors0 = deque(maxlen=maxlen)
        motors1 = deque(maxlen=maxlen)
        motors2 = deque(maxlen=maxlen)
        motors3 = deque(maxlen=maxlen)

        plt.ion()
        fig, (ax_rates, ax_error, ax_motors) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        (line_rx,) = ax_rates.plot([], [], label="p")
        (line_ry,) = ax_rates.plot([], [], label="q")
        (line_rz,) = ax_rates.plot([], [], label="r")
        (line_sx,) = ax_rates.plot([], [], label="p_sp", linestyle="--")
        (line_sy,) = ax_rates.plot([], [], label="q_sp", linestyle="--")
        (line_sz,) = ax_rates.plot([], [], label="r_sp", linestyle="--")
        ax_rates.set_ylabel("rad/s")
        ax_rates.legend()

        (line_ex,) = ax_error.plot([], [], label="p_err")
        (line_ey,) = ax_error.plot([], [], label="q_err")
        (line_ez,) = ax_error.plot([], [], label="r_err")
        ax_error.set_ylabel("rad/s err")
        ax_error.legend()

        (line_m0,) = ax_motors.plot([], [], label="m0")
        (line_m1,) = ax_motors.plot([], [], label="m1")
        (line_m2,) = ax_motors.plot([], [], label="m2")
        (line_m3,) = ax_motors.plot([], [], label="m3")
        ax_motors.set_ylabel("cmd")
        ax_motors.set_xlabel("time (s)")
        ax_motors.legend()

        next_refresh = time.time()

        while True:
            data, _ = sock.recvfrom(4096)
            if len(data) < STRUCT_SIZE:
                continue
            fields = struct.unpack_from(STRUCT_FMT, data)
            if fields[0] != MAGIC:
                continue

            timestamp_us = fields[4]
            angular_velocity = fields[9:12]
            motor_count = fields[2]
            motors = fields[25:33]
            setpoint_body_rates = fields[21:24]

            t_s = timestamp_us * 1e-6
            times.append(t_s)
            rates_x.append(angular_velocity[0])
            rates_y.append(angular_velocity[1])
            rates_z.append(angular_velocity[2])
            set_rates_x.append(setpoint_body_rates[0])
            set_rates_y.append(setpoint_body_rates[1])
            set_rates_z.append(setpoint_body_rates[2])
            err_x.append(setpoint_body_rates[0] - angular_velocity[0])
            err_y.append(setpoint_body_rates[1] - angular_velocity[1])
            err_z.append(setpoint_body_rates[2] - angular_velocity[2])

            motors0.append(motors[0] if motor_count > 0 else 0.0)
            motors1.append(motors[1] if motor_count > 1 else 0.0)
            motors2.append(motors[2] if motor_count > 2 else 0.0)
            motors3.append(motors[3] if motor_count > 3 else 0.0)

            now = time.time()
            if now >= next_refresh:
                if times:
                    t0 = times[0]
                    x = [t - t0 for t in times]
                    line_rx.set_data(x, list(rates_x))
                    line_ry.set_data(x, list(rates_y))
                    line_rz.set_data(x, list(rates_z))
                    line_sx.set_data(x, list(set_rates_x))
                    line_sy.set_data(x, list(set_rates_y))
                    line_sz.set_data(x, list(set_rates_z))
                    line_ex.set_data(x, list(err_x))
                    line_ey.set_data(x, list(err_y))
                    line_ez.set_data(x, list(err_z))
                    line_m0.set_data(x, list(motors0))
                    line_m1.set_data(x, list(motors1))
                    line_m2.set_data(x, list(motors2))
                    line_m3.set_data(x, list(motors3))

                    ax_rates.relim()
                    ax_rates.autoscale_view()
                    ax_error.relim()
                    ax_error.autoscale_view()
                    ax_motors.relim()
                    ax_motors.autoscale_view()
                    plt.pause(0.001)

                next_refresh = now + (1.0 / max(1.0, args.rate))
    else:
        while True:
            data, _ = sock.recvfrom(4096)
            if len(data) < STRUCT_SIZE:
                continue
            fields = struct.unpack_from(STRUCT_FMT, data)

            magic = fields[0]
            if magic != MAGIC:
                continue

            motor_count = fields[2]
            timestamp_us = fields[4]

            orientation = fields[5:9]
            angular_velocity = fields[9:12]
            velocity = fields[15:18]
            setpoint_velocity = fields[18:21]
            setpoint_body_rates = fields[21:24]
            setpoint_thrust = fields[24]
            motors = fields[25:33]
            esc_channel = fields[33]
            esc_data = fields[34]
            esc_raw = fields[35]
            esc_crc_ok = fields[36]
            esc_present = fields[37]
            armed = fields[38]

            print(
                "ts=%d armed=%d q=%s rates=%s vel=%s set_vel=%s set_rates=%s thrust=%.3f motors=%s esc=%s"
                % (
                    timestamp_us,
                    armed,
                    tuple(round(v, 3) for v in orientation),
                    tuple(round(v, 3) for v in angular_velocity),
                    tuple(round(v, 3) for v in velocity),
                    tuple(round(v, 3) for v in setpoint_velocity),
                    tuple(round(v, 3) for v in setpoint_body_rates),
                    setpoint_thrust,
                    tuple(round(v, 3) for v in motors[:motor_count]),
                    (
                        f"ch={esc_channel} data={esc_data} raw={esc_raw} crc={esc_crc_ok}"
                        if esc_present
                        else "none"
                    ),
                )
            )


if __name__ == "__main__":
    main()
