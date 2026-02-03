# Telemetry and Runtime Introspection

This guide describes the lightweight UDP telemetry path used in this framework for tuning and diagnostics.

## Why UDP Telemetry

- Minimal overhead and dependencies.
- Easy to extend with new fields.
- Compatible with simple Python or CLI tooling.

MAVLink remains a good future step, but UDP telemetry gets you tuning data faster.

## Data Flow

```mermaid
flowchart LR
  FC[Flight Controller] --> U[UDP Telemetry Sender]
  U --> PC[Host Tools / Plots]
```

## Packet Contents

The default UDP telemetry packet includes:

- Timestamp
- Orientation quaternion
- Angular velocity
- Position and velocity
- Setpoint velocity, body rates, and thrust
- Motor outputs
- Optional ESC telemetry (raw, decoded, CRC)

## Where It Lives In Code

- Telemetry interface: `include/flight/telemetry/telemetry.h`
- UDP sender: `include/flight/telemetry/udp_telemetry.h`
- UDP implementation: `src/telemetry/udp_telemetry.cpp`
- Vehicle publisher: `src/vehicle/rov4_vehicle.cpp`

## Tuning Workflow

1. Run the flight controller and enable telemetry.
2. Listen on the configured UDP port (default 14560).
3. Plot attitude, rate, and motor outputs.
4. Adjust PID gains and observe response.

This loop provides fast feedback without the overhead of full MAVLink integration.

The live plot includes actual rates, setpoint rates, and tracking error to help tune rate loops.

## Python Receiver

Use the helper script to inspect packets on your workstation:

```bash
python3 scripts/telemetry_receiver.py --port 14560 --plot
```
