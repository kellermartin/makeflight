# Advanced Flight Controller Topics

This section goes deeper into the topics listed in the introductory guide and ties each concept back to concrete implementation patterns in this repository.

## What You Will Find Here

- Sensor fusion beyond AHRS, including multi-sensor EKF strategies.
- Motor and propeller modeling, thrust curves, and saturation effects.
- System identification workflows that produce better control gains.
- Control allocation for multi-actuator vehicles, including constrained mixing.
- Real-time scheduling, timing budgets, and jitter tolerance.
- A final end-to-end mapping that couples theory directly to code in this framework.

## Diagrams

All diagrams in this section are ASCII so they render everywhere.

## Recommended Order

1. `ekf-fusion.md`
2. `motor-modeling.md`
3. `system-identification.md`
4. `control-allocation.md`
5. `real-time-scheduling.md`
6. `framework-coupling.md`
7. `rp2350-dshot.md`
8. `telemetry-debugging.md`
