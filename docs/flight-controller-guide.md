# Flight Controller Concepts: An In-Depth Guide

This guide introduces the core concepts behind flight controllers and their supporting systems, from sensor fusion and control loops to actuator protocols like DShot and OneShot. It aims to give you a strong mental model of how the entire stack works and why each piece matters.

---

## 1. Big Picture: What a Flight Controller Does

A flight controller is a real-time system that:
- Observes the vehicle's state using sensors.
- Estimates orientation, velocity, and position.
- Computes control outputs based on pilot commands or autonomous goals.
- Drives actuators (ESCs/motors/servos) at high speed to maintain stability and track commands.

Think of it as a closed-loop system where **sensing → estimation → control → actuation** runs continuously.

---

## 2. Reference Frames and Attitude

Understanding reference frames is essential.

- **Body frame**: The coordinate system fixed to the vehicle (X forward, Y right, Z down is common).
- **World/inertial frame**: A fixed reference, often aligned to gravity (North-East-Down or East-North-Up).
- **Attitude**: Orientation of the body frame relative to the world frame.

Attitude can be represented by:
- **Euler angles** (roll, pitch, yaw) — intuitive but suffer from gimbal lock.
- **Quaternion** — compact, avoids singularities, common in control and estimation.
- **Rotation matrix** — full 3x3 representation, heavier but useful for transformations.

---

## 3. Sensors and What They Measure

### 3.1 IMU (Inertial Measurement Unit)
An IMU usually contains:
- **Gyroscope**: Measures angular velocity (rad/s). Great for short-term orientation change but drifts.
- **Accelerometer**: Measures linear acceleration (m/s²). Includes gravity, which provides absolute "down".

### 3.2 Magnetometer
Measures Earth's magnetic field (microtesla). Provides heading (yaw) reference to correct gyro drift.

### 3.3 Barometer
Measures air pressure to estimate altitude. Good for relative height but noisy and slow.

### 3.4 GPS
Provides global position, velocity, and time. Slow (5–20 Hz) but gives long-term drift correction.

### 3.5 Other Sensors
- Optical flow, lidar, sonar, or vision for position/velocity near the ground.
- Airspeed sensor for fixed-wing aircraft.

---

## 4. Sensor Fusion and AHRS

### 4.1 What AHRS Means
**AHRS (Attitude and Heading Reference System)** fuses IMU + magnetometer to produce a stable orientation estimate.

### 4.2 Why Fusion Is Needed
- Gyros are smooth but drift.
- Accelerometers and magnetometers provide absolute reference but are noisy.
Fusion combines their strengths.

### 4.3 Common Filters
- **Madgwick filter**: Fast, gradient-descent correction. Great for embedded use.
- **Mahony filter**: Similar, uses PI control on error.
- **Extended Kalman Filter (EKF)**: More complex, can fuse many sensors including GPS.

Most systems run a **fast attitude filter** (hundreds of Hz) and a **slower position filter** (tens of Hz).

---

## 5. Control Loops: The Core of Stability

Flight controllers usually run multiple nested loops:

1. **Rate control** (inner loop):
   - Controls angular velocity (roll/pitch/yaw rates).
   - High bandwidth (hundreds to thousands of Hz).
   - Uses gyro data directly.

2. **Attitude control** (middle loop):
   - Controls desired orientation (roll/pitch/yaw angles).
   - Produces rate setpoints for the rate controller.

3. **Position control** (outer loop):
   - Controls position or velocity in world frame.
   - Produces attitude setpoints for the attitude controller.

This layered structure gives stability and responsiveness.

---

## 6. Control Modes Explained

### 6.1 Rate Mode (Acro)
- Pilot commands desired angular rates.
- No self-leveling. Great for aerobatics and manual control.

### 6.2 Attitude Mode (Stabilize)
- Pilot commands desired angles.
- Controller levels the craft when sticks return to center.

### 6.3 Altitude Hold
- Barometer (and sometimes IMU) used to hold height.
- Throttle stick becomes climb/descent rate command.

### 6.4 Position Hold / Loiter
- Uses GPS or optical flow to maintain location.
- Outer loop produces attitude and thrust commands.

### 6.5 Mission/Auto
- Fully autonomous waypoints and behaviors.
- Requires careful sensor fusion and safety systems.

---

## 7. PID Control in Flight Controllers

Most flight controllers still rely on PID:

- **P (Proportional)**: Reacts to current error.
- **I (Integral)**: Removes steady-state error (wind, imbalance).
- **D (Derivative)**: Predicts error trend and damps oscillations.

Tuning often starts with the **rate loop**, then attitude, then position.

---

## 8. Mixer and Motor Outputs

Once the controller produces desired forces and torques, a **mixer** converts them into motor commands.

For example, a quadcopter mixer assigns roll/pitch/yaw corrections across four motors in different directions. The mixer ensures:
- Total thrust meets altitude needs.
- Torque commands are satisfied.

---

## 9. ESCs and Motor Control

### 9.1 ESC Basics
An **Electronic Speed Controller (ESC)** drives a BLDC motor by commutating phases at high frequency. The flight controller sends it a command signal representing desired motor power.

### 9.2 Common Signal Types

#### PWM (Traditional)
- Pulse width encodes throttle (typically 1000–2000 µs).
- Update rates around 50–500 Hz.
- Simple but relatively slow.

#### OneShot125 / OneShot42
- Shorter pulses (125 µs or 42 µs).
- Faster updates (up to ~2 kHz).
- Analog signal, no checksum.

#### MultiShot
- Even shorter pulses (~5–25 µs).
- Higher update rates, lower latency.

#### DShot (Digital)
- Digital signal with checksum.
- Multiple speeds: DShot150, 300, 600, 1200.
- Benefits:
  - No calibration needed.
  - Robust against signal distortion.
  - Can send telemetry in some variants.

### 9.3 DShot in Practice
DShot sends a 16-bit packet:
- 11 bits throttle (0–2047)
- 1 bit telemetry request
- 4-bit checksum

The ESC decodes it and applies a fixed, linear throttle response. Because it is digital, it avoids issues of timing jitter and calibration.

---

## 10. Timing and Update Rates

Typical update rates:
- **Gyro sampling**: 1–8 kHz
- **Rate loop**: 500–2000 Hz
- **Attitude loop**: 100–500 Hz
- **Position loop**: 10–100 Hz
- **ESC updates**: 50 Hz to several kHz depending on protocol

Matching these rates and avoiding aliasing is critical for stability.

---

## 11. Sensor Calibration and Alignment

### 11.1 Calibration
Sensors need bias and scale calibration:
- Gyro bias (zero-rate offset).
- Accelerometer scale and axis alignment.
- Magnetometer hard/soft iron calibration.

### 11.2 Alignment
Sensor axes must be aligned with the vehicle body frame. Mismatches cause coupling and control errors.

---

## 12. Practical Safety Systems

Safety is not optional:
- **Arming/disarming logic**.
- **Failsafe** on RC signal loss.
- **Throttle limits** to prevent runaway thrust.
- **Battery monitoring** to avoid brownout.

Robust systems incorporate sensor health checks and fallback behaviors.

---

## 13. Putting It All Together (Data Flow)

A simplified high-level flow:

1. **Sensors**: IMU, mag, baro, GPS produce raw data.
2. **Filter**: AHRS and state estimators compute orientation and position.
3. **Controller**: PID loops compute desired torque and thrust.
4. **Mixer**: Converts torques/thrust into individual motor commands.
5. **ESC**: Drives motor power using PWM/OneShot/DShot.

This loop runs continuously in real time.

```mermaid
flowchart LR
  S[Sensors\nIMU, Mag, Baro, GPS] --> E[Estimator\nAHRS / EKF]
  E --> C[Controller\nRate / Attitude / Position]
  C --> M[Mixer / Allocation]
  M --> A[Actuator Output\nPWM / OneShot / DShot]
  A --> ESC[ESCs / Motors / Thrusters]
```

---

## 14. Glossary

- **AHRS**: Attitude and Heading Reference System.
- **ESC**: Electronic Speed Controller.
- **IMU**: Inertial Measurement Unit.
- **EKF**: Extended Kalman Filter.
- **DShot**: Digital ESC command protocol.

---

## 15. Suggested Next Topics

If you want to go deeper:
- Advanced EKF fusion (GPS + baro + vision).
- Motor modeling and thrust curves.
- System identification for better PID tuning.
- Control allocation for hexacopters and fixed-wing aircraft.
- Real-time scheduling and jitter analysis.

See the advanced docs for a deeper dive: `docs/advanced/index.md`.
For RP2350 DShot specifics, see `docs/advanced/rp2350-dshot.md`.

---

## 16. Notes on This Project

If you want, we can connect this guide directly to the implementation details in this codebase:
- Estimators (e.g., Madgwick)
- Controllers (rate/attitude/position)
- Output mixers and ESC drivers

Let me know if you'd like a follow-up page that maps these concepts to the actual code modules in this repository.
