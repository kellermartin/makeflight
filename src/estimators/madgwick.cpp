/**
 * @file madgwick.cpp
 * @brief Madgwick estimator implementation.
 *
 * This implementation follows the Madgwick AHRS/IMU algorithm:
 * - Integrates quaternion rate from gyroscope measurements.
 * - Applies a gradient-descent correction term using accelerometer and
 *   optionally magnetometer measurements.
 * - Normalizes the quaternion after each update.
 *
 * Reference:
 * S. O. H. Madgwick, "An efficient orientation filter for inertial and
 * inertial/magnetic sensor arrays", 2010.
 */

#include "flight/estimators/madgwick.h"

#include <cmath>

namespace flight::estimators {

namespace {

/** @brief Algorithm gain controlling correction strength. */
constexpr float kBeta = 0.1f;
/** @brief Maximum dt allowed for integration (seconds). */
constexpr float kMaxDtSec = 0.1f;

/** @brief Fast inverse square-root using std::sqrt for determinism. */
inline float InvSqrt(float value) {
  return 1.0f / std::sqrt(value);
}

/**
 * @brief Normalize a 3D vector in place.
 * @param v Vector to normalize.
 * @return true if normalization succeeded (non-zero length).
 */
inline bool NormalizeVector(flight::core::Vector3f& v) {
  const float norm_sq = v.x * v.x + v.y * v.y + v.z * v.z;
  if (norm_sq <= 0.0f) {
    return false;
  }
  const float inv_norm = InvSqrt(norm_sq);
  v.x *= inv_norm;
  v.y *= inv_norm;
  v.z *= inv_norm;
  return true;
}

/**
 * @brief Normalize a quaternion in place.
 * @param q Quaternion to normalize.
 */
inline void NormalizeQuaternion(flight::core::Quaternionf& q) {
  const float norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (norm_sq <= 0.0f) {
    q = {};
    q.w = 1.0f;
    return;
  }
  const float inv_norm = InvSqrt(norm_sq);
  q.w *= inv_norm;
  q.x *= inv_norm;
  q.y *= inv_norm;
  q.z *= inv_norm;
}

}  // namespace

/** @brief Initialize estimator state. */
bool MadgwickEstimator::Initialize() {
  state_ = {};
  return true;
}

/** @brief Update estimator using available inputs. */
EstimatorOutput MadgwickEstimator::Update(const EstimatorInput& input) {
  if (!input.imu) {
    return state_;
  }

  const auto& imu = *input.imu;
  state_.pose.angular_velocity_rps = imu.gyro_rps;

  if (state_.timestamp_us == 0) {
    state_.timestamp_us = imu.timestamp_us;
    return state_;
  }

  const float dt =
      static_cast<float>(imu.timestamp_us - state_.timestamp_us) * 1e-6f;
  if (dt <= 0.0f || dt > kMaxDtSec) {
    state_.timestamp_us = imu.timestamp_us;
    return state_;
  }

  auto& q = state_.pose.orientation;

  /** @brief Rate of change of quaternion from gyroscope. */
  const float gx = imu.gyro_rps.x;
  const float gy = imu.gyro_rps.y;
  const float gz = imu.gyro_rps.z;

  float qDot1 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
  float qDot2 = 0.5f * (q.w * gx + q.y * gz - q.z * gy);
  float qDot3 = 0.5f * (q.w * gy - q.x * gz + q.z * gx);
  float qDot4 = 0.5f * (q.w * gz + q.x * gy - q.y * gx);

  /** @brief Corrective step from accelerometer (and magnetometer if available). */
  flight::core::Vector3f accel = imu.accel_mps2;
  const bool accel_ok = NormalizeVector(accel);

  if (accel_ok) {
    if (input.mag) {
      flight::core::Vector3f mag = input.mag->magnetic_ut;
      const bool mag_ok = NormalizeVector(mag);

      if (mag_ok) {
        /** @brief Auxiliary variables to reduce computation. */
        const float q1 = q.w;
        const float q2 = q.x;
        const float q3 = q.y;
        const float q4 = q.z;

        const float _2q1 = 2.0f * q1;
        const float _2q2 = 2.0f * q2;
        const float _2q3 = 2.0f * q3;
        const float _2q4 = 2.0f * q4;
        const float q1q1 = q1 * q1;
        const float q1q2 = q1 * q2;
        const float q1q3 = q1 * q3;
        const float q1q4 = q1 * q4;
        const float q2q2 = q2 * q2;
        const float q2q3 = q2 * q3;
        const float q2q4 = q2 * q4;
        const float q3q3 = q3 * q3;
        const float q3q4 = q3 * q4;
        const float q4q4 = q4 * q4;

        /** @brief Reference direction of Earth's magnetic field. */
        const float hx =
            mag.x * (q1q1 + q2q2 - q3q3 - q4q4) +
            mag.y * (_2q2 * q3 - _2q1 * q4) +
            mag.z * (_2q2 * q4 + _2q1 * q3);
        const float hy =
            mag.x * (_2q2 * q3 + _2q1 * q4) +
            mag.y * (q1q1 - q2q2 + q3q3 - q4q4) +
            mag.z * (_2q3 * q4 - _2q1 * q2);
        const float _2bx = 2.0f * std::sqrt(hx * hx + hy * hy);
        const float _2bz =
            2.0f *
            (mag.x * (_2q2 * q4 - _2q1 * q3) +
             mag.y * (_2q3 * q4 + _2q1 * q2) +
             mag.z * (q1q1 - q2q2 - q3q3 + q4q4));
        const float _4bx = 2.0f * _2bx;
        const float _4bz = 2.0f * _2bz;

        /** @brief Gradient descent algorithm corrective step. */
        const float s1 =
            -_2q3 * (2.0f * (q2q4 - q1q3) - accel.x) +
            _2q2 * (2.0f * (q1q2 + q3q4) - accel.y) -
            _2bz * q3 *
                (_2bx * (0.5f - q3q3 - q4q4) +
                 _2bz * (q2q4 - q1q3) - mag.x) +
            (-_2bx * q4 + _2bz * q2) *
                (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) -
                 mag.y) +
            _2bx * q3 *
                (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) -
                 mag.z);
        const float s2 =
            _2q4 * (2.0f * (q2q4 - q1q3) - accel.x) +
            _2q1 * (2.0f * (q1q2 + q3q4) - accel.y) -
            4.0f * q2 * (2.0f * (0.5f - q2q2 - q3q3) - accel.z) +
            _2bz * q4 *
                (_2bx * (0.5f - q3q3 - q4q4) +
                 _2bz * (q2q4 - q1q3) - mag.x) +
            (_2bx * q3 + _2bz * q1) *
                (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) -
                 mag.y) +
            (_2bx * q4 - _4bz * q2) *
                (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) -
                 mag.z);
        const float s3 =
            -_2q1 * (2.0f * (q2q4 - q1q3) - accel.x) +
            _2q4 * (2.0f * (q1q2 + q3q4) - accel.y) -
            4.0f * q3 * (2.0f * (0.5f - q2q2 - q3q3) - accel.z) +
            (-_4bx * q3 - _2bz * q1) *
                (_2bx * (0.5f - q3q3 - q4q4) +
                 _2bz * (q2q4 - q1q3) - mag.x) +
            (_2bx * q2 + _2bz * q4) *
                (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) -
                 mag.y) +
            (_2bx * q1 - _4bz * q3) *
                (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) -
                 mag.z);
        const float s4 =
            _2q2 * (2.0f * (q2q4 - q1q3) - accel.x) +
            _2q3 * (2.0f * (q1q2 + q3q4) - accel.y) +
            (-_4bx * q4 + _2bz * q2) *
                (_2bx * (0.5f - q3q3 - q4q4) +
                 _2bz * (q2q4 - q1q3) - mag.x) +
            (-_2bx * q1 + _2bz * q3) *
                (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) -
                 mag.y) +
            _2bx * q2 *
                (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) -
                 mag.z);

        const float s_norm_sq = s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4;
        if (s_norm_sq > 0.0f) {
          const float norm_s = InvSqrt(s_norm_sq);
          qDot1 -= kBeta * s1 * norm_s;
          qDot2 -= kBeta * s2 * norm_s;
          qDot3 -= kBeta * s3 * norm_s;
          qDot4 -= kBeta * s4 * norm_s;
        }
      }
    } else {
      /** @brief Auxiliary variables to reduce computation. */
      const float q1 = q.w;
      const float q2 = q.x;
      const float q3 = q.y;
      const float q4 = q.z;

      const float _2q1 = 2.0f * q1;
      const float _2q2 = 2.0f * q2;
      const float _2q3 = 2.0f * q3;
      const float _2q4 = 2.0f * q4;
      const float _4q1 = 4.0f * q1;
      const float _4q2 = 4.0f * q2;
      const float _4q3 = 4.0f * q3;
      const float _8q2 = 8.0f * q2;
      const float _8q3 = 8.0f * q3;
      const float q1q1 = q1 * q1;
      const float q2q2 = q2 * q2;
      const float q3q3 = q3 * q3;
      const float q4q4 = q4 * q4;

      /** @brief Gradient descent algorithm corrective step. */
      const float s1 = _4q1 * q3q3 + _2q3 * accel.x +
                       _4q1 * q2q2 - _2q2 * accel.y;
      const float s2 = _4q2 * q4q4 - _2q4 * accel.x +
                       4.0f * q1q1 * q2 - _2q1 * accel.y - _4q2 +
                       _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * accel.z;
      const float s3 = 4.0f * q1q1 * q3 + _2q1 * accel.x +
                       _4q3 * q4q4 - _2q4 * accel.y - _4q3 +
                       _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * accel.z;
      const float s4 = 4.0f * q2q2 * q4 - _2q2 * accel.x +
                       4.0f * q3q3 * q4 - _2q3 * accel.y;

      const float s_norm_sq = s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4;
      if (s_norm_sq > 0.0f) {
        const float norm_s = InvSqrt(s_norm_sq);
        qDot1 -= kBeta * s1 * norm_s;
        qDot2 -= kBeta * s2 * norm_s;
        qDot3 -= kBeta * s3 * norm_s;
        qDot4 -= kBeta * s4 * norm_s;
      }
    }
  }

  q.w += qDot1 * dt;
  q.x += qDot2 * dt;
  q.y += qDot3 * dt;
  q.z += qDot4 * dt;
  NormalizeQuaternion(q);

  state_.timestamp_us = imu.timestamp_us;
  return state_;
}

}  // namespace flight::estimators
