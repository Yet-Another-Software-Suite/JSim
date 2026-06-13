// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file quaternion.hpp
 * @brief Unit-quaternion rotation representation and integration helpers.
 */

#pragma once
#include <cmath>
#include <algorithm>
#include <cassert>
#include "vector.hpp"

/** @addtogroup math @{ */

namespace frcsim {

/**
 * @brief Unit-quaternion rotation representation and rotation algebra helpers.
 *
 * Physics principle: quaternion rotational kinematics avoids gimbal lock and
 * provides stable composition/integration of 3D orientation. Vectors are
 * rotated with q v q^-1 and orientation updates use angular velocity via
 * quaternion derivatives.
 *
 * References:
 * https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
 * https://en.wikipedia.org/wiki/Gimbal_lock
 */
struct Quaternion {
  /// @brief Real (scalar) component of the quaternion.
  alignas(16) double w;
  /// @brief Imaginary x component.
  double x;
  /// @brief Imaginary y component.
  double y;
  /// @brief Imaginary z component.
  double z;

  /// @brief Constructs the identity quaternion (w=1, x=y=z=0).
  constexpr Quaternion() noexcept : w(1.0), x(0.0), y(0.0), z(0.0) {}
  /**
   * @brief Constructs from explicit components.
   * @param w_ Real part.
   * @param x_ Imaginary x.
   * @param y_ Imaginary y.
   * @param z_ Imaginary z.
   */
  constexpr Quaternion(double w_, double x_, double y_, double z_) noexcept
      : w(w_), x(x_), y(y_), z(z_) {}
  /**
   * @brief Constructs from scalar and vector parts.
   * @param w_ Real part.
   * @param v Imaginary vector part.
   */
  constexpr Quaternion(double w_, const Vector3& v) noexcept
      : w(w_), x(v.x), y(v.y), z(v.z) {}

  /// @brief Returns squared magnitude: w^2+x^2+y^2+z^2.
  [[nodiscard]]
  constexpr double norm2() const noexcept {
    return w * w + x * x + y * y + z * z;
  }
  /// @brief Returns Euclidean magnitude.
  [[nodiscard]]
  double norm() const noexcept {
    return std::sqrt(norm2());
  }

  /**
   * @brief Returns true when this quaternion is within `eps` of the identity.
   * @param eps Absolute tolerance applied to each component.
   */
  [[nodiscard]]
  bool isIdentity(double eps = 1e-12) const noexcept {
    return std::abs(w - 1.0) < eps && std::abs(x) < eps && std::abs(y) < eps &&
           std::abs(z) < eps;
  }
  /// @brief Returns true when any component is NaN.
  [[nodiscard]]
  bool hasNaN() const noexcept {
    return std::isnan(w) || std::isnan(x) || std::isnan(y) || std::isnan(z);
  }

  /**
   * @brief Returns a normalized copy.
   * @return Unit-length quaternion in the same orientation, or identity if near-zero.
   */
  [[nodiscard]]
  Quaternion normalized() const noexcept {
    double n = norm();
    if (n < 1e-12)
      return Quaternion();
    return Quaternion(w / n, x / n, y / n, z / n);
  }

  /**
   * @brief Normalizes in place only if the quaternion is not already unit-length.
   * @param eps Tolerance for determining when re-normalization is needed.
   */
  void normalizeIfNeeded(double eps = 1e-12) noexcept {
    double n2 = norm2();
    if (std::abs(n2 - 1.0) > eps)
      normalize();
  }

  /**
   * @brief Constructs a rotation quaternion from an axis and angle.
   * @param axis Rotation axis (need not be unit length; normalized internally).
   * @param angleRad Rotation angle in radians.
   * @return Unit quaternion representing the rotation.
   */
  static Quaternion fromAxisAngle(const Vector3& axis,
                                  double angleRad) noexcept {
    Vector3 nAxis = axis.normalized();
    double half = 0.5 * angleRad;
    double s = std::sin(half);
    return Quaternion(std::cos(half), nAxis.x * s, nAxis.y * s, nAxis.z * s);
  }

  /**
   * @brief Constructs the incremental rotation quaternion for an angular-velocity step.
   * @param omega Angular velocity vector in radians per second.
   * @param dt Integration timestep in seconds.
   * @return Unit quaternion representing the incremental rotation.
   */
  static Quaternion fromAngularVelocity(const Vector3& omega,
                                        double dt) noexcept {
    double angle = omega.norm() * dt;
    Vector3 axis = (angle > 1e-12) ? omega.normalized() : Vector3(1, 0, 0);
    return fromAxisAngle(axis, angle);
  }
  /**
   * @brief Decomposes this quaternion into an axis and angle.
   * @param axis Output rotation axis (unit length).
   * @param angleRad Output rotation angle in radians.
   */
  void toAxisAngle(Vector3& axis, double& angleRad) const noexcept {
    Quaternion q = normalized();
    q.w = std::clamp(q.w, -1.0, 1.0);
    angleRad = 2.0 * std::acos(q.w);
    double s = std::sqrt(std::max(0.0, 1.0 - q.w * q.w));
    if (s < 1e-12) {
      axis = Vector3(1, 0, 0);
    } else {
      axis = Vector3(q.x / s, q.y / s, q.z / s);
    }
  }
  /**
   * @brief Spherical linear interpolation between two orientations.
   * @param a Start quaternion.
   * @param b End quaternion.
   * @param t Interpolation parameter in [0, 1].
   * @return Interpolated unit quaternion.
   */
  static Quaternion slerp(const Quaternion& a, const Quaternion& b,
                          double t) noexcept {
    double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    Quaternion b2 = b;
    if (dot < 0.0) {
      dot = -dot;
      b2 = b * -1.0;
    }
    if (dot > 0.9995) {
      // Linear
      return (a * (1.0 - t) + b2 * t).normalized();
    }
    double theta = std::acos(dot);
    double s1 = std::sin((1.0 - t) * theta);
    double s2 = std::sin(t * theta);
    double s = std::sin(theta);
    return (a * s1 + b2 * s2) * (1.0 / s);
  }
  /**
   * @brief Fills a row-major 3×3 rotation matrix from this quaternion.
   * @param m Output 3×3 array in row-major order.
   */
  void toMatrix(double m[3][3]) const noexcept {
    double xx = x * x, yy = y * y, zz = z * z;
    double xy = x * y, xz = x * z, yz = y * z;
    double wx = w * x, wy = w * y, wz = w * z;
    m[0][0] = 1.0 - 2.0 * (yy + zz);
    m[0][1] = 2.0 * (xy - wz);
    m[0][2] = 2.0 * (xz + wy);
    m[1][0] = 2.0 * (xy + wz);
    m[1][1] = 1.0 - 2.0 * (xx + zz);
    m[1][2] = 2.0 * (yz - wx);
    m[2][0] = 2.0 * (xz - wy);
    m[2][1] = 2.0 * (yz + wx);
    m[2][2] = 1.0 - 2.0 * (xx + yy);
  }

  /// @brief Normalizes this quaternion in place; resets to identity on near-zero input.
  void normalize() noexcept {
    double n = norm();
    if (n < 1e-12) {
      w = 1.0;
      x = y = z = 0.0;
      return;
    }
    w /= n;
    x /= n;
    y /= n;
    z /= n;
  }

  /// @brief Returns the conjugate (w, -x, -y, -z) — inverse for unit quaternions.
  [[nodiscard]]
  constexpr Quaternion conjugate() const noexcept {
    return Quaternion(w, -x, -y, -z);
  }

  /**
   * @brief Returns the multiplicative inverse.
   * @return q^-1 = conjugate / norm^2; identity on near-zero norm.
   */
  [[nodiscard]]
  Quaternion inverse() const noexcept {
    double n2 = norm2();
    if (n2 < 1e-12)
      return Quaternion();
    Quaternion c = conjugate();
    return Quaternion(c.w / n2, c.x / n2, c.y / n2, c.z / n2);
  }

  /// @brief Returns the Hamilton product of this quaternion and `rhs`.
  [[nodiscard]]
  constexpr Quaternion operator*(const Quaternion& rhs) const noexcept {
    return Quaternion(w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
                      w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
                      w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
                      w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w);
  }

  /// @brief Returns this quaternion scaled by `scalar` (right multiply).
  [[nodiscard]]
  constexpr Quaternion operator*(double scalar) const noexcept {
    return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
  }
  /// @brief Returns quaternion `q` scaled by `scalar` (left multiply).
  friend constexpr Quaternion operator*(double scalar,
                                        const Quaternion& q) noexcept {
    return Quaternion(q.w * scalar, q.x * scalar, q.y * scalar, q.z * scalar);
  }

  /**
   * @brief Rotates a vector by this quaternion using q v q^-1.
   * @param v Vector to rotate.
   * @return Rotated vector.
   */
  [[nodiscard]]
  Vector3 rotate(const Vector3& v) const noexcept {
    Quaternion qv(0.0, v);
    Quaternion res = (*this * qv * conjugate());
    return Vector3(res.x, res.y, res.z);
  }

  /// @brief Returns the local +Z basis vector in world space.
  [[nodiscard]]
  Vector3 forward() const noexcept {
    return rotate(Vector3(0, 0, 1));
  }
  /// @brief Returns the local +Y basis vector in world space.
  [[nodiscard]]
  Vector3 up() const noexcept {
    return rotate(Vector3(0, 1, 0));
  }
  /// @brief Returns the local +X basis vector in world space.
  [[nodiscard]]
  Vector3 right() const noexcept {
    return rotate(Vector3(1, 0, 0));
  }

  /// @brief Returns the component-wise sum of two quaternions (not a rotation composition).
  [[nodiscard]]
  constexpr Quaternion operator+(const Quaternion& rhs) const noexcept {
    return Quaternion(w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z);
  }
  /// @brief Returns the negation of all components.
  [[nodiscard]]
  constexpr Quaternion operator-() const noexcept {
    return Quaternion(-w, -x, -y, -z);
  }
  /// @brief Exact component-wise equality comparison.
  [[nodiscard]]
  constexpr bool operator==(const Quaternion& rhs) const noexcept {
    return w == rhs.w && x == rhs.x && y == rhs.y && z == rhs.z;
  }
  /// @brief Exact component-wise inequality comparison.
  [[nodiscard]]
  constexpr bool operator!=(const Quaternion& rhs) const noexcept {
    return !(*this == rhs);
  }

  /// @brief Streams the quaternion in [w, (x, y, z)] notation.
  friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    return os << "[" << q.w << ", (" << q.x << ", " << q.y << ", " << q.z
              << ")]";
  }
};

}  // namespace frcsim

/** @} */
