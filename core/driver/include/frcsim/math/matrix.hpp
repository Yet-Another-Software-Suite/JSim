// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file matrix.hpp
 * @brief 3×3 matrix type for rotations and inertia-tensor operations.
 */

#pragma once
#include <cmath>
#include <cassert>
#include <limits>
#include <iostream>
#include <iomanip>

#include "vector.hpp"
#include "quaternion.hpp"

/** @addtogroup math @{ */

namespace frcsim {

/**
 * @brief 3x3 matrix type used for rotations and inertia-tensor operations.
 *
 * Physics principle: linear transformations in 3D Euclidean space. In rigid
 * dynamics this type is used for tensor transforms, inverse-inertia
 * calculations, and basis changes between local and world frames.
 *
 * Reference:
 * https://en.wikipedia.org/wiki/Inertia_tensor
 */
struct alignas(16) Matrix3 {
  /// @brief Storage array in row-major order; m[row][col].
  double m[3][3];

  /* Constructors */

  /// @brief Constructs the 3×3 identity matrix.
  constexpr Matrix3() noexcept : m{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} {}

  /**
   * @brief Constructs a matrix from explicit element values (row-major order).
   * @param m00 Row 0, column 0.
   * @param m01 Row 0, column 1.
   * @param m02 Row 0, column 2.
   * @param m10 Row 1, column 0.
   * @param m11 Row 1, column 1.
   * @param m12 Row 1, column 2.
   * @param m20 Row 2, column 0.
   * @param m21 Row 2, column 1.
   * @param m22 Row 2, column 2.
   */
  constexpr Matrix3(double m00, double m01, double m02, double m10, double m11,
                    double m12, double m20, double m21, double m22) noexcept
      : m{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}} {}

  /// @brief Returns the 3×3 identity matrix.
  static constexpr Matrix3 identity() noexcept { return Matrix3(); }

  /// @brief Returns the 3×3 zero matrix.
  static constexpr Matrix3 zero() noexcept {
    return Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  /* Element access */

  /**
   * @brief Returns a mutable pointer to row `i`.
   * @param i Row index in [0, 2].
   */
  double* operator[](size_t i) {
    assert(i < 3);
    return m[i];
  }

  /**
   * @brief Returns a const pointer to row `i`.
   * @param i Row index in [0, 2].
   */
  const double* operator[](size_t i) const {
    assert(i < 3);
    return m[i];
  }

  /* Matrix arithmetic */

  /// @brief Returns the component-wise sum of two matrices.
  constexpr Matrix3 operator+(const Matrix3& o) const noexcept {
    Matrix3 r = zero();

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        r.m[i][j] = m[i][j] + o.m[i][j];

    return r;
  }

  /// @brief Returns the component-wise difference.
  constexpr Matrix3 operator-(const Matrix3& o) const noexcept {
    Matrix3 r = zero();

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        r.m[i][j] = m[i][j] - o.m[i][j];

    return r;
  }

  /// @brief Returns this matrix scaled by scalar `s`.
  constexpr Matrix3 operator*(double s) const noexcept {
    Matrix3 r = zero();

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        r.m[i][j] = m[i][j] * s;

    return r;
  }

  /* Matrix × Vector */

  /**
   * @brief Transforms vector `v` by this matrix (M * v).
   * @param v Input column vector.
   * @return Transformed vector.
   */
  Vector3 operator*(const Vector3& v) const noexcept {
    return Vector3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                   m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                   m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
  }

  /* Matrix × Matrix */

  /**
   * @brief Returns the matrix product of this matrix and `o`.
   * @param o Right-hand matrix operand.
   * @return Product matrix.
   */
  Matrix3 operator*(const Matrix3& o) const noexcept {
    Matrix3 r = zero();

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++)
          r.m[i][j] += m[i][k] * o.m[k][j];

    return r;
  }

  /* Transpose */

  /// @brief Returns the transpose of this matrix.
  Matrix3 transpose() const noexcept {
    return Matrix3(m[0][0], m[1][0], m[2][0], m[0][1], m[1][1], m[2][1],
                   m[0][2], m[1][2], m[2][2]);
  }

  /* Determinant */

  /// @brief Returns the scalar determinant of this matrix.
  double determinant() const noexcept {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  }

  /* Inverse */

  /**
   * @brief Attempts to compute the matrix inverse.
   * @param out Output matrix set to the inverse when the function returns true.
   * @param eps Minimum absolute determinant below which the matrix is considered singular.
   * @return True on success; false when the determinant is below `eps`.
   */
  bool tryInverse(
      Matrix3& out,
      double eps = std::numeric_limits<double>::epsilon()) const noexcept {
    double det = determinant();

    if (std::abs(det) < eps)
      return false;

    double invDet = 1.0 / det;

    Matrix3 r;

    r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;

    r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;

    r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
    r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;

    out = r;
    return true;
  }

  /**
   * @brief Returns the inverse, falling back to identity on singular input.
   * @return Inverse matrix, or identity when the matrix is singular.
   * @note Prefer tryInverse() when caller must distinguish the singular case.
   */
  Matrix3 inverse() const noexcept {
    Matrix3 r;
    if (!tryInverse(r))
      return identity();  // safe fallback for legacy callers

    return r;
  }

  /* Rotation helper */

  /**
   * @brief Constructs a rotation matrix from a unit quaternion.
   * @param q Unit quaternion representing the rotation.
   * @return Equivalent 3×3 rotation matrix.
   */
  static Matrix3 fromQuaternion(const Quaternion& q) noexcept {
    Matrix3 r;

    q.toMatrix(r.m);

    return r;
  }

  /* Comparison */

  /// @brief Exact component-wise equality comparison.
  bool operator==(const Matrix3& o) const noexcept {
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        if (m[i][j] != o.m[i][j])
          return false;

    return true;
  }

  /// @brief Exact component-wise inequality comparison.
  bool operator!=(const Matrix3& o) const noexcept { return !(*this == o); }

  /* Debug output */

  /// @brief Streams the matrix as three bracketed rows.
  friend std::ostream& operator<<(std::ostream& os, const Matrix3& mat) {
    os << std::fixed << std::setprecision(4);

    for (int i = 0; i < 3; i++)
      os << "[" << mat.m[i][0] << " " << mat.m[i][1] << " " << mat.m[i][2]
         << "]\n";

    return os;
  }
};

/// @brief Left scalar multiplication helper (`s * M`).
inline Matrix3 operator*(double s, const Matrix3& m) noexcept {
  return m * s;
}

}  // namespace frcsim

/** @} */
