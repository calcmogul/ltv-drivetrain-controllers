// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

namespace wpi::math {

/// A rotation in a 2D coordinate frame represented by a point on the unit
/// circle (cosine and sine).
class Rotation2d {
 public:
  /// Constructs a Rotation2d with a default angle of 0 degrees.
  constexpr Rotation2d() = default;

  /// Constructs a Rotation2d with the given angle in radians.
  ///
  /// @param value The value of the angle in radians.
  constexpr Rotation2d(double value)  // NOLINT
      : m_cos{std::cos(value)}, m_sin{std::sin(value)} {}

  /// Constructs a Rotation2d with the given cosine and sine components.
  ///
  /// @param cos The cosine of the rotation.
  /// @param sin The sine of the rotation.
  constexpr Rotation2d(double cos, double sin) : m_cos{cos}, m_sin{sin} {}

  constexpr Rotation2d operator+(const Rotation2d& other) const {
    // [cos_new]   [other.cos, -other.sin][cos]
    // [sin_new] = [other.sin,  other.cos][sin]
    return {cos() * other.cos() - sin() * other.sin(),
            cos() * other.sin() + sin() * other.cos()};
  }

  constexpr Rotation2d operator-(const Rotation2d& other) const {
    return *this + (-other);
  }

  constexpr Rotation2d operator-() const { return Rotation2d{m_cos, -m_sin}; }

  constexpr double radians() const { return std::atan2(m_sin, m_cos); }

  constexpr double degrees() const {
    constexpr double pi = 3.141592653589793238462643383279502884L;
    return radians() / pi * 180.0;
  }

  constexpr double cos() const { return m_cos; }

  constexpr double sin() const { return m_sin; }

 private:
  double m_cos = 1.0;
  double m_sin = 0.0;
};

}  // namespace wpi::math
