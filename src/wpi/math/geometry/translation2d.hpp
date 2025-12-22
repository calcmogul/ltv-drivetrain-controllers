// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "wpi/math/geometry/rotation2d.hpp"

namespace wpi::math {

/// Represents a translation in 2D space (North-West-Up convention).
struct Translation2d {
  double x = 0.0;
  double y = 0.0;

  constexpr Translation2d rotate_by(const Rotation2d& other) const {
    // [x_new]   [other.cos, -other.sin][x]
    // [y_new] = [other.sin,  other.cos][y]
    return {x * other.cos() - y * other.sin(),
            x * other.sin() + y * other.cos()};
  }

  constexpr Translation2d operator+(const Translation2d& other) const {
    return {x + other.x, y + other.y};
  }

  constexpr Translation2d operator-(const Translation2d& other) const {
    return {x - other.x, y - other.y};
  }
};

}  // namespace wpi::math
