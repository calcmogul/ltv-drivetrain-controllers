// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "wpi/math/geometry/rotation2d.hpp"
#include "wpi/math/geometry/translation2d.hpp"

namespace wpi::math {

/// Represents a 2D pose containing translational and rotational elements.
struct Pose2d {
  Translation2d translation;
  Rotation2d rotation;

  /// Returns this pose relative to another one.
  ///
  /// @param other The other pose.
  constexpr Pose2d relative_to(const Pose2d& other) const {
    return {(translation - other.translation).rotate_by(-other.rotation),
            rotation - other.rotation};
  }
};

}  // namespace wpi::math
