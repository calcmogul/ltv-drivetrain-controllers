// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdexcept>

#include "wpi/math/system/linear_system.hpp"

namespace wpi::math {

/// Creates a differential drive state-space model from SysId constants kᵥ and
/// kₐ in both linear {(V/(m/s), (V/(m/s²))} and angular {(V/(rad/s),
/// (V/(rad/s²))} cases.
///
/// The states are [left velocity, right velocity], the inputs are [left
/// voltage, right voltage], and the outputs are [left velocity, right
/// velocity].
///
/// @param linear_kv The linear velocity gain in V/(m/s).
/// @param linear_ka The linear acceleration gain in V/(m/s²).
/// @param angular_kv The angular velocity gain in V/(m/s).
/// @param angular_ka The angular acceleration gain in V/(m/s²).
/// @throws domain_error if linear_kv <= 0, linear_ka <= 0, angular_kv <= 0,
///     or angular_ka <= 0.
/// @see <a
/// href="https://github.com/wpilibsuite/sysid">https://github.com/wpilibsuite/sysid</a>
inline LinearSystem<2, 2, 2> differential_drive_from_sysid(double linear_kv,
                                                           double linear_ka,
                                                           double angular_kv,
                                                           double angular_ka) {
  if (linear_kv <= decltype(linear_kv){0}) {
    throw std::domain_error("Kv,linear must be greater than zero.");
  }
  if (linear_ka <= decltype(linear_ka){0}) {
    throw std::domain_error("Ka,linear must be greater than zero.");
  }
  if (angular_kv <= decltype(angular_kv){0}) {
    throw std::domain_error("Kv,angular must be greater than zero.");
  }
  if (angular_ka <= decltype(angular_ka){0}) {
    throw std::domain_error("Ka,angular must be greater than zero.");
  }

  double A1 = -(linear_kv / linear_ka + angular_kv / angular_ka);
  double A2 = -(linear_kv / linear_ka - angular_kv / angular_ka);
  double B1 = 1.0 / linear_ka + 1.0 / angular_ka;
  double B2 = 1.0 / linear_ka - 1.0 / angular_ka;

  A1 /= 2.0;
  A2 /= 2.0;
  B1 /= 2.0;
  B2 /= 2.0;

  Eigen::Matrix2d A{{A1, A2}, {A2, A1}};
  Eigen::Matrix2d B{{B1, B2}, {B2, B1}};
  Eigen::Matrix2d C{{1.0, 0.0}, {0.0, 1.0}};
  Eigen::Matrix2d D{{0.0, 0.0}, {0.0, 0.0}};

  return LinearSystem<2, 2, 2>{A, B, C, D};
}

}  // namespace wpi::math
