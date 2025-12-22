// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/controller/ltv_unicycle_controller.hpp"

#include <cmath>

#include <Eigen/Core>

#include "wpi/math/controller/lqr.hpp"
#include "wpi/math/system/discretize_ab.hpp"

namespace wpi::math {

ChassisSpeeds LTVUnicycleController::calculate(const Pose2d& current_pose,
                                               const Pose2d& pose_ref,
                                               double linear_velocity_ref,
                                               double angular_velocity_ref) {
  // This implements the linear time-varying unicycle controller in theorem
  // 8.8.1 of https://controls-in-frc.link/
  //
  //     [x]       [Vₗ]
  // x = [y]   u = [Vᵣ]
  //     [θ]

  if (!m_enabled) {
    return {linear_velocity_ref, 0.0, angular_velocity_ref};
  }

  // The DARE is ill-conditioned if the velocity is close to zero, so don't
  // let the system stop.
  if (std::abs(linear_velocity_ref) < 1e-4) {
    linear_velocity_ref = 1e-4;
  }

  m_pose_error = pose_ref.relative_to(current_pose);

  Eigen::Matrix<double, 3, 3> A{
      {0.0, 0.0, 0.0}, {0.0, 0.0, linear_velocity_ref}, {0.0, 0.0, 0.0}};
  Eigen::Matrix<double, 3, 2> B{{1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}};

  auto [disc_A, disc_B] = discretize_ab(A, B, m_dt);

  auto K = lqr(disc_A, disc_B, m_Q, m_R);

  Eigen::Vector3d e{m_pose_error.translation.x, m_pose_error.translation.y,
                    m_pose_error.rotation.radians()};
  Eigen::Vector2d u = K * e;

  return {linear_velocity_ref + u[0], 0.0, angular_velocity_ref + u[1]};
}

}  // namespace wpi::math
