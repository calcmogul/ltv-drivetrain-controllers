// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/controller/ltv_differential_drive_controller.hpp"

#include <cmath>

#include "wpi/math/controller/lqr.hpp"
#include "wpi/math/system/discretize_ab.hpp"

using namespace wpi::math;

namespace {

/// Wraps an angle to the range -π to π radians.
///
/// @param angle Angle to wrap.
constexpr double angle_modulus(double angle) {
  constexpr double pi = 3.141592653589793238462643383279502884L;
  constexpr double min_angle = -pi;
  constexpr double max_angle = pi;
  constexpr double modulus = max_angle - min_angle;

  // Wrap input if it's above the maximum input
  int num_max = (angle - min_angle) / modulus;
  angle -= num_max * modulus;

  // Wrap input if it's below the minimum input
  int num_min = (angle - max_angle) / modulus;
  angle -= num_min * modulus;

  return angle;
}

}  // namespace

DifferentialDriveWheelVoltages LTVDifferentialDriveController::calculate(
    const Pose2d& current_pose, double left_velocity, double right_velocity,
    const Pose2d& pose_ref, double left_velocity_ref,
    double right_velocity_ref) {
  // This implements the linear time-varying differential drive controller in
  // theorem 8.7.4 of https://controls-in-frc.link/
  //
  //     [x ]
  //     [y ]       [Vₗ]
  // x = [θ ]   u = [Vᵣ]
  //     [vₗ]
  //     [vᵣ]

  double velocity = (left_velocity + right_velocity) / 2.0;

  // The DARE is ill-conditioned if the velocity is close to zero, so don't
  // let the system stop.
  if (std::abs(velocity) < 1e-4) {
    velocity = 1e-4;
  }

  Eigen::Vector<double, 5> r{pose_ref.translation.x, pose_ref.translation.y,
                             pose_ref.rotation.radians(), left_velocity_ref,
                             right_velocity_ref};
  Eigen::Vector<double, 5> x{
      current_pose.translation.x, current_pose.translation.y,
      current_pose.rotation.radians(), left_velocity, right_velocity};

  m_error = r - x;
  m_error[2] = angle_modulus(m_error[2]);

  Eigen::Matrix<double, 5, 5> A{
      {0.0, 0.0, 0.0, 0.5, 0.5},
      {0.0, 0.0, velocity, 0.0, 0.0},
      {0.0, 0.0, 0.0, -1.0 / m_trackwidth, 1.0 / m_trackwidth},
      {0.0, 0.0, 0.0, m_A(0, 0), m_A(0, 1)},
      {0.0, 0.0, 0.0, m_A(1, 0), m_A(1, 1)}};
  Eigen::Matrix<double, 5, 2> B{{0.0, 0.0},
                                {0.0, 0.0},
                                {0.0, 0.0},
                                {m_B(0, 0), m_B(0, 1)},
                                {m_B(1, 0), m_B(1, 1)}};

  auto [disc_A, disc_B] = discretize_ab(A, B, m_dt);

  auto K = lqr(disc_A, disc_B, m_Q, m_R);

  Eigen::Matrix<double, 5, 5> in_robot_frame{
      {std::cos(x[2]), std::sin(x[2]), 0.0, 0.0, 0.0},
      {-std::sin(x[2]), std::cos(x[2]), 0.0, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 1.0}};

  Eigen::Vector2d u = K * in_robot_frame * m_error;

  return {u[0], u[1]};
}
