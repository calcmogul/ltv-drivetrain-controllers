// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cmath>

#include <Eigen/Core>

#include "wpi/math/geometry/pose2d.hpp"
#include "wpi/math/util/cost_matrix.hpp"

namespace wpi::math {

/// Represents the speed of a robot chassis.
struct ChassisSpeeds {
  /// Velocity along the x-axis. (Fwd is +)
  double vx = 0.0;

  /// Velocity along the y-axis. (Left is +)
  double vy = 0.0;

  /// Represents the angular velocity of the robot frame. (CCW is +)
  double omega = 0.0;
};

/// The linear time-varying unicycle controller has a similar form to the LQR,
/// but the model used to compute the controller gain is the nonlinear unicycle
/// model linearized around the drivetrain's current state.
///
/// See section 8.9 in Controls Engineering in FRC for a derivation of the
/// control law we used shown in theorem 8.9.1.
class LTVUnicycleController {
 public:
  /// Constructs a linear time-varying unicycle controller with default maximum
  /// desired error tolerances of (0.0625 m, 0.125 m, 2 rad) and default maximum
  /// desired control effort of (1 m/s, 2 rad/s).
  ///
  /// @param dt Discretization timestep in seconds.
  explicit LTVUnicycleController(double dt)
      : LTVUnicycleController{{0.0625, 0.125, 2.0}, {1.0, 2.0}, dt} {}

  /// Constructs a linear time-varying unicycle controller.
  ///
  /// See
  /// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-tuning
  /// for how to select the tolerances.
  ///
  /// @param Q_elems The maximum desired error tolerance for each state.
  /// @param R_elems The maximum desired control effort for each input.
  /// @param dt Discretization timestep in seconds.
  LTVUnicycleController(const std::array<double, 3>& Q_elems,
                        const std::array<double, 2>& R_elems, double dt)
      : m_Q{cost_matrix(Q_elems)}, m_R{cost_matrix(R_elems)}, m_dt{dt} {}

  /// Move constructor.
  LTVUnicycleController(LTVUnicycleController&&) = default;

  /// Move assignment operator.
  LTVUnicycleController& operator=(LTVUnicycleController&&) = default;

  /// Returns true if the pose error is within tolerance of the reference.
  bool at_reference() const {
    const auto& e_translate = m_pose_error.translation;
    const auto& e_rotate = m_pose_error.rotation;
    const auto& tol_translate = m_pose_tolerance.translation;
    const auto& tol_rotate = m_pose_tolerance.rotation;
    return std::abs(e_translate.x) < tol_translate.x &&
           std::abs(e_translate.y) < tol_translate.y &&
           std::abs(e_rotate.radians()) < tol_rotate.radians();
  }

  /// Sets the pose error which is considered tolerable for use with
  /// at_reference().
  ///
  /// @param pose_tolerance Pose error which is tolerable.
  void set_tolerance(const Pose2d& pose_tolerance) {
    m_pose_tolerance = pose_tolerance;
  }

  /// Returns the linear and angular velocity outputs of the LTV controller.
  ///
  /// The reference pose, linear velocity, and angular velocity should come from
  /// a drivetrain trajectory.
  ///
  /// @param current_pose The current pose.
  /// @param pose_ref The desired pose.
  /// @param linear_velocity_ref The desired linear velocity.
  /// @param angular_velocity_ref The desired angular velocity.
  ChassisSpeeds calculate(const Pose2d& current_pose, const Pose2d& pose_ref,
                          double linear_velocity_ref,
                          double angular_velocity_ref);

  /// Enables and disables the controller for troubleshooting purposes.
  ///
  /// @param enabled If the controller is enabled or not.
  void set_enabled(bool enabled) { m_enabled = enabled; }

 private:
  // LQR cost matrices
  Eigen::Matrix<double, 3, 3> m_Q;
  Eigen::Matrix<double, 2, 2> m_R;

  double m_dt;

  Pose2d m_pose_error;
  Pose2d m_pose_tolerance;
  bool m_enabled = true;
};

}  // namespace wpi::math
