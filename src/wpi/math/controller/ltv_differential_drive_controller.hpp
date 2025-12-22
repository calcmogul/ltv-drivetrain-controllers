// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cmath>

#include <Eigen/Core>

#include "wpi/math/geometry/pose2d.hpp"
#include "wpi/math/system/linear_system.hpp"
#include "wpi/math/util/cost_matrix.hpp"

namespace wpi::math {

/// Motor voltages for a differential drive.
struct DifferentialDriveWheelVoltages {
  /// Left wheel voltage.
  double left = 0.0;

  /// Right wheel voltage.
  double right = 0.0;
};

/// The linear time-varying differential drive controller has a similar form to
/// the LQR, but the model used to compute the controller gain is the nonlinear
/// differential drive model linearized around the drivetrain's current state.
/// We precompute gains for important places in our state-space, then
/// interpolate between them with a lookup table to save computational
/// resources.
///
/// This controller has a flat hierarchy with pose and wheel velocity references
/// and voltage outputs. This is different from a unicycle controller's nested
/// hierarchy where the top-level controller has a pose reference and chassis
/// velocity command outputs, and the low-level controller has wheel velocity
/// references and voltage outputs. Flat hierarchies are easier to tune in one
/// shot.
///
/// See section 8.7 in Controls Engineering in FRC for a derivation of the
/// control law we used shown in theorem 8.7.4.
class LTVDifferentialDriveController {
 public:
  /// Constructs a linear time-varying differential drive controller.
  ///
  /// See
  /// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-tuning
  /// for how to select the tolerances.
  ///
  /// @param plant The differential drive velocity plant.
  /// @param trackwidth The distance between the differential drive's left and
  ///     right wheels.
  /// @param Q_elems The maximum desired error tolerance for each state.
  /// @param R_elems The maximum desired control effort for each input.
  /// @param dt Discretization timestep.
  LTVDifferentialDriveController(const LinearSystem<2, 2, 2>& plant,
                                 double trackwidth,
                                 const std::array<double, 5>& Q_elems,
                                 const std::array<double, 2>& R_elems,
                                 double dt)
      : m_trackwidth{trackwidth},
        m_A{plant.A},
        m_B{plant.B},
        m_Q{cost_matrix(Q_elems)},
        m_R{cost_matrix(R_elems)},
        m_dt{dt} {}

  /// Move constructor.
  LTVDifferentialDriveController(LTVDifferentialDriveController&&) = default;

  /// Move assignment operator.
  LTVDifferentialDriveController& operator=(LTVDifferentialDriveController&&) =
      default;

  /// Returns true if the pose error is within tolerance of the reference.
  bool at_reference() const {
    return std::abs(m_error(0)) < m_tolerance(0) &&
           std::abs(m_error(1)) < m_tolerance(1) &&
           std::abs(m_error(2)) < m_tolerance(2) &&
           std::abs(m_error(3)) < m_tolerance(3) &&
           std::abs(m_error(4)) < m_tolerance(4);
  }

  /// Sets the pose error which is considered tolerable for use with
  /// at_reference().
  ///
  /// @param pose_tolerance Pose error which is tolerable.
  /// @param left_velocity_tolerance Left velocity error which is tolerable.
  /// @param right_velocity_tolerance Right velocity error which is tolerable.
  void set_tolerance(const Pose2d& pose_tolerance,
                     double left_velocity_tolerance,
                     double right_velocity_tolerance) {
    m_tolerance = Eigen::Vector<double, 5>{
        pose_tolerance.translation.x, pose_tolerance.translation.y,
        pose_tolerance.rotation.radians(), left_velocity_tolerance,
        right_velocity_tolerance};
  }

  /// Returns the left and right output voltages of the LTV controller.
  ///
  /// The reference pose, linear velocity, and angular velocity should come from
  /// a drivetrain trajectory.
  ///
  /// @param current_pose The current pose.
  /// @param left_velocity The current left velocity.
  /// @param right_velocity The current right velocity.
  /// @param pose_ref The desired pose.
  /// @param left_velocity_ref The desired left velocity.
  /// @param right_velocity_ref The desired right velocity.
  DifferentialDriveWheelVoltages calculate(const Pose2d& current_pose,
                                           double left_velocity,
                                           double right_velocity,
                                           const Pose2d& pose_ref,
                                           double left_velocity_ref,
                                           double right_velocity_ref);

 private:
  double m_trackwidth;

  // Continuous velocity dynamics
  Eigen::Matrix<double, 2, 2> m_A;
  Eigen::Matrix<double, 2, 2> m_B;

  // LQR cost matrices
  Eigen::Matrix<double, 5, 5> m_Q;
  Eigen::Matrix<double, 2, 2> m_R;

  double m_dt;

  Eigen::Vector<double, 5> m_error;
  Eigen::Vector<double, 5> m_tolerance;
};

}  // namespace wpi::math
