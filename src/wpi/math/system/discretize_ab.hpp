// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <tuple>
#include <utility>

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

namespace wpi::math {

/// Discretizes the given continuous A and B matrices.
///
/// @tparam States Number of states.
/// @tparam Inputs Number of inputs.
/// @param cont_A Continuous system matrix.
/// @param cont_B Continuous input matrix.
/// @param dt Discretization timestep.
/// @return The discrete system and input matrices.
template <int States, int Inputs>
std::tuple<Eigen::Matrix<double, States, States>,
           Eigen::Matrix<double, States, Inputs>>
discretize_ab(const Eigen::Matrix<double, States, States>& cont_A,
              const Eigen::Matrix<double, States, Inputs>& cont_B, double dt) {
  // M = [A  B]
  //     [0  0]
  Eigen::Matrix<double, States + Inputs, States + Inputs> M;
  M.template block<States, States>(0, 0) = cont_A;
  M.template block<States, Inputs>(0, States) = cont_B;
  M.template block<Inputs, States + Inputs>(States, 0).setZero();

  // ϕ = eᴹᵀ = [A_d  B_d]
  //           [ 0    I ]
  Eigen::Matrix<double, States + Inputs, States + Inputs> phi = (M * dt).exp();

  Eigen::Matrix<double, States, States> disc_A =
      phi.template block<States, States>(0, 0);
  Eigen::Matrix<double, States, Inputs> disc_B =
      phi.template block<States, Inputs>(0, States);

  return {std::move(disc_A), std::move(disc_B)};
}

}  // namespace wpi::math
