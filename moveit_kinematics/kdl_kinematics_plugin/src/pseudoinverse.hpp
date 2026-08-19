#pragma once

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace kdl_kinematics_plugin::internal
{
inline Eigen::VectorXd solvePseudoinverse(const Eigen::Ref<const Eigen::MatrixXd>& jacobian,
                                          const Eigen::Ref<const Eigen::VectorXd>& input, double threshold)
{
  const Eigen::MatrixXd normal = jacobian * jacobian.transpose();
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(normal);
  const Eigen::VectorXd& eigenvalues = solver.eigenvalues();
  const Eigen::MatrixXd& eigenvectors = solver.eigenvectors();
  const double singular_max = std::sqrt(std::max(0.0, eigenvalues.maxCoeff()));
  const double absolute_threshold = threshold * singular_max;
  Eigen::VectorXd projected = eigenvectors.transpose() * input;
  for (Eigen::Index index = 0; index < eigenvalues.size(); ++index)
  {
    const double singular = std::sqrt(std::max(0.0, eigenvalues(index)));
    projected(index) = singular > absolute_threshold ? projected(index) / eigenvalues(index) : 0.0;
  }
  return jacobian.transpose() * (eigenvectors * projected);
}
}  // namespace kdl_kinematics_plugin::internal
