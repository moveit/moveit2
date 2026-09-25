#include "pseudoinverse.hpp"

#include <Eigen/QR>
#include <Eigen/SVD>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
Eigen::MatrixXd matrixWithSingularValues(const std::vector<double>& values, Eigen::Index columns)
{
  const Eigen::Index rows = values.size();
  Eigen::MatrixXd left(rows, rows);
  Eigen::MatrixXd right(columns, rows);
  for (Eigen::Index row = 0; row < rows; ++row)
  {
    for (Eigen::Index column = 0; column < rows; ++column)
      left(row, column) = std::sin((row + 1) * (column + 2));
    for (Eigen::Index column = 0; column < columns; ++column)
      right(column, row) = std::cos((row + 2) * (column + 1));
  }
  const Eigen::MatrixXd u = left.householderQr().householderQ() * Eigen::MatrixXd::Identity(rows, rows);
  const Eigen::MatrixXd v = right.householderQr().householderQ() * Eigen::MatrixXd::Identity(columns, rows);
  return u * Eigen::Map<const Eigen::VectorXd>(values.data(), rows).asDiagonal() * v.transpose();
}

void expectSameSolution(const std::vector<double>& singular_values)
{
  constexpr double threshold = 0.001;
  constexpr Eigen::Index columns = 7;
  const Eigen::MatrixXd jacobian = matrixWithSingularValues(singular_values, columns);
  const Eigen::VectorXd input = Eigen::VectorXd::LinSpaced(jacobian.rows(), -0.7, 0.9);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian.rows(), jacobian.cols(), Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(threshold);
  svd.compute(jacobian);
  const Eigen::VectorXd expected = svd.solve(input);
  Eigen::MatrixXd jacobian_storage = Eigen::MatrixXd::Zero(jacobian.rows() + 1, jacobian.cols());
  Eigen::VectorXd input_storage = Eigen::VectorXd::Zero(input.size() + 1);
  jacobian_storage.topRows(jacobian.rows()) = jacobian;
  input_storage.topRows(input.size()) = input;
  const Eigen::VectorXd actual = kdl_kinematics_plugin::internal::solvePseudoinverse(
      jacobian_storage.topRows(jacobian.rows()), input_storage.topRows(input.size()), threshold);
  EXPECT_LT((expected - actual).norm(), 1e-8 * std::max(1.0, expected.norm()));
  EXPECT_NEAR((jacobian * expected - input).norm(), (jacobian * actual - input).norm(), 1e-9);
}
}  // namespace

TEST(Pseudoinverse, FullRank)
{
  expectSameSolution({ 1.0, 0.7, 0.4, 0.2, 0.1, 0.02 });
  expectSameSolution({ 1.0, 0.3, 0.02 });
}

TEST(Pseudoinverse, RankDeficient)
{
  expectSameSolution({ 1.0, 0.4, 0.1, 0.0, 0.0, 0.0 });
}

TEST(Pseudoinverse, ThresholdBoundary)
{
  expectSameSolution({ 1.0, 0.4, 0.1, 0.01, 0.001001, 0.0005 });
  expectSameSolution({ 1.0, 0.4, 0.1, 0.01, 0.000999, 0.0005 });
}

TEST(Pseudoinverse, IllConditioned)
{
  expectSameSolution({ 1.0, 0.01, 0.0001, 1e-6, 1e-8, 0.0 });
}
