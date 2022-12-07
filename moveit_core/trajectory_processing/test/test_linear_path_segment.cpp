#include <gtest/gtest.h>
#include <moveit/trajectory_processing/linear_path_segment.hpp>

using trajectory_processing::LinearPathSegment;

// Tests LinearPathSegment constructor.
TEST(LinearPathSegment, Constructor)
{
  Eigen::VectorXd start(2);
  start << 0.0, 0.0;
  Eigen::VectorXd end(2);
  end << 3.0, 4.0;
  LinearPathSegment segment(start, end);

  // Check that the length of the segment is correct.
  EXPECT_NEAR(segment.getLength(), 5.0, 1e-6);
}

// Tests LinearPathSegment::getConfig() method.
TEST(LinearPathSegment, GetConfig)
{
  Eigen::VectorXd start(2);
  start << 0.0, 0.0;
  Eigen::VectorXd end(2);
  end << 3.0, 4.0;
  LinearPathSegment segment(start, end);

  // Check that the configuration at 0 is equal to the start.
  EXPECT_EQ(segment.getConfig(0.0), start);

  // Check that the configuration at the length is equal to the end.
  EXPECT_EQ(segment.getConfig(segment.getLength()), end);

  // Check that the configuration at an intermediate point is correct.
  Eigen::VectorXd config(2);
  config << 1.5, 2.0;
  EXPECT_EQ(segment.getConfig(segment.getLength() / 2.0), config);
}

// Tests LinearPathSegment::getTangent() method.
TEST(LinearPathSegment, GetTangent)
{
  Eigen::VectorXd start(2);
  start << 0.0, 0.0;
  Eigen::VectorXd end(2);
  end << 3.0, 4.0;
  LinearPathSegment segment(start, end);

  // Check that the tangent is correct.
  Eigen::VectorXd tangent(2);
  tangent << 3.0 / 5.0, 4.0 / 5.0;
  EXPECT_EQ(segment.getTangent(0.0), tangent);
}

// Tests LinearPathSegment::getCurvature() method.
TEST(LinearPathSegment, GetCurvature)
{
  Eigen::VectorXd start(2);
  start << 0.0, 0.0;
  Eigen::VectorXd end(2);
  end << 3.0, 4.0;
  LinearPathSegment segment(start, end);

  // Check that the curvature is zero.
  EXPECT_EQ(segment.getCurvature(0.0), Eigen::VectorXd::Zero(start.size()));
}

// Tests LinearPathSegment::getSwitchingPoints() method.
TEST(LinearPathSegment, GetSwitchingPoints)
{
  Eigen::VectorXd start(2);
  start << 0.0, 0.0;
  Eigen::VectorXd end(2);
  end << 3.0, 4.0;
  LinearPathSegment segment(start, end);

  // Check that the list of switching points is empty.
  EXPECT_TRUE(segment.getSwitchingPoints().empty());
}

// Tests LinearPathSegment::clone() method.
TEST(LinearPathSegment, Clone)
{
  Eigen::VectorXd start(2);
  start << 0.0, 0.0;
  Eigen::VectorXd end(2);
  end << 3.0, 4.0;
  LinearPathSegment segment(start, end);

  // Clone the segment and check that the two segments are equal.
  LinearPathSegment* clone = segment.clone();
  EXPECT_EQ(segment, *clone);

  // Clean up.
  delete clone;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
