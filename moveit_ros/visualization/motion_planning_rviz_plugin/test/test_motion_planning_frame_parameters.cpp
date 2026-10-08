#include "motion_planning_frame_parameters.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace moveit_rviz_plugin
{
namespace detail
{

class MotionPlanningFrameParametersTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(MotionPlanningFrameParametersTest, ReadsRvizParameterInsteadOfNamespacedLookalike)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({ rclcpp::Parameter("default_planning_pipeline", "ompl") });

  auto node = std::make_shared<rclcpp::Node>("motion_planning_frame_parameters_test", options);
  node->declare_parameter<std::string>("/robotdefault_planning_pipeline", "pilz_industrial_motion_planner");

  EXPECT_EQ(getDefaultPlanningPipeline(node), "ompl");
}

TEST_F(MotionPlanningFrameParametersTest, DeclaresEmptyDefaultWhenParameterIsMissing)
{
  auto node = std::make_shared<rclcpp::Node>("motion_planning_frame_parameters_default_test");

  EXPECT_EQ(getDefaultPlanningPipeline(node), "");
  EXPECT_TRUE(node->has_parameter("default_planning_pipeline"));
}

}  // namespace detail
}  // namespace moveit_rviz_plugin
