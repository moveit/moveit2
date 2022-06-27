#include "controller_manager_plugin.cpp"

// Testing
#include <gtest/gtest.h>

std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> generate_response(int num_chained_controller,
                                                                                           int rotation)
{
  auto result = std::make_shared<controller_manager_msgs::srv::ListControllers::Response>();
  controller_manager_msgs::msg::ControllerState cs;
  result->controller.resize(2 + num_chained_controller);

  for (int i = 0; i <= num_chained_controller; i++)
  {
    cs = result->controller[(i + rotation) % (num_chained_controller + 2)];
    cs.name = "chained_controller_" + std::to_string(i);
    std::string chained_to_name;
    if (i == num_chained_controller - 1)
    {
      chained_to_name = "admittance_controller";
    }
    else
    {
      chained_to_name = "chained_controller_" + std::to_string(i + 1);
    }

    cs.claimed_interfaces = {
      chained_to_name + "/shoulder_pan_joint/position",  chained_to_name + "/shoulder_pan_joint/velocity",
      chained_to_name + "/shoulder_lift_joint/position", chained_to_name + "/shoulder_lift_joint/velocity",
      chained_to_name + "/elbow_joint/position",         chained_to_name + "/elbow_joint/velocity",
      chained_to_name + "/wrist_1_joint/position",       chained_to_name + "/wrist_1_joint/velocity",
      chained_to_name + "/wrist_2_joint/position",       chained_to_name + "/wrist_2_joint/velocity",
      chained_to_name + "/wrist_3_joint/position",       chained_to_name + "/wrist_3_joint/velocity"
    };

    cs.claimed_interfaces.push_back("physical_interface" + std::to_string(i));

    result->controller[(i + rotation) % (num_chained_controller + 2)] = cs;
  }

  cs = result->controller[(num_chained_controller + rotation) % (num_chained_controller + 2)];
  cs.name = "admittance_controller";
  cs.claimed_interfaces = { "shoulder_pan_joint/position", "shoulder_lift_joint/position", "elbow_joint/position",
                            "wrist_1_joint/position",      "wrist_2_joint/position",       "wrist_3_joint/position" };
  cs.required_command_interfaces = cs.claimed_interfaces;
  result->controller[(num_chained_controller + rotation) % (num_chained_controller + 2)] = cs;

  cs = result->controller[(num_chained_controller + 1 + rotation) % (num_chained_controller + 2)];
  cs.name = "faked_forces_controller";
  cs.claimed_interfaces = { "tcp_fts_sensor/force.x",  "tcp_fts_sensor/force.y",  "tcp_fts_sensor/force.z",
                            "tcp_fts_sensor/torque.x", "tcp_fts_sensor/torque.y", "tcp_fts_sensor/torque.z" };
  cs.required_command_interfaces = cs.claimed_interfaces;
  result->controller[(num_chained_controller + 1 + rotation) % (num_chained_controller + 2)] = cs;
  return result;
}

TEST(ControllerManagerPluginTest, FixChainedCntrollerInControllerListTest)
{
  std::vector<std::string> expected_interfaces = { "physical_interface0",         "physical_interface1",
                                                   "shoulder_pan_joint/position", "shoulder_lift_joint/position",
                                                   "elbow_joint/position",        "wrist_1_joint/position",
                                                   "wrist_2_joint/position",      "wrist_3_joint/position" };

  int num_chained_controller = 2;
  for (int rotation = 0; rotation < 4; rotation++)
  {
    auto result = generate_response(num_chained_controller, rotation);
    moveit_ros_control_interface::MoveItControllerManager::fixChainedControllers(result);
    auto chained_controller = result->controller[0];
    if (chained_controller.name != "chained_controller_0")
    {
      chained_controller = result->controller[1];
    }

    ASSERT_EQ(expected_interfaces, chained_controller.claimed_interfaces);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();

  return result;
}
