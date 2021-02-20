/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : joystick_to_servo.hpp
 *      Project   : moveit_servo
 *      Created   : 18/02/2021
 *      Author    : Denis Stogl
 */

#include <memory>
#include <string>
#include <mutex>
#include <vector>

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace moveit_servo
{
class JoystickToServo : public rclcpp::Node
{
public:
  JoystickToServo(const rclcpp::NodeOptions& options);

  ~JoystickToServo() = default;

  // To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
  // functions
  /** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
   * @param axes The vector of continuous controller joystick axes
   * @param buttons The vector of discrete controller button values
   */
  void convertJoyToCmd(const std::vector<float> & axes, const std::vector<int> & buttons);

  // Callbacks

  void inputTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

protected:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  int enable_joint_commands_button_ = -1;
  int enable_twist_commands_button_ = -1;
  int switch_cartesian_command_frame_button_ = -1;

  std::vector<std::string> cartesian_command_frames_;
  std::vector<std::string> joint_names_;
  std::vector<int> joint_axes_;

  int command_frame_index_ = 0;

  geometry_msgs::msg::Twist::SharedPtr input_twist_;

  std::recursive_mutex twist_update_lock_;
};  // class JoystickToServo

}  // namespace moveit_servo
