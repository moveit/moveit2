#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace trajectory_processing
{
/**
 * @brief Base class for trajectory parameterization algorithms
 */
MOVEIT_CLASS_FORWARD(TimeParameterization);
class TimeParameterization
{
public:
  TimeParameterization() = default;
  TimeParameterization(const TimeParameterization&) = default;
  TimeParameterization(TimeParameterization&&) = default;
  TimeParameterization& operator=(const TimeParameterization&) = default;
  TimeParameterization& operator=(TimeParameterization&&) = default;
  virtual ~TimeParameterization() = default;

  /**
   * \brief Compute a trajectory with waypoints spaced equally in time
   * \param[in,out] trajectory A path which needs time-parameterization. It's OK if this path has already been
   * time-parameterized; this function will re-time-parameterize it.
   * \param max_velocity_scaling_factor A factor in the range [0,1] which can slow down the trajectory.
   * \param max_acceleration_scaling_factor A factor in the range [0,1] which can slow down the trajectory.
   */
  virtual bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                                 const double max_velocity_scaling_factor = 1.0,
                                 const double max_acceleration_scaling_factor = 1.0) const = 0;

  /**
   * \brief Compute a trajectory with waypoints spaced equally in time
   * \param[in,out] trajectory A path which needs time-parameterization. It's OK if this path has already been
   * time-parameterized; this function will re-time-parameterize it.
   * \param velocity_limits Joint names and velocity limits in rad/s
   * \param acceleration_limits Joint names and acceleration limits in rad/s^2
   * \param max_velocity_scaling_factor A factor in the range [0,1] which can slow down the trajectory.
   * \param max_acceleration_scaling_factor A factor in the range [0,1] which can slow down the trajectory.
   */
  virtual bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                                 const std::unordered_map<std::string, double>& velocity_limits,
                                 const std::unordered_map<std::string, double>& acceleration_limits,
                                 const double max_velocity_scaling_factor = 1.0,
                                 const double max_acceleration_scaling_factor = 1.0) const = 0;

  /**
   * \brief Compute a trajectory with waypoints spaced equally in time
   * \param[in,out] trajectory A path which needs time-parameterization. It's OK if this path has already been
   * time-parameterized; this function will re-time-parameterize it.
   * \param joint_limits Joint names and corresponding velocity limits in rad/s and acceleration limits in rad/s^2
   * \param max_velocity_scaling_factor A factor in the range [0,1] which can slow down the trajectory.
   * \param max_acceleration_scaling_factor A factor in the range [0,1] which can slow down the trajectory.
   */
  virtual bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                                 const std::vector<moveit_msgs::msg::JointLimits>& joint_limits,
                                 const double max_velocity_scaling_factor = 1.0,
                                 const double max_acceleration_scaling_factor = 1.0) const = 0;
};
}  // namespace trajectory_processing
