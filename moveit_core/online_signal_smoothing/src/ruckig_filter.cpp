#include <moveit/online_signal_smoothing/ruckig_filter.h>

namespace online_signal_smoothing
{
namespace
{
constexpr double DEFAULT_MAX_VELOCITY = 5;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 20;          // rad/s^3
constexpr double VELOCITY_EPS_RAD_S = 1e-3;      // rad/s
}  // namespace

bool RuckigFilterPlugin::initialize(rclcpp::Node::SharedPtr node, const moveit::core::JointModelGroup& group,
                                    size_t num_dof, double timestep)
{
  num_dof_ = num_dof;
  node_ = node;
  ruckig_ = std::make_shared<ruckig::Ruckig<0, true /*print debug when failure*/>>(num_dof_, timestep);
  ruckig_input_ = std::make_shared<ruckig::InputParameter<0>>(num_dof_);
  ruckig_output_ = std::make_shared<ruckig::OutputParameter<0>>(num_dof_);

  // Velocity mode works best for smoothing one waypoint at a time
  ruckig_input_->control_interface = ruckig::ControlInterface::Velocity;
  ruckig_input_->synchronization = ruckig::Synchronization::Time;

  // Kinematic limits
  const std::vector<std::string>& vars = group.getVariableNames();
  const moveit::core::RobotModel& rmodel = group.getParentModel();
  for (size_t i = 0; i < num_dof_; ++i)
  {
    // TODO(andyz): read this from the joint group if/when jerk limits are added to the JointModel
    ruckig_input_->max_jerk.at(i) = DEFAULT_MAX_JERK;

    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars.at(i));
    // This assumes min/max bounds are symmetric
    if (bounds.velocity_bounded_)
    {
      ruckig_input_->max_velocity.at(i) = bounds.max_velocity_;
    }
    else
    {
      ruckig_input_->max_velocity.at(i) = DEFAULT_MAX_VELOCITY;
    }
    if (bounds.acceleration_bounded_)
    {
      ruckig_input_->max_acceleration.at(i) = bounds.max_acceleration_;
    }
    else
    {
      ruckig_input_->max_acceleration.at(i) = DEFAULT_MAX_ACCELERATION;
    }
  }

  return true;
};

bool RuckigFilterPlugin::doSmoothing(std::vector<double>& /*unused*/, std::vector<double>& velocity_vector)
{
  // Feed output from the previous timestep back as input
  for (size_t joint = 0; joint < num_dof_; ++joint)
  {
    // We don't update position since Ruckig is in velocity control mode
    ruckig_input_->current_velocity.at(joint) = ruckig_output_->new_velocity.at(joint);
    ruckig_input_->current_acceleration.at(joint) = ruckig_output_->new_acceleration.at(joint);

    // Target state is the next waypoint
    ruckig_input_->target_velocity.at(joint) = velocity_vector.at(joint);
    // Set a target of zero acceleration
    ruckig_input_->target_acceleration.at(joint) = 0;
  }

  // Ruckig (like other smoothing algorithms incl. TOTG) does not do well when the requested state almost matches the
  // current state. So, skip smoothing if (target_state ~= current_state)
  double magnitude_velocity_difference = 0;
  for (size_t joint = 0; joint < num_dof_; ++joint)
  {
    magnitude_velocity_difference +=
        pow((ruckig_input_->target_velocity.at(joint) - ruckig_input_->current_velocity.at(joint)), 2.);
  }
  magnitude_velocity_difference = sqrt(magnitude_velocity_difference);
  if (magnitude_velocity_difference < VELOCITY_EPS_RAD_S)
  {
    // velocity_vector is returned unmodified
    return true;
  }

  ruckig::Result result;
  if (ruckig_)
  {
    result = ruckig_->update(*ruckig_input_, *ruckig_output_);
  }
  else
  {
    return false;
  }

  // Update output
  if (result == ruckig::Result::Finished)
  {
    velocity_vector = ruckig_output_->new_velocity;
  }

  return result == ruckig::Result::Finished;
}

bool RuckigFilterPlugin::reset(const std::vector<double>& joint_positions)
{
  std::copy_n(joint_positions.begin(), num_dof_, ruckig_input_->current_position.begin());
  std::vector<double> zero_vector(num_dof_, 0);
  std::copy_n(zero_vector.begin(), num_dof_, ruckig_input_->current_velocity.begin());
  std::copy_n(zero_vector.begin(), num_dof_, ruckig_input_->current_acceleration.begin());

  // Initialize output data struct
  ruckig_output_->new_position = ruckig_input_->current_position;
  ruckig_output_->new_velocity = ruckig_input_->current_velocity;
  ruckig_output_->new_acceleration = ruckig_input_->current_acceleration;

  return true;
};

}  // namespace online_signal_smoothing

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(online_signal_smoothing::RuckigFilterPlugin, online_signal_smoothing::SmoothingBaseClass)
