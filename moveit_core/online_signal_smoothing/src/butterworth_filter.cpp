#include <moveit/online_signal_smoothing/butterworth_filter.h>

namespace online_signal_smoothing
{
namespace
{
constexpr double EPSILON = 1e-9;
}

ButterworthFilter::ButterworthFilter(double low_pass_filter_coeff)
  : previous_measurements_{ 0., 0. }
  , previous_filtered_measurement_(0.)
  , scale_term_(1. / (1. + low_pass_filter_coeff))
  , feedback_term_(1. - low_pass_filter_coeff)
{
  // guarantee this doesn't change because the logic below depends on this length implicity
  static_assert(ButterworthFilter::FILTER_LENGTH == 2,
                "online_signal_smoothing::ButterworthFilter::FILTER_LENGTH should be 2");

  if (std::isinf(feedback_term_))
    throw std::length_error("online_signal_smoothing::ButterworthFilter: infinite feedback_term_");

  if (std::isinf(scale_term_))
    throw std::length_error("online_signal_smoothing::ButterworthFilter: infinite scale_term_");

  if (low_pass_filter_coeff < 1)
    throw std::length_error(
        "online_signal_smoothing::ButterworthFilter: Filter coefficient < 1. makes the lowpass filter unstable");

  if (std::abs(feedback_term_) < EPSILON)
    throw std::length_error(
        "online_signal_smoothing::ButterworthFilter: Filter coefficient value resulted in feedback term of 0");
}

double ButterworthFilter::filter(double new_measurement)
{
  // Push in the new measurement
  previous_measurements_[1] = previous_measurements_[0];
  previous_measurements_[0] = new_measurement;

  previous_filtered_measurement_ = scale_term_ * (previous_measurements_[1] + previous_measurements_[0] -
                                                  feedback_term_ * previous_filtered_measurement_);

  return previous_filtered_measurement_;
}

void ButterworthFilter::reset(const double data)
{
  previous_measurements_[0] = data;
  previous_measurements_[1] = data;
  previous_filtered_measurement_ = data;
}

bool ButterworthFilterPlugin::initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                                         size_t num_joints)
{
  node_ = node;
  num_joints_ = num_joints;

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // Low-pass filters for the joint positions
    // TODO(andyz): read a parameter
    position_filters_.emplace_back(1.5 /* filter coefficient, should be >1 */);
  }
  return true;
};

bool ButterworthFilterPlugin::doSmoothing(std::vector<double>& position_vector)
{
  if (position_vector.size() != position_filters_.size())
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Position vector to be smoothed does not have the right length.");
    return false;
  }
  for (size_t i = 0; i < position_vector.size(); ++i)
  {
    // Lowpass filter the position command
    position_vector[i] = position_filters_.at(i).filter(position_vector[i]);
  }
  return true;
};

bool ButterworthFilterPlugin::reset(const std::vector<double>& joint_positions)
{
  if (joint_positions.size() != position_filters_.size())
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Position vector to be reset does not have the right length.");
    return false;
  }
  for (size_t joint_idx = 0; joint_idx < joint_positions.size(); ++joint_idx)
  {
    position_filters_.at(joint_idx).reset(joint_positions.at(joint_idx));
  }
  return true;
};

}  // namespace online_signal_smoothing

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(online_signal_smoothing::ButterworthFilterPlugin, online_signal_smoothing::SmoothingBaseClass)
