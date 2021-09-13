#include <moveit_servo/smoothing_plugin/low_pass_filter.h>

namespace moveit_servo
{
namespace
{
constexpr double EPSILON = 1e-9;
}

LowpassFilterImpl::LowpassFilterImpl(double low_pass_filter_coeff)
  : previous_measurements_{ 0., 0. }
  , previous_filtered_measurement_(0.)
  , scale_term_(1. / (1. + low_pass_filter_coeff))
  , feedback_term_(1. - low_pass_filter_coeff)
{
  // guarantee this doesn't change because the logic below depends on this length implicity
  static_assert(LowpassFilterImpl::FILTER_LENGTH == 2, "moveit_servo::LowpassFilterImpl::FILTER_LENGTH should be 2");

  if (std::isinf(feedback_term_))
    throw std::length_error("moveit_servo::LowpassFilterImpl: infinite feedback_term_");

  if (std::isinf(scale_term_))
    throw std::length_error("moveit_servo::LowpassFilterImpl: infinite scale_term_");

  if (low_pass_filter_coeff < 1)
    throw std::length_error(
        "moveit_servo::LowpassFilterImpl: Filter coefficient < 1. makes the lowpass filter unstable");

  if (std::abs(feedback_term_) < EPSILON)
    throw std::length_error("moveit_servo::LowpassFilterImpl: Filter coefficient value resulted in feedback term of 0");
}

double LowpassFilterImpl::filter(double new_measurement)
{
  // Push in the new measurement
  previous_measurements_[1] = previous_measurements_[0];
  previous_measurements_[0] = new_measurement;

  double new_filtered_measurement = scale_term_ * (previous_measurements_[1] + previous_measurements_[0] -
                                                   feedback_term_ * previous_filtered_measurement_);

  // Store the new filtered measurement
  previous_filtered_measurement_ = new_filtered_measurement;

  return new_filtered_measurement;
}

void LowpassFilterImpl::reset(double data)
{
  previous_measurements_[0] = data;
  previous_measurements_[1] = data;

  previous_filtered_measurement_ = data;
}

bool LowPassFilter::initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                               const size_t num_joints,
                               const std::shared_ptr<const moveit_servo::ServoParameters>& parameters)
{
  node_ = node;
  num_joints_ = num_joints;

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // Low-pass filters for the joint positions
    position_filters_.emplace_back(parameters->low_pass_filter_coeff);
  }

  return true;
};

bool LowPassFilter::doSmoothing(Eigen::ArrayXd& delta_theta)
{
  for (uint i = 0; i < delta_theta.size(); ++i)
  {
    // Lowpass filter the position command
    delta_theta[i] = position_filters_.at(i).filter(delta_theta[i]);
  }
  return true;
};

}  // namespace moveit_servo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_servo::LowPassFilter, moveit_servo::SmoothingBaseClass)
