#pragma once

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <Eigen/Geometry>

namespace trajectory_processing
{
/**
 * \class LinearPathSegment
 *
 * \brief Subclass of PathSegment defining a linear segment.
 *
 * This class is a subclass of PathSegment and defines a linear segment. It has a constructor which takes two
 * Eigen::VectorXd objects, start and end, and sets the length to the distance between them. It also contains methods
 * to get the configuration of the segment at a given point, the tangent of the segment at a given point, the curvature
 * of the segment at a given point, and a list of switching points. Lastly, it contains a clone method to create a deep
 * copy of the LinearPathSegment.
 */
class LinearPathSegment : public PathSegment
{
public:
  /**
   * \brief Constructor for the LinearPathSegment class.
   *
   * This constructor takes two Eigen::VectorXd objects and sets the length of the segment to the distance between them.
   *
   * \param start The start position of the segment.
   * \param end The end position of the segment.
   */
  LinearPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& end)
    : PathSegment((end - start).norm()), end_(end), start_(start)
  {
  }

  /**
   * \brief Gets the configuration of the segment at a given point.
   *
   * This method gets the configuration of the segment at a given point.
   *
   * \param s The point at which to get the configuration.
   * \return The configuration of the segment at the given point.
   */
  Eigen::VectorXd getConfig(double s) const override
  {
    s /= length_;
    s = std::max(0.0, std::min(1.0, s));
    return (1.0 - s) * start_ + s * end_;
  }

  /**
   * \brief Gets the tangent of the segment at a given point.
   *
   * This method gets the tangent of the segment at a given point.
   *
   * \param s The point at which to get the tangent.
   * \return The tangent of the segment at the given point.
   */
  Eigen::VectorXd getTangent(double /* s */) const override
  {
    return (end_ - start_) / length_;
  }

  /**
   * \brief Gets the curvature of the segment at a given point.
   *
   * This method gets the curvature of the segment at a given point.
   *
   * \param s The point at which to get the curvature.
   * \return The curvature of the segment at the given point.
   */
  Eigen::VectorXd getCurvature(double /* s */) const override
  {
    return Eigen::VectorXd::Zero(start_.size());
  }

  /**
   * \brief Gets a list of switching points for the segment.
   *
   * This method gets a list of switching points for the segment.
   *
   * \return A list of switching points for the segment.
   */
  std::list<double> getSwitchingPoints() const override
  {
    return std::list<double>();
  }

  /**
   * \brief Creates a deep copy of the LinearPathSegment.
   *
   * This method creates a deep copy of the LinearPathSegment.
   *
   * \return A deep copy of the LinearPathSegment.
   */
  LinearPathSegment* clone() const override
  {
    return new LinearPathSegment(*this);
  }

  bool operator==(const LinearPathSegment& rhs) const
  {
    return (length_ == rhs.length_ && end_ == rhs.end_ && start_ == rhs.start_);
  }

private:
  Eigen::VectorXd end_;
  Eigen::VectorXd start_;
};

}  // namespace trajectory_processing
