/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Acorn Pooley */

#pragma once

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection/world_diff.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/macros/class_forward.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <octomap_msgs/msg/octomap_with_pose.hpp>
#include <memory>
#include <functional>
#include <optional>
#include <thread>
#include <variant>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include <moveit_planning_scene_export.h>

/** \brief This namespace includes the central class for representing planning contexts */
namespace planning_scene
{
MOVEIT_CLASS_FORWARD(PlanningScene);  // Defines PlanningScenePtr, ConstPtr, WeakPtr... etc

/** \brief This is the function signature for additional feasibility checks to be imposed on states (in addition to
   respecting constraints and collision avoidance).
    The first argument is the state to check the feasibility for, the second one is whether the check should be verbose
   or not. */
typedef std::function<bool(const moveit::core::RobotState&, bool)> StateFeasibilityFn;

/** \brief This is the function signature for additional feasibility checks to be imposed on motions segments between
   states (in addition to respecting constraints and collision avoidance).
    The order of the arguments matters: the notion of feasibility is to be checked for motion segments that start at the
   first state and end at the second state. The third argument indicates
    whether the check should be verbose or not. */
using MotionFeasibilityFn = std::function<bool(const moveit::core::RobotState&, const moveit::core::RobotState&, bool)>;

/** \brief A map from object names (e.g., attached bodies, collision objects) to their colors */
using ObjectColorMap = std::map<std::string, std_msgs::msg::ColorRGBA>;

/** \brief A map from object names (e.g., attached bodies, collision objects) to their types */
using ObjectTypeMap = std::map<std::string, object_recognition_msgs::msg::ObjectType>;

/** \brief This class maintains the representation of the
    environment as seen by a planning instance. The environment
    geometry, the robot geometry and state are maintained. */
class MOVEIT_PLANNING_SCENE_EXPORT PlanningScene : public std::enable_shared_from_this<PlanningScene>
{
public:
  /**
   * @brief PlanningScene cannot be copy-constructed
   */
  PlanningScene(const PlanningScene&) = delete;

  /**
   * @brief PlanningScene cannot be copy-assigned
   */
  PlanningScene& operator=(const PlanningScene&) = delete;

  /** \brief construct using an existing RobotModel */
  PlanningScene(const moveit::core::RobotModelConstPtr& robot_model,
                const collision_detection::WorldPtr& world = std::make_shared<collision_detection::World>());

  /** \brief construct using a urdf and srdf.
   * A RobotModel for the PlanningScene will be created using the urdf and srdf. */
  PlanningScene(const urdf::ModelInterfaceSharedPtr& urdf_model, const srdf::ModelConstSharedPtr& srdf_model,
                const collision_detection::WorldPtr& world = std::make_shared<collision_detection::World>());

  static const std::string OCTOMAP_NS;
  static const std::string DEFAULT_SCENE_NAME;

  ~PlanningScene();

  /** \brief Get the name of the planning scene. This is empty by default */
  const std::string& getName() const
  {
    return name_;
  }

  /** \brief Set the name of the planning scene */
  void setName(const std::string& name)
  {
    name_ = name;
  }

  /** \brief Return a new child PlanningScene that uses this one as parent.
   *
   *  The child scene has its own copy of the world. It maintains a list (in
   *  world_diff_) of changes made to the child world.
   *
   *  The robot_model_, robot_state_, scene_transforms_, and acm_ are not copied.
   *  They are shared with the parent.  So if changes to these are made in the parent they will be visible in the child.
   * But if any of these is modified (i.e. if the get*NonConst functions are called) in the child then a copy is made
   * and subsequent changes to the corresponding member of the parent will no longer be visible in the child.
   */
  PlanningScenePtr diff() const;

  /** \brief Return a new child PlanningScene that uses this one as parent and
   * has the diffs specified by \e msg applied. */
  PlanningScenePtr diff(const moveit_msgs::msg::PlanningScene& msg) const;

  /** \brief Get the parent scene (with respect to which the diffs are maintained). This may be empty */
  const PlanningSceneConstPtr& getParent() const
  {
    return parent_;
  }

  /** \brief Get the kinematic model for which the planning scene is maintained */
  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    // the kinematic model does not change
    return robot_model_;
  }

  /** \brief Get the state at which the robot is assumed to be. */
  const moveit::core::RobotState& getCurrentState() const
  {
    // if we have an updated state, return it; otherwise, return the parent one
    return robot_state_.has_value() ? robot_state_.value() : parent_->getCurrentState();
  }
  /** \brief Get the state at which the robot is assumed to be. */
  moveit::core::RobotState& getCurrentStateNonConst();

  /** \brief Get a copy of the current state with components overwritten by the state message \e update */
  moveit::core::RobotStatePtr getCurrentStateUpdated(const moveit_msgs::msg::RobotState& update) const;

  /**
   * \name Reasoning about frames
   */
  /**@{*/

  /** \brief Get the frame in which planning is performed */
  const std::string& getPlanningFrame() const
  {
    // if we have an updated set of transforms, return it; otherwise, return the parent one
    return scene_transforms_.has_value() ? scene_transforms_.value()->getTargetFrame() : parent_->getPlanningFrame();
  }

  /** \brief Get the set of fixed transforms from known frames to the planning frame */
  const moveit::core::Transforms& getTransforms() const
  {
    if (scene_transforms_.has_value() || !parent_)
    {
      return *scene_transforms_.value();
    }

    // if this planning scene is a child of another, and doesn't have its own custom transforms
    return parent_->getTransforms();
  }

  /** \brief Get the set of fixed transforms from known frames to the planning frame. This variant is non-const and also
   * updates the current state */
  const moveit::core::Transforms& getTransforms();

  /** \brief Get the set of fixed transforms from known frames to the planning frame */
  moveit::core::Transforms& getTransformsNonConst();

  /** \brief Get the transform corresponding to the frame \e id. This will be known if \e id is a link name, an attached
     body id or a collision object.
      Return identity when no transform is available. Use knowsFrameTransform() to test if this function will be
     successful or not. */
  const Eigen::Isometry3d& getFrameTransform(const std::string& id) const;

  /** \brief Get the transform corresponding to the frame \e id. This will be known if \e id is a link name, an attached
     body id or a collision object.
      Return identity when no transform is available. Use knowsFrameTransform() to test if this function will be
     successful or not.
      Because this function is non-const, the current state transforms are also updated, if needed. */
  const Eigen::Isometry3d& getFrameTransform(const std::string& id);

  /** \brief Get the transform corresponding to the frame \e id. This will be known if \e id is a link name, an attached
     body id or a collision object.
      Return identity when no transform is available. Use knowsFrameTransform() to test if this function will be
     successful or not. This function also
      updates the link transforms of \e state. */
  const Eigen::Isometry3d& getFrameTransform(moveit::core::RobotState& state, const std::string& id) const
  {
    state.updateLinkTransforms();
    return getFrameTransform(static_cast<const moveit::core::RobotState&>(state), id);
  }

  /** \brief Get the transform corresponding to the frame \e id. This will be known if \e id is a link name, an attached
     body id or a collision object.
      Return identity when no transform is available. Use knowsFrameTransform() to test if this function will be
     successful or not. */
  const Eigen::Isometry3d& getFrameTransform(const moveit::core::RobotState& state, const std::string& id) const;

  /** \brief Check if a transform to the frame \e id is known. This will be known if \e id is a link name, an attached
   * body id or a collision object */
  bool knowsFrameTransform(const std::string& id) const;

  /** \brief Check if a transform to the frame \e id is known. This will be known if \e id is a link name, an attached
   * body id or a collision object */
  bool knowsFrameTransform(const moveit::core::RobotState& state, const std::string& id) const;

  /**@}*/

  /**
   * \name Reasoning about the geometry of the planning scene
   */
  /**@{*/

  const std::string getCollisionDetectorName() const
  {
    // If no collision detector is allocated, return an empty string
    return collision_detector_ ? (collision_detector_->alloc_->getName()) : "";
  }

  /** \brief Get the representation of the world */
  const collision_detection::WorldConstPtr& getWorld() const
  {
    // we always have a world representation
    return world_const_;
  }

  /** \brief Get the representation of the world */
  const collision_detection::WorldPtr& getWorldNonConst()
  {
    // we always have a world representation
    return world_;
  }

  /** \brief Get the active collision environment */
  const collision_detection::CollisionEnvConstPtr& getCollisionEnv() const
  {
    return collision_detector_->getCollisionEnv();
  }

  /** \brief Get the active collision detector for the robot */
  const collision_detection::CollisionEnvConstPtr& getCollisionEnvUnpadded() const
  {
    return collision_detector_->getCollisionEnvUnpadded();
  }

  /** \brief Get a specific collision detector for the world.  If not found return active CollisionWorld. */
  const collision_detection::CollisionEnvConstPtr& getCollisionEnv(const std::string& collision_detector_name) const;

  /** \brief Get a specific collision detector for the unpadded robot.  If no found return active unpadded
   * CollisionRobot. */
  const collision_detection::CollisionEnvConstPtr&
  getCollisionEnvUnpadded(const std::string& collision_detector_name) const;

  /** \brief Get the representation of the collision robot
   * This can be used to set padding and link scale on the active collision_robot. */
  const collision_detection::CollisionEnvPtr& getCollisionEnvNonConst();

  /** \brief Get the allowed collision matrix */
  const collision_detection::AllowedCollisionMatrix& getAllowedCollisionMatrix() const
  {
    return acm_.has_value() ? acm_.value() : parent_->getAllowedCollisionMatrix();
  }

  /** \brief Get the allowed collision matrix */
  collision_detection::AllowedCollisionMatrix& getAllowedCollisionMatrixNonConst();

  /** \brief Set the allowed collision matrix */
  void setAllowedCollisionMatrix(const collision_detection::AllowedCollisionMatrix& acm);

  /**@}*/

  /**
   * \name Collision checking with respect to this planning scene
   */
  /**@{*/

  /** \brief Check if the current state is in collision (with the environment or self collision).
      If a group name is specified, collision checking is done for that group only (plus descendent links).
      Since the function is non-const, the current state transforms are updated before the collision check. */
  bool isStateColliding(const std::string& group = "", bool verbose = false);

  /** \brief Check if the current state is in collision (with the environment or self collision).  If a group name is
     specified,
      collision checking is done for that group only (plus descendent links). It is expected the current state
     transforms are up to date. */
  bool isStateColliding(const std::string& group = "", bool verbose = false) const
  {
    return isStateColliding(getCurrentState(), group, verbose);
  }

  /** \brief Check if a given state is in collision (with the environment or self collision) If a group name is
     specified,
      collision checking is done for that group only (plus descendent links). The link transforms for \e state are
     updated before the collision check. */
  bool isStateColliding(moveit::core::RobotState& state, const std::string& group = "", bool verbose = false) const
  {
    state.updateCollisionBodyTransforms();
    return isStateColliding(static_cast<const moveit::core::RobotState&>(state), group, verbose);
  }

  /** \brief Check if a given state is in collision (with the environment or self collision)
      If a group name is specified, collision checking is done for that group only (plus descendent links). It is
     expected that the link transforms of \e state are up to date. */
  bool isStateColliding(const moveit::core::RobotState& state, const std::string& group = "",
                        bool verbose = false) const;

  /** \brief Check if a given state is in collision (with the environment or self collision)
      If a group name is specified, collision checking is done for that group only (plus descendent links). */
  bool isStateColliding(const moveit_msgs::msg::RobotState& state, const std::string& group = "",
                        bool verbose = false) const;

  /** \brief Check whether the current state is in collision, and if needed, updates the collision transforms of the
   * current state before the computation. */
  void checkCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res);

  /** \brief Check whether the current state is in collision. The current state is expected to be updated. */
  void checkCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res) const;

  /** \brief Check whether a specified state (\e robot_state) is in collision. This variant of the function takes
      a non-const \e robot_state and calls updateCollisionBodyTransforms() on it. */
  void checkCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                      moveit::core::RobotState& robot_state) const;

  /** \brief Check whether a specified state (\e robot_state) is in collision. The collision transforms of \e
   * robot_state are
   * expected to be up to date. */
  void checkCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                      const moveit::core::RobotState& robot_state) const;

  /** \brief Check whether a specified state (\e robot_state) is in collision, with respect to a given
      allowed collision matrix (\e acm). This variant of the function takes
      a non-const \e robot_state and updates its link transforms if needed. */
  void checkCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                      moveit::core::RobotState& robot_state,
                      const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Check whether a specified state (\e robot_state) is in collision, with respect to a given
      allowed collision matrix (\e acm). */
  void checkCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                      const moveit::core::RobotState& robot_state,
                      const collision_detection::AllowedCollisionMatrix& acm, bool validate_transforms = true) const;

  /** \brief Check whether the current state is in collision,
      but use a collision_detection::CollisionRobot instance that has no padding.
      Since the function is non-const, the current state transforms are also updated if needed. */
  [[deprecated("Use new CollisionRequest flags to enable/ disable padding instead.")]] void
  checkCollisionUnpadded(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res);

  /** \brief Check whether the current state is in collision,
      but use a collision_detection::CollisionRobot instance that has no padding.  */
  [[deprecated("Use new CollisionRequest flags to enable/ disable padding instead.")]] void
  checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                         collision_detection::CollisionResult& res) const;

  /** \brief Check whether a specified state (\e robot_state) is in collision,
      but use a collision_detection::CollisionRobot instance that has no padding.  */
  [[deprecated("Use new CollisionRequest flags to enable/ disable padding instead.")]] void
  checkCollisionUnpadded(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                         const moveit::core::RobotState& robot_state) const;

  /** \brief Check whether a specified state (\e robot_state) is in collision,
      but use a collision_detection::CollisionRobot instance that has no padding.
      Update the link transforms of \e robot_state if needed. */
  [[deprecated("Use new CollisionRequest flags to enable/ disable padding instead.")]] void
  checkCollisionUnpadded(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                         moveit::core::RobotState& robot_state) const;

  /** \brief Check whether a specified state (\e robot_state) is in collision, with respect to a given
      allowed collision matrix (\e acm), but use a collision_detection::CollisionRobot instance that has no padding.
      This variant of the function takes a non-const \e robot_state and calls updates the link transforms if needed. */
  [[deprecated("Use new CollisionRequest flags to enable/ disable padding instead.")]] void
  checkCollisionUnpadded(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                         moveit::core::RobotState& robot_state,
                         const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Check whether a specified state (\e robot_state) is in collision, with respect to a given
      allowed collision matrix (\e acm), but use a collision_detection::CollisionRobot instance that has no padding.  */
  [[deprecated("Use new CollisionRequest flags to enable/ disable padding instead.")]] void
  checkCollisionUnpadded(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                         const moveit::core::RobotState& robot_state,
                         const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Check whether the current state is in self collision */
  void checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res);

  /** \brief Check whether the current state is in self collision */
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult& res) const;

  /** \brief Check whether a specified state (\e robot_state) is in self collision */
  void checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                          moveit::core::RobotState& robot_state) const;

  /** \brief Check whether a specified state (\e robot_state) is in self collision */
  void checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& robot_state) const;

  /** \brief Check whether a specified state (\e robot_state) is in self collision, with respect to a given
      allowed collision matrix (\e acm). The link transforms of \e robot_state are updated if needed. */
  void checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                          moveit::core::RobotState& robot_state,
                          const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Check whether a specified state (\e robot_state) is in self collision, with respect to a given
      allowed collision matrix (\e acm) */
  void checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& robot_state,
                          const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Get the names of the links that are involved in collisions for the current state */
  void getCollidingLinks(std::vector<std::string>& links);

  /** \brief Get the names of the links that are involved in collisions for the current state */
  void getCollidingLinks(std::vector<std::string>& links) const
  {
    getCollidingLinks(links, getCurrentState(), getAllowedCollisionMatrix());
  }

  /** \brief Get the names of the links that are involved in collisions for the state \e robot_state.
      Update the link transforms for \e robot_state if needed. */
  void getCollidingLinks(std::vector<std::string>& links, moveit::core::RobotState& robot_state) const
  {
    robot_state.updateCollisionBodyTransforms();
    getCollidingLinks(links, static_cast<const moveit::core::RobotState&>(robot_state), getAllowedCollisionMatrix());
  }

  /** \brief Get the names of the links that are involved in collisions for the state \e robot_state */
  void getCollidingLinks(std::vector<std::string>& links, const moveit::core::RobotState& robot_state) const
  {
    getCollidingLinks(links, robot_state, getAllowedCollisionMatrix());
  }

  /** \brief  Get the names of the links that are involved in collisions for the state \e robot_state given the
      allowed collision matrix (\e acm) */
  void getCollidingLinks(std::vector<std::string>& links, moveit::core::RobotState& robot_state,
                         const collision_detection::AllowedCollisionMatrix& acm) const
  {
    robot_state.updateCollisionBodyTransforms();
    getCollidingLinks(links, static_cast<const moveit::core::RobotState&>(robot_state), acm);
  }

  /** \brief  Get the names of the links that are involved in collisions for the state \e robot_state given the
      allowed collision matrix (\e acm) */
  void getCollidingLinks(std::vector<std::string>& links, const moveit::core::RobotState& robot_state,
                         const collision_detection::AllowedCollisionMatrix& acm) const;

  /** \brief Get the names of the links that are involved in collisions for the current state.
      Update the link transforms for the current state if needed. */
  void getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts);

  /** \brief Get the names of the links that are involved in collisions for the current state */
  void getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts) const
  {
    getCollidingPairs(contacts, getCurrentState(), getAllowedCollisionMatrix());
  }

  /** \brief Get the names of the links that are involved in collisions for the state \e robot_state.
   *  Can be restricted to links part of or updated by \e group_name (plus descendent links) */
  void getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts,
                         const moveit::core::RobotState& robot_state, const std::string& group_name = "") const
  {
    getCollidingPairs(contacts, robot_state, getAllowedCollisionMatrix(), group_name);
  }

  /** \brief Get the names of the links that are involved in collisions for the state \e robot_state.
      Update the link transforms for \e robot_state if needed.
      Can be restricted to links part of or updated by \e group_name (plus descendent links) */
  void getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts,
                         moveit::core::RobotState& robot_state, const std::string& group_name = "") const
  {
    robot_state.updateCollisionBodyTransforms();
    getCollidingPairs(contacts, static_cast<const moveit::core::RobotState&>(robot_state), getAllowedCollisionMatrix(),
                      group_name);
  }

  /** \brief  Get the names of the links that are involved in collisions for the state \e robot_state given the
      allowed collision matrix (\e acm). Update the link transforms for \e robot_state if needed.
      Can be restricted to links part of or updated by \e group_name (plus descendent links) */
  void getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts,
                         moveit::core::RobotState& robot_state, const collision_detection::AllowedCollisionMatrix& acm,
                         const std::string& group_name = "") const
  {
    robot_state.updateCollisionBodyTransforms();
    getCollidingPairs(contacts, static_cast<const moveit::core::RobotState&>(robot_state), acm, group_name);
  }

  /** \brief  Get the names of the links that are involved in collisions for the state \e robot_state given the
      allowed collision matrix (\e acm). Can be restricted to links part of or updated by \e group_name (plus descendent links) */
  void getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts,
                         const moveit::core::RobotState& robot_state,
                         const collision_detection::AllowedCollisionMatrix& acm,
                         const std::string& group_name = "") const;

  /**@}*/

  /**
   * \name Distance computation
   */
  /**@{*/

  /** \brief The distance between the robot model at state \e robot_state to the nearest collision (ignoring
   * self-collisions)
   */
  double distanceToCollision(moveit::core::RobotState& robot_state) const
  {
    robot_state.updateCollisionBodyTransforms();
    return distanceToCollision(static_cast<const moveit::core::RobotState&>(robot_state));
  }

  /** \brief The distance between the robot model at state \e robot_state to the nearest collision (ignoring
   * self-collisions)
   */
  double distanceToCollision(const moveit::core::RobotState& robot_state) const
  {
    return getCollisionEnv()->distanceRobot(robot_state, getAllowedCollisionMatrix());
  }

  /** \brief The distance between the robot model at state \e robot_state to the nearest collision (ignoring
   * self-collisions), if the robot has no padding */
  double distanceToCollisionUnpadded(moveit::core::RobotState& robot_state) const
  {
    robot_state.updateCollisionBodyTransforms();
    return distanceToCollisionUnpadded(static_cast<const moveit::core::RobotState&>(robot_state));
  }

  /** \brief The distance between the robot model at state \e robot_state to the nearest collision (ignoring
   * self-collisions), if the robot has no padding */
  double distanceToCollisionUnpadded(const moveit::core::RobotState& robot_state) const
  {
    return getCollisionEnvUnpadded()->distanceRobot(robot_state, getAllowedCollisionMatrix());
  }

  /** \brief The distance between the robot model at state \e robot_state to the nearest collision, ignoring
   * self-collisions
   * and elements that are allowed to collide. */
  double distanceToCollision(moveit::core::RobotState& robot_state,
                             const collision_detection::AllowedCollisionMatrix& acm) const
  {
    robot_state.updateCollisionBodyTransforms();
    return distanceToCollision(static_cast<const moveit::core::RobotState&>(robot_state), acm);
  }

  /** \brief The distance between the robot model at state \e robot_state to the nearest collision, ignoring
   * self-collisions
   * and elements that are allowed to collide. */
  double distanceToCollision(const moveit::core::RobotState& robot_state,
                             const collision_detection::AllowedCollisionMatrix& acm) const
  {
    return getCollisionEnv()->distanceRobot(robot_state, acm);
  }

  /** \brief The distance between the robot model at state \e robot_state to the nearest collision, ignoring
   * self-collisions
   * and elements that are allowed to collide, if the robot has no padding. */
  double distanceToCollisionUnpadded(moveit::core::RobotState& robot_state,
                                     const collision_detection::AllowedCollisionMatrix& acm) const
  {
    robot_state.updateCollisionBodyTransforms();
    return distanceToCollisionUnpadded(static_cast<const moveit::core::RobotState&>(robot_state), acm);
  }

  /** \brief The distance between the robot model at state \e robot_state to the nearest collision, ignoring
   * self-collisions
   * and elements that always allowed to collide, if the robot has no padding. */
  double distanceToCollisionUnpadded(const moveit::core::RobotState& robot_state,
                                     const collision_detection::AllowedCollisionMatrix& acm) const
  {
    return getCollisionEnvUnpadded()->distanceRobot(robot_state, acm);
  }

  /**@}*/

  /** \brief Save the geometry of the planning scene to a stream, as plain text */
  void saveGeometryToStream(std::ostream& out) const;

  /** \brief Load the geometry of the planning scene from a stream */
  bool loadGeometryFromStream(std::istream& in);

  /** \brief Load the geometry of the planning scene from a stream at a certain location using offset*/
  bool loadGeometryFromStream(std::istream& in, const Eigen::Isometry3d& offset);

  /** \brief Fill the message \e scene with the differences between this instance of PlanningScene with respect to the
     parent.
      If there is no parent, everything is considered to be a diff and the function behaves like getPlanningSceneMsg()
     */
  void getPlanningSceneDiffMsg(moveit_msgs::msg::PlanningScene& scene) const;

  /** \brief Construct a message (\e scene) with all the necessary data so that the scene can be later reconstructed to
     be
      exactly the same using setPlanningSceneMsg() */
  void getPlanningSceneMsg(moveit_msgs::msg::PlanningScene& scene) const;

  /** \brief Construct a message (\e scene) with the data requested in \e comp. If all options in \e comp are filled,
      this will be a complete planning scene message */
  void getPlanningSceneMsg(moveit_msgs::msg::PlanningScene& scene,
                           const moveit_msgs::msg::PlanningSceneComponents& comp) const;

  /** \brief Construct a message (\e collision_object) with the collision object data from the planning_scene for the
   * requested object*/
  bool getCollisionObjectMsg(moveit_msgs::msg::CollisionObject& collision_obj, const std::string& ns) const;

  /** \brief Construct a vector of messages (\e collision_objects) with the collision object data for all objects in
   * planning_scene */
  void getCollisionObjectMsgs(std::vector<moveit_msgs::msg::CollisionObject>& collision_objs) const;

  /** \brief Construct a message (\e attached_collision_object) with the attached collision object data from the
   * planning_scene for the requested object*/
  bool getAttachedCollisionObjectMsg(moveit_msgs::msg::AttachedCollisionObject& attached_collision_obj,
                                     const std::string& ns) const;

  /** \brief Construct a vector of messages (\e attached_collision_objects) with the attached collision object data for
   * all objects in planning_scene */
  void
  getAttachedCollisionObjectMsgs(std::vector<moveit_msgs::msg::AttachedCollisionObject>& attached_collision_objs) const;

  /** \brief Construct a message (\e octomap) with the octomap data from the planning_scene */
  bool getOctomapMsg(octomap_msgs::msg::OctomapWithPose& octomap) const;

  /** \brief Construct a vector of messages (\e object_colors) with the colors of the objects from the planning_scene */
  void getObjectColorMsgs(std::vector<moveit_msgs::msg::ObjectColor>& object_colors) const;

  /** \brief Apply changes to this planning scene as diffs, even if the message itself is not marked as being a diff
     (is_diff
      member). A parent is not required to exist. However, the existing data in the planning instance is not cleared.
     Data from
      the message is only appended (and in cases such as e.g., the robot state, is overwritten). */
  bool setPlanningSceneDiffMsg(const moveit_msgs::msg::PlanningScene& scene);

  /** \brief Set this instance of a planning scene to be the same as the one serialized in the \e scene message, even if
   * the message itself is marked as being a diff (is_diff member) */
  bool setPlanningSceneMsg(const moveit_msgs::msg::PlanningScene& scene);

  /** \brief Call setPlanningSceneMsg() or setPlanningSceneDiffMsg() depending on how the is_diff member of the message
   * is set */
  bool usePlanningSceneMsg(const moveit_msgs::msg::PlanningScene& scene);

  /** \brief Takes the object message and returns the object pose, shapes and shape poses.
   * If the object pose is empty (identity) but the shape pose is set, this uses the shape
   * pose as the object pose. The shape pose becomes the identity instead.
   */
  bool shapesAndPosesFromCollisionObjectMessage(const moveit_msgs::msg::CollisionObject& object,
                                                Eigen::Isometry3d& object_pose_in_header_frame,
                                                std::vector<shapes::ShapeConstPtr>& shapes,
                                                EigenSTL::vector_Isometry3d& shape_poses);

  bool processCollisionObjectMsg(const moveit_msgs::msg::CollisionObject& object);
  bool processAttachedCollisionObjectMsg(const moveit_msgs::msg::AttachedCollisionObject& object);

  bool processPlanningSceneWorldMsg(const moveit_msgs::msg::PlanningSceneWorld& world);

  void processOctomapMsg(const octomap_msgs::msg::OctomapWithPose& map);
  void processOctomapMsg(const octomap_msgs::msg::Octomap& map);
  void processOctomapPtr(const std::shared_ptr<const octomap::OcTree>& octree, const Eigen::Isometry3d& t);

  /**
   * \brief Clear all collision objects in planning scene
   */
  void removeAllCollisionObjects();

  /** \brief Set the current robot state to be \e state. If not
      all joint values are specified, the previously maintained
      joint values are kept. */
  void setCurrentState(const moveit_msgs::msg::RobotState& state);

  /** \brief Set the current robot state */
  void setCurrentState(const moveit::core::RobotState& state);

  /** \brief Set the callback to be triggered when changes are made to the current scene state */
  void setAttachedBodyUpdateCallback(const moveit::core::AttachedBodyCallback& callback);

  /** \brief Set the callback to be triggered when changes are made to the current scene world */
  void setCollisionObjectUpdateCallback(const collision_detection::World::ObserverCallbackFn& callback);

  bool hasObjectColor(const std::string& id) const;

  /**
   * \brief Gets the current color of an object.
   * \param id The string id of the target object.
   * \return The current object color.
   */
  const std_msgs::msg::ColorRGBA& getObjectColor(const std::string& id) const;

  /**
   * \brief Tries to get the original color of an object, if one has been set before.
   * \param id The string id of the target object.
   * \return The original object color, if available, otherwise std::nullopt.
   */
  std::optional<std_msgs::msg::ColorRGBA> getOriginalObjectColor(const std::string& id) const;

  void setObjectColor(const std::string& id, const std_msgs::msg::ColorRGBA& color);
  void removeObjectColor(const std::string& id);
  void getKnownObjectColors(ObjectColorMap& kc) const;

  bool hasObjectType(const std::string& id) const;

  const object_recognition_msgs::msg::ObjectType& getObjectType(const std::string& id) const;
  void setObjectType(const std::string& id, const object_recognition_msgs::msg::ObjectType& type);
  void removeObjectType(const std::string& id);
  void getKnownObjectTypes(ObjectTypeMap& kc) const;

  /** \brief Clear the diffs accumulated for this planning scene, with respect to:
   * the parent PlanningScene (if it exists)
   * the parent CollisionDetector (if it exists)
   * This function is a no-op if there is no parent planning scene. */
  void clearDiffs();

  /** \brief If there is a parent specified for this scene, then the diffs with respect to that parent are applied to a
     specified planning scene, whatever
      that scene may be. If there is no parent specified, this function is a no-op. */
  void pushDiffs(const PlanningScenePtr& scene);

  /** \brief Make sure that all the data maintained in this
      scene is local. All unmodified data is copied from the
      parent and the pointer to the parent is discarded. */
  void decoupleParent();

  /** \brief Specify a predicate that decides whether states are considered valid or invalid for reasons beyond ones
     covered by collision checking and constraint evaluation.
      This is useful for setting up problem specific constraints (e.g., stability) */
  void setStateFeasibilityPredicate(const StateFeasibilityFn& fn)
  {
    state_feasibility_ = fn;
  }

  /** \brief Get the predicate that decides whether states are considered valid or invalid for reasons beyond ones
   * covered by collision checking and constraint evaluation. */
  const StateFeasibilityFn& getStateFeasibilityPredicate() const
  {
    return state_feasibility_;
  }

  /** \brief Specify a predicate that decides whether motion segments are considered valid or invalid for reasons beyond
   * ones covered by collision checking and constraint evaluation.  */
  void setMotionFeasibilityPredicate(const MotionFeasibilityFn& fn)
  {
    motion_feasibility_ = fn;
  }

  /** \brief Get the predicate that decides whether motion segments are considered valid or invalid for reasons beyond
   * ones covered by collision checking and constraint evaluation. */
  const MotionFeasibilityFn& getMotionFeasibilityPredicate() const
  {
    return motion_feasibility_;
  }

  /** \brief Check if a given state is feasible, in accordance to the feasibility predicate specified by
   * setStateFeasibilityPredicate(). Returns true if no feasibility predicate was specified. */
  bool isStateFeasible(const moveit_msgs::msg::RobotState& state, bool verbose = false) const;

  /** \brief Check if a given state is feasible, in accordance to the feasibility predicate specified by
   * setStateFeasibilityPredicate(). Returns true if no feasibility predicate was specified. */
  bool isStateFeasible(const moveit::core::RobotState& state, bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const moveit_msgs::msg::RobotState& state, const moveit_msgs::msg::Constraints& constr,
                          bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const moveit::core::RobotState& state, const moveit_msgs::msg::Constraints& constr,
                          bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const moveit_msgs::msg::RobotState& state,
                          const kinematic_constraints::KinematicConstraintSet& constr, bool verbose = false) const;

  /** \brief Check if a given state satisfies a set of constraints */
  bool isStateConstrained(const moveit::core::RobotState& state,
                          const kinematic_constraints::KinematicConstraintSet& constr, bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions and feasibility. Includes descendent
   * links of \e group. */
  bool isStateValid(const moveit_msgs::msg::RobotState& state, const std::string& group = "",
                    bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions and feasibility. Includes descendent
   * links of \e group. */
  bool isStateValid(const moveit::core::RobotState& state, const std::string& group = "", bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions, feasibility  and whether the user
   * specified validity conditions hold as well. Includes descendent links of \e group. */
  bool isStateValid(const moveit_msgs::msg::RobotState& state, const moveit_msgs::msg::Constraints& constr,
                    const std::string& group = "", bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions, feasibility  and whether the user
   * specified validity conditions hold as well. Includes descendent links of \e group. */
  bool isStateValid(const moveit::core::RobotState& state, const moveit_msgs::msg::Constraints& constr,
                    const std::string& group = "", bool verbose = false) const;

  /** \brief Check if a given state is valid. This means checking for collisions, feasibility  and whether the user
   * specified validity conditions hold as well. Includes descendent links of \e group. */
  bool isStateValid(const moveit::core::RobotState& state, const kinematic_constraints::KinematicConstraintSet& constr,
                    const std::string& group = "", bool verbose = false) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance and feasibility).
   * Includes descendent links of \e group. */
  bool isPathValid(const moveit_msgs::msg::RobotState& start_state, const moveit_msgs::msg::RobotTrajectory& trajectory,
                   const std::string& group = "", bool verbose = false,
                   std::vector<std::size_t>* invalid_index = nullptr) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and
   * constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the
   * passed in trajectory.  Includes descendent links of \e group. */
  bool isPathValid(const moveit_msgs::msg::RobotState& start_state, const moveit_msgs::msg::RobotTrajectory& trajectory,
                   const moveit_msgs::msg::Constraints& path_constraints, const std::string& group = "",
                   bool verbose = false, std::vector<std::size_t>* invalid_index = nullptr) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and
   * constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the
   * passed in trajectory. Includes descendent links of \e group. */
  bool isPathValid(const moveit_msgs::msg::RobotState& start_state, const moveit_msgs::msg::RobotTrajectory& trajectory,
                   const moveit_msgs::msg::Constraints& path_constraints,
                   const moveit_msgs::msg::Constraints& goal_constraints, const std::string& group = "",
                   bool verbose = false, std::vector<std::size_t>* invalid_index = nullptr) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and
   * constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the
   * passed in trajectory. Includes descendent links of \e group. */
  bool isPathValid(const moveit_msgs::msg::RobotState& start_state, const moveit_msgs::msg::RobotTrajectory& trajectory,
                   const moveit_msgs::msg::Constraints& path_constraints,
                   const std::vector<moveit_msgs::msg::Constraints>& goal_constraints, const std::string& group = "",
                   bool verbose = false, std::vector<std::size_t>* invalid_index = nullptr) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and
   * constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the
   * passed in trajectory. Includes descendent links of \e group. */
  bool isPathValid(const robot_trajectory::RobotTrajectory& trajectory,
                   const moveit_msgs::msg::Constraints& path_constraints,
                   const std::vector<moveit_msgs::msg::Constraints>& goal_constraints, const std::string& group = "",
                   bool verbose = false, std::vector<std::size_t>* invalid_index = nullptr) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and
   * constraint satisfaction). It is also checked that the goal constraints are satisfied by the last state on the
   * passed in trajectory. Includes descendent links of \e group. */
  bool isPathValid(const robot_trajectory::RobotTrajectory& trajectory,
                   const moveit_msgs::msg::Constraints& path_constraints,
                   const moveit_msgs::msg::Constraints& goal_constraints, const std::string& group = "",
                   bool verbose = false, std::vector<std::size_t>* invalid_index = nullptr) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance, feasibility and
   * constraint satisfaction). Includes descendent links of \e group. */
  bool isPathValid(const robot_trajectory::RobotTrajectory& trajectory,
                   const moveit_msgs::msg::Constraints& path_constraints, const std::string& group = "",
                   bool verbose = false, std::vector<std::size_t>* invalid_index = nullptr) const;

  /** \brief Check if a given path is valid. Each state is checked for validity (collision avoidance and feasibility).
   * Includes descendent links of \e group. */
  bool isPathValid(const robot_trajectory::RobotTrajectory& trajectory, const std::string& group = "",
                   bool verbose = false, std::vector<std::size_t>* invalid_index = nullptr) const;

  /** \brief Get the top \e max_costs cost sources for a specified trajectory. The resulting costs are stored in \e
   * costs */
  void getCostSources(const robot_trajectory::RobotTrajectory& trajectory, std::size_t max_costs,
                      std::set<collision_detection::CostSource>& costs, double overlap_fraction = 0.9) const;

  /** \brief Get the top \e max_costs cost sources for a specified trajectory, but only for group \e group_name (plus
   * descendent links). The resulting costs are stored in \e costs */
  void getCostSources(const robot_trajectory::RobotTrajectory& trajectory, std::size_t max_costs,
                      const std::string& group_name, std::set<collision_detection::CostSource>& costs,
                      double overlap_fraction = 0.9) const;

  /** \brief Get the top \e max_costs cost sources for a specified state. The resulting costs are stored in \e costs */
  void getCostSources(const moveit::core::RobotState& state, std::size_t max_costs,
                      std::set<collision_detection::CostSource>& costs) const;

  /** \brief Get the top \e max_costs cost sources for a specified state, but only for group \e group_name (plus
   * descendent links). The resulting costs are stored in \e costs */
  void getCostSources(const moveit::core::RobotState& state, std::size_t max_costs, const std::string& group_name,
                      std::set<collision_detection::CostSource>& costs) const;

  /** \brief Outputs debug information about the planning scene contents */
  void printKnownObjects(std::ostream& out = std::cout) const;

  /** \brief Clone a planning scene. Even if the scene \e scene depends on a parent, the cloned scene will not. */
  static PlanningScenePtr clone(const PlanningSceneConstPtr& scene);

  /** \brief Allocate a new collision detector and replace the previous one if there was any.
   *
   * The collision detector type is specified with (a shared pointer to) an
   * allocator which is a subclass of CollisionDetectorAllocator.  This
   * identifies a combination of CollisionWorld/CollisionRobot which can be
   * used together.
   *
   * A new PlanningScene uses an FCL collision detector by default.
   *
   * example: to add FCL collision detection (normally not necessary) call
   *   planning_scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
   *
   * */
  void allocateCollisionDetector(const collision_detection::CollisionDetectorAllocatorPtr& allocator)
  {
    allocateCollisionDetector(allocator, nullptr /* no parent_detector */);
  }

private:
  /* Private constructor used by the diff() methods. */
  PlanningScene(const PlanningSceneConstPtr& parent);

  /* Initialize the scene.  This should only be called by the constructors.
   * Requires a valid robot_model_ */
  void initialize();

  /* Helper functions for processing collision objects */
  bool processCollisionObjectAdd(const moveit_msgs::msg::CollisionObject& object);
  bool processCollisionObjectRemove(const moveit_msgs::msg::CollisionObject& object);
  bool processCollisionObjectMove(const moveit_msgs::msg::CollisionObject& object);

  MOVEIT_STRUCT_FORWARD(CollisionDetector);

  /* Construct a new CollisionDector from allocator, copy-construct environments from parent_detector if not nullptr */
  void allocateCollisionDetector(const collision_detection::CollisionDetectorAllocatorPtr& allocator,
                                 const CollisionDetectorPtr& parent_detector);

  /* \brief A set of compatible collision detectors */
  struct CollisionDetector
  {
    collision_detection::CollisionDetectorAllocatorPtr alloc_;
    collision_detection::CollisionEnvPtr cenv_;  // never nullptr
    collision_detection::CollisionEnvConstPtr cenv_const_;

    collision_detection::CollisionEnvPtr cenv_unpadded_;
    collision_detection::CollisionEnvConstPtr cenv_unpadded_const_;

    const collision_detection::CollisionEnvConstPtr& getCollisionEnv() const
    {
      return cenv_const_;
    }
    const collision_detection::CollisionEnvConstPtr& getCollisionEnvUnpadded() const
    {
      return cenv_unpadded_const_;
    }
    void copyPadding(const CollisionDetector& src);
  };
  friend struct CollisionDetector;

  std::string name_;  // may be empty

  PlanningSceneConstPtr parent_;  // Null unless this is a diff scene

  moveit::core::RobotModelConstPtr robot_model_;  // Never null (may point to same model as parent)

  std::optional<moveit::core::RobotState> robot_state_;  // if there is no value use parent's

  // Called when changes are made to attached bodies
  moveit::core::AttachedBodyCallback current_state_attached_body_callback_;

  // This variable is not necessarily used by child planning scenes
  // This Transforms class is actually a SceneTransforms class
  std::optional<moveit::core::TransformsPtr> scene_transforms_;  // if there is no value use parent's

  collision_detection::WorldPtr world_;             // never nullptr, never shared with parent/child
  collision_detection::WorldConstPtr world_const_;  // copy of world_
  collision_detection::WorldDiffPtr world_diff_;    // nullptr unless this is a diff scene
  collision_detection::World::ObserverCallbackFn current_world_object_update_callback_;
  collision_detection::World::ObserverHandle current_world_object_update_observer_handle_;

  CollisionDetectorPtr collision_detector_;  // Never nullptr.

  std::optional<collision_detection::AllowedCollisionMatrix> acm_;  // if there is no value use parent's

  StateFeasibilityFn state_feasibility_;
  MotionFeasibilityFn motion_feasibility_;

  // Maps of current and original object colors (to manage attaching/detaching objects)
  std::unique_ptr<ObjectColorMap> object_colors_;
  std::unique_ptr<ObjectColorMap> original_object_colors_;

  // a map of object types
  std::optional<ObjectTypeMap> object_types_;
};
}  // namespace planning_scene
