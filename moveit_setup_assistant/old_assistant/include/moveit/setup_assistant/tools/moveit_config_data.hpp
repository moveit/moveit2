/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Dave Coleman */

#pragma once

#include <moveit/setup_assistant/tools/compute_default_collisions.h>  // for LinkPairMap
#include <yaml-cpp/yaml.h>                                            // outputting yaml config files

#include <utility>

namespace moveit_setup_assistant
{
// Default kin solver values
static const double DEFAULT_KIN_SOLVER_SEARCH_RESOLUTION = 0.005;
static const double DEFAULT_KIN_SOLVER_TIMEOUT = 0.005;

// ******************************************************************************************
// Structs
// ******************************************************************************************

/**
 * Planning groups extra data not found in srdf but used in config files
 */
struct GroupMetaData
{
  std::string kinematics_solver_;               // Name of kinematics plugin to use
  double kinematics_solver_search_resolution_;  // resolution to use with solver
  double kinematics_solver_timeout_;            // solver timeout
  std::string kinematics_parameters_file_;      // file for additional kinematics parameters
  std::string default_planner_;                 // Name of the default planner to use
};

/**
 * Controllers settings which may be set in the config files
 */
struct ControllerConfig
{
  std::string name_;                 // controller name
  std::string type_;                 // controller type
  std::vector<std::string> joints_;  // joints controller by this controller
};

/**
 * Planning parameters which may be set in the config files
 */
struct OmplPlanningParameter
{
  std::string name;     // name of parameter
  std::string value;    // value parameter will receive (but as a string)
  std::string comment;  // comment briefly describing what this parameter does
};

/** \brief This class describes the OMPL planners by name, type, and parameter list, used to create the
 * ompl_planning.yaml file */
class OMPLPlannerDescription
{
public:
  /** \brief Constructor
   *  @param name: name of planner
   *  @parameter type: type of planner
   */
  OMPLPlannerDescription(const std::string& name, const std::string& type)
  {
    name_ = name;
    type_ = type;
  };
  /** \brief Destructor */
  ~OMPLPlannerDescription()
  {
    parameter_list_.clear();
  };
  /** \brief adds a parameter to the planner description
   * @param name: name of parameter to add
   * @parameter: value: value of parameter as a string
   *  @parameter: value: value of parameter as a string
   */
  void addParameter(const std::string& name, const std::string& value = "", const std::string& comment = "")
  {
    OmplPlanningParameter temp;
    temp.name = name;
    temp.value = value;
    temp.comment = comment;
    parameter_list_.push_back(temp);
  }
  std::vector<OmplPlanningParameter> parameter_list_;
  std::string name_;  // name of planner
  std::string type_;  // type of planner (geometric)
};

/** \brief This class is shared with all widgets and contains the common configuration data
    needed for generating each robot's MoveIt configuration package.

    All SRDF data is contained in a subclass of this class -
    srdf_writer.cpp. This class also contains the functions for writing
    out the configuration files. */
class MoveItConfigData
{
public:
  MoveItConfigData();
  ~MoveItConfigData();

  // bits of information that can be entered in Setup Assistant
  enum InformationFields
  {
    COLLISIONS = 1 << 1,
    VIRTUAL_JOINTS = 1 << 2,
    GROUPS = 1 << 3,
    GROUP_CONTENTS = 1 << 4,
    GROUP_KINEMATICS = 1 << 5,
    POSES = 1 << 6,
    END_EFFECTORS = 1 << 7,
    PASSIVE_JOINTS = 1 << 8,
    SIMULATION = 1 << 9,
    AUTHOR_INFO = 1 << 10,
    SENSORS_CONFIG = 1 << 11,
    SRDF = COLLISIONS | VIRTUAL_JOINTS | GROUPS | GROUP_CONTENTS | POSES | END_EFFECTORS | PASSIVE_JOINTS
  };
  unsigned long changes;  // bitfield of changes (composed of InformationFields)

  /// Planning groups extra data not found in srdf but used in config files
  std::map<std::string, GroupMetaData> group_meta_data_;

  /// Setup Assistants package's path for when we use its templates
  std::string setup_assistant_path_;

  /// Location that moveit_setup_assistant stores its templates
  std::string template_package_path_;

  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /**
   * Find the associated group by name
   *
   * @param name - name of data to find in datastructure
   * @return pointer to data in datastructure
   */
  srdf::Model::Group* findGroupByName(const std::string& name);

  // ******************************************************************************************
  // Public Functions for outputting configuration and setting files
  // ******************************************************************************************
  std::vector<OMPLPlannerDescription> getOMPLPlanners() const;
  std::map<std::string, double> getInitialJoints() const;
  bool outputSetupAssistantFile(const std::string& file_path);
  bool outputGazeboURDFFile(const std::string& file_path);
  bool outputOMPLPlanningYAML(const std::string& file_path);
  bool outputSTOMPPlanningYAML(const std::string& file_path);
  bool outputKinematicsYAML(const std::string& file_path);
  bool outputJointLimitsYAML(const std::string& file_path);
  bool outputFakeControllersYAML(const std::string& file_path);
  bool outputSimpleControllersYAML(const std::string& file_path);
  bool outputROSControllersYAML(const std::string& file_path);

  /**
   * \brief Helper function to get the controller that is controlling the joint
   * \return controller type
   */
  std::string getJointHardwareInterface(const std::string& joint_name);

  /**
   * \brief Parses the existing urdf and constructs a string from it with the elements required by gazebo simulator
   * added
   * \return gazebo compatible urdf or empty if error encountered
   */
  std::string getGazeboCompatibleURDF();

  /**
   * \brief Decide the best two joints to be used for the projection evaluator
   * \param planning_group name of group to use
   * \return string - value to insert into yaml file
   */
  std::string decideProjectionJoints(const std::string& planning_group);

  /**
   * Input ompl_planning.yaml file for editing its values
   * @param file_path path to ompl_planning.yaml in the input package
   * @return true if the file was read correctly
   */
  bool inputOMPLYAML(const std::string& file_path);

  /**
   * Input kinematics.yaml file for editing its values
   * @param file_path path to kinematics.yaml in the input package
   * @return true if the file was read correctly
   */
  bool inputKinematicsYAML(const std::string& file_path);

  /**
   * Input planning_context.launch for editing its values
   * @param file_path path to planning_context.launch in the input package
   * @return true if the file was read correctly
   */
  bool inputPlanningContextLaunch(const std::string& file_path);

  /**
   * Helper function for parsing ros_controllers.yaml file
   * @param YAML::Node - individual controller to be parsed
   * @return true if the file was read correctly
   */
  bool parseROSController(const YAML::Node& controller);

  /**
   * Helper function for parsing ros_controllers.yaml file
   * @param std::ifstream of ros_controller.yaml
   * @return true if the file was read correctly
   */
  bool processROSControllers(std::ifstream& input_stream);

  /**
   * Input ros_controllers.yaml file for editing its values
   * @param file_path path to ros_controllers.yaml in the input package
   * @return true if the file was read correctly
   */
  bool inputROSControllersYAML(const std::string& file_path);

  /**
   * \brief Add a Follow Joint Trajectory action Controller for each Planning Group
   * \return true if controllers were added to the controller_configs_ data structure
   */
  bool addDefaultControllers();

  /**
   * Helper Function for joining a file path and a file name, or two file paths, etc,
   * in a cross-platform way
   *
   * @param path1 first half of path
   * @param path2 second half of path, or filename
   * @return string resulting combined paths
   */
  std::string appendPaths(const std::string& path1, const std::string& path2);

  /**
   * \brief Adds a controller to controller_configs_ vector
   * \param new_controller a new Controller to add
   * \return true if inserted correctly
   */
  bool addController(const ControllerConfig& new_controller);

  /**
   * \brief Gets controller_configs_ vector
   * \return pointer to controller_configs_
   */
  std::vector<ControllerConfig>& getControllers()
  {
    return controller_configs_;
  }

  /**
   * Find the associated controller by name
   *
   * @param controller_name - name of controller to find in datastructure
   * @return pointer to data in datastructure
   */
  ControllerConfig* findControllerByName(const std::string& controller_name);

  /**
   * Delete controller by name
   *
   * @param controller_name - name of controller to delete
   * @return true if deleted, false if not found
   */
  bool deleteController(const std::string& controller_name);

  /**
   * \brief Helper function to get the default start pose for moveit_sim_hw_interface
   */
  srdf::Model::GroupState getDefaultStartPose();

  /**
   * \brief Custom std::set comparator, used for sorting the joint_limits.yaml file into alphabetical order
   * \param jm1 - a pointer to the first joint model to compare
   * \param jm2 - a pointer to the second joint model to compare
   * \return bool of alphabetical sorting comparison
   */
  struct JointModelCompare
  {
    bool operator()(const moveit::core::JointModel* jm1, const moveit::core::JointModel* jm2) const
    {
      return jm1->getName() < jm2->getName();
    }
  };

private:
  // ******************************************************************************************
  // Private Vars
  // ******************************************************************************************

  /// ROS Controllers config data
  std::vector<ROSControlConfig> ros_controllers_config_;
};

}  // namespace moveit_setup_assistant
