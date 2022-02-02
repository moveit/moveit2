# Migration Strategery

This file should be deleted once all tasks are complete. Also delete `SETUP_STEP.{c|h}pp`

## Steps to Migrate
 * `moveit_setup_core_plugins`
   * [x] Start Screen
   * [x] Author Information
   * [x] Configuration Files
 * `moveit_setup_srdf_plugins`
   * [x] Self-Collisions
   * [x] Virtual Joints
   * [ ] Robot Poses
   * [ ] End Effectors
   * [ ] Passive Joints
   * [x] Planning Groups
     * BUG: Creating a new group does not properly create a new JointModelGroup
     * TODO: Need method for getting a list of OMPL Planners
 * `moveit_setup_app_plugins`
   * [ ] ROS Control
   * [ ] Simulation
   * [x] Perception

### Instructions for migrating steps
Let `SETUP_STEP` be the name of the step you are migrating. You should already have files named `SETUP_STEP_widget.hpp` and `SETUP_STEP_widget.cpp`.
 1. Create new files:
  * Copy `SETUP_STEP.hpp` to `include/PACKAGE_NAME/SETUP_STEP.hpp`
  * Copy `SETUP_STEP.cpp` to `src/SETUP_STEP.cpp`
 1. Find/Replace the following variables in both files
  * `YOUR_NAME`
  * `SETUP_STEP`
  * `PACKAGE_NAME`
 1. In the appropriate package, edit the CMake to
  * add the file `include/PACKAGE_NAME/X_widget.hpp` to the call to `qt5_wrap_cpp`
  * add the file `X_widget.cpp` and `X.cpp` to the call to `add_library`
 1. In `X_widget.hpp`
  * Add these headers
     * `#include <moveit_setup_framework/qt/setup_step_widget.hpp>`
     * `#include <PACKAGE_NAME/SETUP_STEP.hpp>`
  * Change the namespace to `PACKAGE_NAME`
  * Change the class declaration to extend `public moveit_setup_framework::SetupStepWidget`
  * Add public method
  ```
      moveit_setup_framework::SetupStep& getSetupStep() override
      {
          return setup_step_;
      }
```
  * Replace the constructor with `void onInit() override;`
  * Add a protected variable `SETUP_STEP setup_step_;`
 1. In `X_widget.cpp`
  * Replace `#include "SETUP_STEP_widget.h"` with `#include <PACKAGE_NAME/SETUP_STEP_widget.hpp>`
  * Replace `#include "header_widget.h"` with `#include <moveit_setup_framework/qt/helper_widgets.hpp>`
  * Change the namespace to `PACKAGE_NAME`
  * Replace the constructor with `void SETUP_STEPWidget::onInit()`
  * Replace `HeaderWidget* header = new HeaderWidget` with `auto header = new moveit_setup_framework::HeaderWidget`
  * Replace `Q_EMIT isModal` with `Q_EMIT setModalMode`
  * Replace `Q_EMIT highlightLink(` with `rviz_panel_->highlightLink(`
  * Replace `Q_EMIT highlightGroup(` with `rviz_panel_->highlightGroup(`
  * Replace `Q_EMIT unhighlightAll();` with `rviz_panel_->unhighlightAll();`
  * Upgrade the ROS debugging to ROS 2
     * Replace `ROS_([A-Z_]+)\(` with `RCLCPP_$1(setup_step_.getLogger(), `
  * At the very bottom of the file (outside the namespace declaration) add the following block
  ```
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(PACKAGE_NAME::SETUP_STEPWidget, moveit_setup_framework::SetupStepWidget)
  ```
 1. Add to `moveit_setup_framework_plugins.xml`
 ```
    <class type="PACKAGE_NAME::SETUP_STEPWidget" base_class_type="moveit_setup_framework::SetupStepWidget">
        <description>A_DESCRIPTION</description>
    </class>
  ```
 1. Move "Business Logic" out of `SETUP_STEP_widget.xpp` This is a more involved process, but a good place to start is to find all references to `config_data_` and move to `SETUP_STEP.xpp`
 1. Add to the list of widgets in `moveit_setup_assistant/src/setup_assistant_widget.cpp`

### Instructions for Adding Configs/Generated Files
 * Lots of additional useful code can be found in the `old_planner` folder.
 * Additional code removed from other classes can be found in `extra_*.txt` in this folder.

## Possible Enhancements
 * There's a common pattern in `GeneratedFile` subclasses which passes a reference to the parent object. This could be templated for reducing code duplication.
