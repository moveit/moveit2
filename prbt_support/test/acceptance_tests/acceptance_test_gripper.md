Copyright © 2018 Pilz GmbH & Co. KG

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

---

# Acceptance Test Gripper

## Prerequisites

## Test Sequence:
  1. Open the model in rviz `roslaunch prbt_support test_urdf.launch gripper:=pg70`.
  2. In Rviz display the robot model (Add->rviz->RobotModel).
  3. Move the sliders of the "joint_state_publisher" window. 
  4. Close everything.
  5. Open the model in rviz `roslaunch prbt_support test_urdf.launch`.
  6. In Rviz display the robot model (Add->rviz->RobotModel).
  7. Close everything.
  8. Open the model in rviz `roslaunch prbt_support test_urdf.launch`.
  9. In Rviz display the robot model (Add->rviz->RobotModel).

## Expected Results:
  1. Everything opens with no errors. A window for a rviz and a window "joint_state_publisher" has appeared.
  2. You see a gripper attached to the end of the robot.
  3. The gripper should move along with the robot in a physical possible manner.
  4. - 
  5. Everything opens with no errors.
  6. You don't see the gripper anymore.
  7. -
  8. Everything opens with no errors.
  9. You don't see the gripper anymore.

---
