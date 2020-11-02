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

# Acceptance Test TCP

## Prerequisites

## Test Sequence:
  1. Open the model in rviz `roslaunch prbt_support test_urdf.launch`.
  2. In Rviz display the tf-frames (Add->rviz->tf).
  3. Close everything.
  4. prbt_support/urdf/prbt.xacro

     Edit the lines
     ```
     <xacro:arg name="tcp_offset_xyz" default="0 0 0.2"/>
     <xacro:arg name="tcp_offset_rpy" default="0 0 0"/>
     ```
     to some desired offset
  5. Open again like before.

## Expected Results:
  1. Everything opens with no errors.
  2. You see a tf-frame named: "[robotname]_tcp".
  3. -
  4. -
  5. Check that the tcp frame now has the desired offset from its preceiding frame.


---
