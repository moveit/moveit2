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

# Acceptance Test Joint Limits using the real robot

## Prerequisites
  - Properly connect and startup the robot. Make sure an emergency stop is within reach.
  - Note that the robot will try to move exactly to the limits of each joint. Make sure that collisions are avoided or
    skip the test for certain joints.

## Starting the joint limit acceptance tests
  - Press the enabling switch and run `roslaunch prbt_support robot.launch`.
  - Run `rosrun prbt_support acceptance_test_joint_limits.py`.
  - Please note: Each test must be confirmed before it is executed. Confirm by entering `y` or `n` to skip test.

## Test descriptions
  - The concrete test descriptions consisting of test-summary, test-sequence and expected results can be found in
    `acceptance_test_joint_limits.py`.

---
