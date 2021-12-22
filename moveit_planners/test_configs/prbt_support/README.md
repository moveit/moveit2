# Overview

The prbt_support package contains files to start the PRBT manipulator. To start the robot run `roslaunch prbt_support robot.launch`.

# ROS API

## SystemInfoNode

Logs important system information.

### Used Services

- /prbt/driver/get_object (canopen_chain_node/GetObject)
  - Read CANOpen object holding firmware information.
