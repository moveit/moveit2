# planar_kinematics_plugin

Description: MoveIt 2 inverse kinematics solver for mobile base and chain of 1-DOF joints.

### Setup

To setup the kinematics plugin we need a joint model group that consist of two joint model groups: one for the manipulator and the other one for the mobile base which has only one joint with planar type so the `*.srdf` file should have

```xml
<group name="manipulator">
  ...
</group>
<group name="mobile_base">
  <joint name="planar_joint" /> <!-- planar_joint must have planar type -->
</group>
<group name="mobile_base_manipulator">
  <group name="manipulator" />
  <group name="mobile_base" />
</group>
```

and in the `kinematics.yaml` we should have

```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
mobile_base_manipulator:
  kinematics_solver: planar_kinematics_plugin/PlanarKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.1
```
