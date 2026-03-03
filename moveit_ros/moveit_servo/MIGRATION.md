# Migration Notes

API changes since last Noetic Release

## Rolling command window (Humble backport of PR #2594)

A new parameter `max_expected_latency` has been added to the servo configuration.
When `command_out_type` is `trajectory_msgs/JointTrajectory`, Servo now publishes multi-point rolling-window trajectories for smoother motion.

Add to your servo YAML:
```yaml
max_expected_latency: 0.1  # seconds
publish_period: 0.01        # reduced from 0.034 for a denser rolling window
```

`std_msgs/Float64MultiArray` output is unchanged.

## Noetic → ROS 2

- Servo::getLatestJointState was removed.  To get the latest joint positions of the group servo is working with use the CSM in the PSM.  Here is an example of how to get the latest joint positions:
        planning_scene_monitor_->getStateMonitor()->getCurrentState()->copyJointGroupPositions(move_group_name, positions);
