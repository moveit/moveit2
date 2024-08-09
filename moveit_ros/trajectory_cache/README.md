# Trajectory Cache

```
 *   This cache does NOT support collision detection!
 *   Plans will be put into and fetched from the cache IGNORING collision.
 *   If your planning scene is expected to change between cache lookups, do NOT
 *   use this cache, fetched plans are likely to result in collision then.
 *
 *   To handle collisions this class will need to hash the planning scene world
 *   msg (after zeroing out std_msgs/Header timestamps and sequences) and do an
 *   appropriate lookup, or otherwise find a way to determine that a planning scene
 *   is "less than" or "in range of" another (e.g. checking that every obstacle/mesh
 *   exists within the other scene's). (very out of scope)
 ```

A trajectory cache based on [`warehouse_ros`](https://github.com/moveit/warehouse_ros) for the move_group planning interface that supports fuzzy lookup for `MotionPlanRequest` and `GetCartesianPath` requests and trajectories.

The cache allows you to insert trajectories and fetch keyed fuzzily on the following:

- Starting robot (joint) state
- Planning request constraints
  - This includes ALL joint, position, and orientation constraints!
  - And also workspace parameters, and some others.
- Planning request goal parameters

It works generically for an arbitrary number of joints, across any number of move_groups.

Furthermore, the strictness of the fuzzy lookup can be configured for start and goal conditions.

## Citations

If you use this package in your work, please cite it using the following:

```
@software{ong_2024_11215428,
  author       = {Ong, Brandon},
  title        = {A Fuzzy-Matching Trajectory Cache for MoveIt 2},
  month        = may,
  year         = 2024,
  publisher    = {GitHub},
  version      = {0.1.0},
  doi          = {10.5281/zenodo.11215428},
  url          = {https://doi.org/10.5281/zenodo.11215428}
}
```

## Example usage

```cpp
// Be sure to set some relevant ROS parameters:
// Relevant ROS Parameters:
//   - `warehouse_plugin`: What database to use
auto traj_cache = std::make_shared<TrajectoryCache>(node);
traj_cache->init(/*db_host=*/":memory:", /*db_port=*/0, /*exact_match_precision=*/1e-6);

auto fetched_trajectory =
    traj_cache->fetchBestMatchingTrajectory(*move_group_interface, robot_name, motion_plan_req_msg,
                                            _cache_start_match_tolerance, _cache_goal_match_tolerance,
                                            /*sort_by=*/"execution_time_s");

if (fetched_trajectory)
{
  // Great! We got a cache hit
  // Do something with the plan.
}
else
{
  // Plan... And put it for posterity!
  traj_cache->insertTrajectory(
      *interface, robot_name, std::move(plan_req_msg), std::move(res->result.trajectory),
      rclcpp::Duration(res->result.trajectory.joint_trajectory.points.back().time_from_start).seconds(),
      res->result.planning_time, /*prune_worse_trajectories=*/true);
}
```

It also has the following optimizations:
- Separate caches for separate move groups
- The cache does "canonicalization" of the poses to be relative to the planning frame to increase the chances of cache hits.
- Configurable fuzzy lookup for the keys above.
- It supports "overwriting" of worse trajectories with better trajectories
  - If a trajectory with keys extremely close to a pre-existing cache entry is inserted with a better planned execution time, the cache can delete all "worse" trajectories.
  - #IsThisMachineLearning
  - It also prunes the database and mitigates a lot of query slow-down as the cache grows

## Working Principle

If a plan request has start, goal, and constraint conditions that are "close enough" (configurable per request) to an entry in the cache, then the cached trajectory should be suitable (as long as obstacles have not changed).

Goal fuzziness is a lot less lenient than start fuzziness by default.

Finally, the databases are sharded by move group, and the constraints are added as columns in a name agnostic fashion (they are even somewhat robust to ordering, because they're sorted!)

## Benefits

A trajectory cache helps:
- Cut down on planning time (especially for known moves)
- Allows for consistent predictable behavior of used together with a stochastic planner
  - It effectively allows you to "freeze" a move

These benefits come from the fact that the cache acts as a lookup table of plans that were already made for a given scenario and constraints, allowing the cache to be substituted for a planner call. The reuse of cached plans then allow you to get predictable execution behavior.

A user may also choose when to leverage the cache (e.g. when planning moves from a static "home" position, or repetitive/cartesian moves) to get these benefits.

Additionally, because the cache class has the ability to sort by planned execution time, over sufficient runs, the stochastic plans eventually converge to better and better plans (execution time wise).

You may build abstractions on top of the class, for example, to expose the following behaviors:
- `TrainingOverwrite`: Always plan, and write to cache, deleting all worse trajectories for "matching" cache keys
- `TrainingAppendOnly`: Always plan, and always add to the cache.
- `ExecuteBestEffort`: Rely on cache wherever possible, but plan otherwise.
- `ExecuteReadOnly`: Only execute if cache hit occurred.

You can see how such behaviors effectively model the "dev" and "deploy" phases of a robot deployment, and how they could be useful.

## Best Practices

- Since this cache does not yet support collisions, ensure the planning scene and obstacles remain static
- Have looser start fuzziness, and stricter goal fuzziness
- Move the robot to static known poses where possible before planning to increase the chances of a cache hit
- Use the cache where repetitive, non-dynamic motion is likely to occur (e.g. known plans, short planned moves, etc.)

## WARNING: The following are unsupported / RFE

Since this is an initial release, the following features are unsupported because they were a little too difficult for the time I had to implement this. So I am leaving it to the community to help!

- **!!! This cache does NOT support collision detection!**
  - Trajectories will be put into and fetched from the cache IGNORING collision.
  - If your planning scene is expected to change between cache lookups, do NOT use this cache, fetched trajectories are likely to result in collision then.
  - To handle collisions this class will need to hash the planning scene world msg (after zeroing out std_msgs/Header timestamps and sequences) and do an appropriate lookup, or do more complicated checks to see if the scene world is "close enough" or is a less obstructed version of the scene in the cache entry.
 - !!! This cache does NOT support keying on joint velocities and efforts.
   - The cache only keys on joint positions.
- !!! This cache does NOT support multi-DOF joints.
- !!! This cache does NOT support certain constraints
  - Including: path, constraint regions, everything related to collision.
- The fuzzy lookup can't be configured on a per-joint basis.
- Alternate ordinal lookup metrics for the cache
  - Currently only execution time is explicitly supported as a way to compare cache entries. Ideally we should be able to inject lambdas to save custom cache DB metadata to represent and sort on custom cost functions (e.g. minimum jerk, path length, etc.). (e.g. https://github.com/moveit/moveit2/pull/2153)
