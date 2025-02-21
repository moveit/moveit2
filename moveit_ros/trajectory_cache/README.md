# Fuzzy-Matching Trajectory Cache

A trajectory cache based on [`warehouse_ros`](https://github.com/moveit/warehouse_ros) for the move_group planning interface that supports fuzzy lookup for `MotionPlanRequest` and `GetCartesianPath` requests and trajectories.

The cache will work on manipulators with an arbitrary number of joints, across any number of move groups.
Furthermore, the cache supports pruning and ranking of fetched trajectories, with extension points for injecting your own feature keying, cache insert, cache prune and cache sorting logic.

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

## WARNING: The following are unsupported / RFE

Since this is an initial release, the following features are unsupported because they were a little too difficult for the time I had to implement this. So I am leaving it to the community to help!

- **!!! This cache does NOT support collision detection, multi-DOF joints, or constraint regions!**
  - Trajectories will be put into and fetched from the cache IGNORING collision. If your planning scene is expected to change significantly between cache lookups, it is likely that the fetched plan will result in collisions.
  - To natively handle collisions this cache will need to hash the planning scene world msg (after zeroing out std_msgs/Header timestamps and sequences) and do an appropriate lookup, or do more complicated checks to see if the scene world is "close enough" or is a strictly less obstructed version of the scene in the cache entry.
- The fuzzy lookup can't be configured on a per-joint basis.

That said, there are ways to get around the lack of native collision support to enable use of this cache, such as:
- Validating a fetched plan for collisions before execution.
- Make use of the hybrid planning pipeline, using local planners for collision avoidance, while keeping the cache as a stand-in for a "global planner", where applicable.

## Example Usage

**PRE-REQUISITE**: The `warehouse_plugin` ROS parameter must be set to a [`warehouse_ros`](https://github.com/moveit/warehouse_ros) plugin you have installed, which determines what database backend should be used for the cache.

```cpp
auto cache = std::make_shared<TrajectoryCache>(node);
cache->init(/*db_host=*/":memory:", /*db_port=*/0, /*exact_match_precision=*/1e-6);

// The default feature extractors key the cache on starting robot state and goal constraints in the plan request.
// Keyed fuzzily with separate fuzziness on start and goal features.
auto default_features = TrajectoryCache::getDefaultFeatures(start_tolerance, goal_tolerance);

std::string TrajectoryCache::getDefaultSortFeature();  // Sorts by planned execution time.

move_group.setPoseTarget(...);
moveit_msgs::msg::MotionPlanRequest motion_plan_req_msg;
move_group.constructMotionPlanRequest(motion_plan_request_msg);

// Use the cache INSTEAD of planning!
auto fetched_trajectory =
    cache->fetchBestMatchingTrajectory(*move_group_interface, robot_name, motion_plan_req_msg,
                                       /*features=*/default_features,
                                       /*sort_by=*/TrajectoryCache::getDefaultSortFeature(),
                                       /*ascending=*/true);

if (fetched_trajectory)  // Great! We got a cache hit, we can execute it.
{
  move_group.execute(*fetched_trajectory);
}
else  // Otherwise, plan... And put it for posterity!
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    cache->insertTrajectory(
        *interface, robot_name, std::move(plan_req_msg), std::move(plan),
        /*cache_insert_policy=*/BestSeenExecutionTimePolicy(),
        /*prune_worse_trajectories=*/true, /*additional_features=*/{});
  }
}
```

## Main Features

### Overview

This trajectory cache package supports:
- Inserting and fetching trajectories, keyed fuzzily on any feature of the plan request and plan response.
- Ranking cache entries on any keying feature that is supported (e.g. sorting by execution time).
- Optional cache pruning to keep fetch times and database sizes low.
- Generic support for manipulators with any arbitrary number of joints, across any number of move_groups.
- Cache namespacing and partitioning
- Extension points for injecting your own feature keying, cache insert, cache prune, and cache sorting logic.

The cache supports `MotionPlanRequest` and `GetCartesianPaths::Request` out of the box!

### Fully Customizable Behavior

This trajectory cache allows you to inject your own implementations to affect:
- What features of the plan request and plan response to key the cache on
- What cache insert and cache pruning policy to adopt

For example, you may decide to write your own feature extractor to key the cache, and decide when to insert or prune a cache entry on features such as:
- Minimum jerk time
- Path length
- Any other feature not supported by this package!

### Pre-Existing Implementations

The package provides some starter implementations that covers most general cases of motion planning.

For more information, see the implementations of:
- [`FeaturesInterface`](./include/moveit/trajectory_cache/features/features_interface.hpp)
- [`CacheInsertPolicyInterface`](./include/moveit/trajectory_cache/cache_insert_policies/cache_insert_policy_interface.hpp)

#### Cache Keying Features

The following are features of the plan request and response that you can key the cache on.

These support separately configurable fuzzy lookup on start and goal conditions!
Additionally, these features "canonicalize" the inputs to reduce the cardinality of the cache, increasing the chances of cache hits. (e.g., restating poses relative to the planning frame).

Supported Features:
- "Start"
  - `WorkspaceFeatures`: Planning workspace
  - `StartStateJointStateFeatures`: Starting robot joint state
- "Goal"
  - `MaxSpeedAndAccelerationFeatures`: Max velocity, acceleration, and cartesian speed limits
  - `GoalConstraintsFeatures`: Planning request `goal_constraints`
    - This includes ALL joint, position, and orientation constraints (but not constraint regions)!
  - `PathConstraintsFeatures`: Planning request `path_constraints`
  - `TrajectoryConstraintsFeatures`: Planning request `trajectory_constraints`And also workspace parameters, and some others.
- Planning request goal parameters

Additionally, support for user-specified features are provided for query-only or cache metadata tagging constant features. (See [`constant_features.hpp`](./include/moveit/trajectory_cache/features/constant_features.hpp))

Similar support exists for the cartesian variants of these.

#### Cache Insert and Pruning Policies

The following are cache insertion and pruning policies to govern when cache entries are inserted, and how they are (optionally) pruned.

Supported Cache Insert Policies:
- `BestSeenExecutionTimePolicy`: Only insert best seen execution time, optionally prune on best execution time.
- `AlwaysInsertNeverPrunePolicy`: Always insert, never prune

## Working Principle

If a plan request has features (e.g., start, goal, and constraint conditions) that are "close enough" to an entry in the cache, then the cached trajectory should be reusable for that request, allowing us to skip planning.

The cache extracts these features from the planning request and plan response, storing them in the cache database.
When a new planning request is used to attempt to fetch a matching plan, the cache attempts to fuzzily match the request to pre-existing cache entries keyed on similar requests.
Any "close enough" matches are then returned as valid cache hits for plan reuse, with the definition of "close enough" depending on the type of feature that is being extracted.

## Benefits

A trajectory cache helps:
- Cut down on planning time
- Allows for consistent predictable behavior of used together with a stochastic planner
  - It effectively allows you to "freeze" a move

To explain this, consider that planners in MoveIt generally fall under two main camps: stochastic/probabilistic, and optimization based.
The probabilistic planners are fast, but usually non-deterministic, whereas the optimization based ones are usually slow, but deterministic.

One way to get around this is to pre-plan and manually select and label robot trajectories to "freeze" in a trajectory database to then replay by name, which avoids needing to spend time to replan, and allows you to ensure that motions are constant and repeatable.
However, this approach is not very flexible and not easily reused.

The trajectory cache improves upon this approach by allowing a user to "freeze" and store successful plans, but **also** look up those plans in a more generalizable and natural way, using the planning request itself to key the cache, effectively allowing the cache to stand in for a planning call.

Furthermore, the specific properties of this trajectory cache provides further unique benefits:
1. With fuzzy matching, "frozen" plans are much easier to look up and reuse, while simultaneously increasing the chances of a cache hit.
2. The ability to rank trajectories will, if used with a stochastic planner over sufficient runs, cause the cache to eventually converge to increasingly optimal plans.

Finally, the cache makes use of pruning to optimize fetch times, and also finds ways to "canonicalize" features of the keying request to increase chances of a cache hit.

## Best Practices

- Since this cache does not yet support collisions, ensure the planning scene and obstacles remain static, or always validate the fetched plan for collisions
- When using the default cache features, have looser start fuzziness, and stricter goal fuzziness
- Move the robot to fixed starting poses where possible before planning to increase the chances of a cache hit
- Use the cache where repetitive, non-dynamic motion is likely to occur (e.g. known plans, short planned moves, etc.)

Additionally, you may build abstractions on top of the class, for example, to expose the following behaviors:
- `TrainingOverwrite`: Always plan, and write to cache, pruning all worse trajectories for "matching" cache keys
- `TrainingAppendOnly`: Always plan, and always add to the cache.
- `ExecuteBestEffort`: Rely on cache wherever possible, but plan otherwise.
- `ExecuteReadOnly`: Only execute if cache hit occurred.

You can see how such behaviors effectively model the "dev" and "deploy" phases of a robot deployment, and how they could be useful.
