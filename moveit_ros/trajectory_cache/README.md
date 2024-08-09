# Fuzzy-Matching Trajectory Cache

```
 *   This cache does NOT support collision detection!
 *   Plans will be put into and fetched from the cache IGNORING collision.
 *   If your planning scene is expected to change between cache lookups, do NOT
 *   use this cache without validating if the fetched plan will result in collision.
 *
 *   To handle collisions the cache will need to be able to hash the planning scene
 *   world msg (after zeroing out std_msgs/Header timestamps and sequences) and do
 *   an appropriate lookup, or otherwise find a way to determine that a planning
 *   scene is "less than" or "in range of" another (e.g. checking that every
 *   obstacle/mesh exists within the other scene's).
 ```

A trajectory cache based on [`warehouse_ros`](https://github.com/moveit/warehouse_ros) for the move_group planning interface that supports fuzzy lookup for `MotionPlanRequest` and `GetCartesianPath` requests and trajectories.

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

The cache supports MotionPlanRequests and GetCartesianPaths::Requests out of the box!

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

Additionally, support for user-specified features are provided for query-only or cache metadata tagging constant features.

Similar support exists for the cartesian variants of these.

#### Cache Insert and Pruning Policies

The following are cache insertion and pruning policies to govern when cache entries are inserted, and how they are (optionally) pruned.

Supported Cache Insert Policies:
- `BestSeenExecutionTimePolicy`: Only insert best seen execution time, optionally prune on best execution time.
- `AlwaysInsertNeverPrunePolicy`: Always insert, never prune

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

## Working Principle

If a plan request has start, goal, and constraint conditions that are "close enough" (configurable per request) to an entry in the cache, then the cached trajectory should be suitable (as long as obstacles have not changed).

Goal fuzziness is a lot less lenient than start fuzziness by default.

Finally, the databases are sharded by move group, and the constraints are added as columns in a name agnostic fashion (they are even somewhat robust to ordering, because they're sorted!)

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
- !!! This cache does NOT support multi-DOF joints.
- !!! This cache does NOT support constraint regions
- The fuzzy lookup can't be configured on a per-joint basis.