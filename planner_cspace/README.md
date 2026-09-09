# planner_cspace package

The topic names will be migrated to ROS recommended namespace model.
Set `/neonavigation_compatible` parameter to `1` to use new topic names.

## planner_3d

planner_3d node provides 2-D/3-DOF seamless global-local path and motion planner.

### Subscribed topics

* ~/costmap (new: costmap) [costmap_cspace_msgs::CSpace3D]
* ~/costmap_update (new: costmap_update) [costmap_cspace_msgs::CSpace3DUpdate]
* ~/goal (new: move_base_simple/goal) [geometry_msgs::PoseStamped]
* /tf

### Published topics

* ~/path (new: path) [nav_msgs::Path]
* ~/debug [sensor_msgs::PointCloud]
    > debug output of planner internal costmap
* ~/remembered [sensor_msgs::PointCloud]
    > debug output of obstacles probability estimated by BBF
* ~/path_start [geometry_msgs::PoseStamped]
* ~/path_end [geometry_msgs::PoseStamped]
* ~/status [planner_cspace_msgs::PlannerStatus]

### Services

* ~/forget (new: forget_planning_cost) [std_srvs::Empty]

### Called services


### Parameters

* "goal_tolerance_lin" (double, default: 0.05)
* "goal_tolerance_ang" (double, default: 0.1)
* "goal_tolerance_ang_finish" (double, default: 0.05)
* "unknown_cost" (int, default: 100)
* "hist_cnt_max" (int, default: 20)
* "hist_cnt_thres" (int, default: 19)
* "hist_cost" (int, default: 90)
* "hist_ignore_range" (double, default: 1.0)
* "remember_updates" (bool, default: false)
* "local_range" (double, default: 2.5)
* "longcut_range" (double, default: 0.0)
* "esc_range" (double, default: 0.25)
* "find_best" (bool, default: true)
* "pos_jump" (double, default: 1.0)
* "yaw_jump" (double, default: 1.5)
* "jump_detect_frame" (string, default: base_link)
* "force_goal_orientation" (bool, default: true)
* "temporary_escape" (bool, default: true)
* "fast_map_update" (bool, default: false)
* "debug_mode" (string, default: std::string("cost_estim"))
    > debug output data type
    > - "hyst": path hysteresis cost
    > - "cost_estim": estimated cost to the goal used as A\* heuristic function
* "queue_size_limit" (int, default: 0)
* "antialias_start" (bool, default: false)
    > If enabled, the planner searches path from multiple surrounding grids within the grid size to reduce path chattering.

----

## planner_2dof_serial_joints

planner_2dof_serial_joints provides collision avoidance for 2-DOF serial joint (e.g. controlling interfering pairs of sub-tracks for tracked vehicle.)

### Subscribed topics

* ~/trajectory_in (new: trajectory_in) [trajectory_msgs::JointTrajectory]
* ~/joint (new: joint_states) [sensor_msgs::JointState]
* /tf

### Published topics

* ~/trajectory_out (new: joint_trajectory) [trajectory_msgs::JointTrajectory]
* ~/status [planner_cspace_msgs::PlannerStatus]

### Services


### Called services


### Parameters

* "resolution" (int, default: 128)
* "debug_aa" (bool, default: false)
* "replan_interval" (double, default: 0.2)
* "queue_size_limit" (int, default: 0)
* "link0_name" (string, default: std::string("link0"))
* "link1_name" (string, default: std::string("link1"))
* "point_vel_mode" (string, default: std::string("prev"))
* "range" (int, default: 8)
* "num_groups" (int, default: 1)

----

## dummy_robot

stub

----

## patrol

stub

----

## ROS 2 notes

The package is hybridized: the same sources build on ROS 1 (catkin) and ROS 2
(ament). The ROS 1 interface layer is unchanged, so everything above still
describes the ROS 1 behaviour exactly. This section lists what differs when the
nodes are run on ROS 2.

All four nodes (`planner_3d`, `planner_2dof_serial_joints`, `dummy_robot`,
`patrol`) are available both as standalone executables and as `rclcpp`
components (`planner_cspace::planner_3d::Planner3dNode`,
`planner_cspace::planner_2dof_serial_joints::Planner2dofSerialJointsNode`,
`planner_cspace::DummyRobotNode`, `planner_cspace::PatrolActionNode`).

The `/neonavigation_compatible` topic name aliases are ROS 1 only; the ROS 2
nodes always use the plain (new) topic names listed above.

Topics that were latched on ROS 1 are published with `transient_local`
durability on ROS 2, so subscribers have to request `transient_local` as well.

### Actions

`move_base_msgs` does not exist on ROS 2, so the `move_base` action of
`planner_3d` (and the action client of `patrol`) uses
`nav2_msgs/action/NavigateToPose`. The action name is unchanged. The field
mapping is:

| `move_base_msgs/MoveBase` (ROS 1) | `nav2_msgs/NavigateToPose` (ROS 2) |
|---|---|
| `goal.target_pose` | `goal.pose` (`goal.behavior_tree` is ignored) |
| `feedback.base_position` | `feedback.current_pose` |
| (empty result) | `result.error_code` (always `NONE`) and `result.error_msg`, which carries the status text that ROS 1 passed to `setSucceeded()` / `setAborted()` |

`feedback.navigation_time` holds the time since the goal was accepted and
`feedback.distance_remaining` the length of the last published path;
`estimated_time_remaining` and `number_of_recoveries` have no counterpart in
this planner and stay zero.

The `tolerant_move` action keeps using
`planner_cspace_msgs/action/MoveWithTolerance` on both ROS versions.

#### Preemption semantics

`actionlib` preempted a running goal automatically when a new one arrived and
reported it with `setPreempted()`. `rclcpp_action` has neither automatic
preemption nor a `PREEMPTED` terminal state, and `canceled()` may only be used
after a cancel request was accepted. `planner_3d` therefore behaves as follows
on ROS 2:

* A goal that is superseded by a **new goal on the same action server** is
  **aborted** with `"Preempted."` (in `result.error_msg` for `move_base`; logged
  for `tolerant_move`, whose result is empty).
* An **explicitly cancelled** goal is terminated with `canceled()`. The cancel
  request is recorded in the cancel callback and the goal handle is finished
  from the planning timer, because a goal handle may not be terminated from
  within `handle_cancel`.
* A goal that arrives while the **other** action server is busy is **rejected**
  in `handle_goal`. ROS 1 logged an error and left such a goal pending forever.

### `planner_3d`

* `move_base_simple/goal` is named **`goal_pose`**, which is what RViz 2
  publishes for "2D Goal Pose".
* `~/make_plan` (`nav_msgs/srv/GetPlan`) cannot report a failure the way a ROS 1
  service could by returning `false`; when no plan is found the response simply
  holds an empty plan.
* `dynamic_reconfigure` is replaced by plain ROS 2 parameters. The names,
  defaults and value ranges are the ones from `cfg/Planner3D.cfg`, and changes
  are applied through a post-set parameter callback, so `ros2 param set` has the
  same effect as `rqt_reconfigure` had on ROS 1.
* `print_planning_duration` raises the level of this node's logger to `Debug`
  instead of the global rosconsole logger.
* The 100 Hz planning loop of the ROS 1 `main()` runs from a 100 Hz timer, and
  the one-shot `costmap_watchdog` timer is a periodic timer which is re-armed or
  stopped on every costmap update.

### `planner_2dof_serial_joints`

ROS 1 read the per-group settings from the nested private namespace
`~/<group>/<key>`. ROS 2 parameters are strictly flat, so the same settings are
given as dot separated names `<group>.<key>`:

```yaml
# ROS 1
planner_2dof_serial_joints:
  num_groups: 1
  group0_name: group0
  group0:
    link0_name: front_flipper
    link1_name: rear_flipper
    resolution: 128
```

```yaml
# ROS 2
planner_2dof_serial_joints:
  ros__parameters:
    num_groups: 1
    group0_name: group0
    group0:
      link0_name: front_flipper
      link1_name: rear_flipper
      resolution: 128
```

The per-group keys are unchanged (`resolution`, `queue_size_limit`, `range`,
`num_threads`, `weight_cost`, `expand`, `point_vel_mode`, `link0_*`, `link1_*`),
and `debug_aa`, `replan_interval`, `num_groups` and `group<i>_name` stay
node-level parameters. A ROS 2 component is a single node, so one node instance
owns every link group instead of one object per group; the published
`~/<group>/status` and the shared `joint_trajectory` / `trajectory_in` /
`joint_states` topics are the same as on ROS 1.
