# neonavigation_launch package

neonavigation_launch package provides sample launch file for testing neonavigation meta-package.

## ROS 1

```bash
roslaunch neonavigation_launch demo.launch
```

`launch/demo.launch`, `launch/navigate.launch`, `config/navigate.yaml` and
`config/visualization.rviz` are unchanged and keep working exactly as before.

## ROS 2

```bash
ros2 launch neonavigation_launch demo.launch.py
# headless (no X11):
ros2 launch neonavigation_launch demo.launch.py use_rviz:=false
# one process per node instead of a component container:
ros2 launch neonavigation_launch demo.launch.py use_composition:=false
```

Publish a goal on `goal_pose` (or use the RViz 2 "2D Goal Pose" tool) to make the
simulated robot drive.

### Files

| ROS 1 | ROS 2 | note |
|---|---|---|
| `launch/demo.launch` | `launch/demo.launch.py` | |
| `launch/navigate.launch` | `launch/navigate.launch.py` | |
| `config/navigate.yaml` | `config/navigate_ros2.yaml` | per-node `ros__parameters`, flattened costmap layers |
| `config/visualization.rviz` | `config/visualization_ros2.rviz` | RViz 2 plugin names / QoS / topic names |
| `map/demo_map.yaml`, `map/demo_map_local.yaml` | (same files) | `nav2_map_server` reads the ROS 1 map YAML format as is |

Both sets are installed side by side; nothing was removed.

### Composition

Following the hybrid kit convention every neonavigation node is loaded into one
`rclcpp_components` `component_container_mt` named `neonavigation_container`, with
`use_intra_process_comms` enabled, so `map` -> `costmap_3d` -> `planner_3d` ->
`trajectory_tracker` runs zero-copy inside a single process.

* `use_composition` (default `true`) — `false` starts one process per node.
* `use_intra_process_comms` (default `true`) — only meaningful with composition.
* `container_name` (default `neonavigation_container`) — `demo.launch.py` passes it
  to `navigate.launch.py`, which creates the container, and then loads its own
  extra nodes into the same container with `LoadComposableNodes`.

Two nodes are intentionally *not* composed:

* `tf2_ros/static_transform_publisher` — the component variant
  (`tf2_ros::StaticTransformBroadcasterNode`) only accepts a quaternion, while the
  launch file takes a `sim_robot_yaw` angle. `/tf_static` is not on any zero-copy
  critical path, so the standalone executable (which understands `--yaw`) is used.
* `rviz2` — a GUI process.

### Behaviour differences from the ROS 1 launch files

* **`neonavigation_compatible` is gone.** It only selected the ROS 1 legacy topic
  names; the ROS 2 nodes always use the plain (new) names.
* **Goal topic.** `planner_3d` subscribes to `goal_pose` instead of
  `move_base_simple/goal`; that is what the RViz 2 "2D Goal Pose" tool publishes.
* **`map_server`.** ROS 1 `map_server/map_server` took the YAML as a command line
  argument; `nav2_map_server`'s `map_server` takes it in the `yaml_filename`
  parameter and is a *managed (lifecycle) node*. There is no
  `nav2_lifecycle_manager` in this workspace, so the launch files set
  `autostart_node: true` (the node configures and activates itself) and
  `bond_heartbeat_period: 0.0` (no bond to a manager). The ROS 1 map YAML format
  (`image` / `resolution` / `origin` / `negate` / `occupied_thresh` /
  `free_thresh`) is read unchanged; `mode` stays unset and defaults to `trinary`.
* **RViz.** `rviz/rviz` -> `rviz2/rviz2`, and every display/tool class was renamed
  (`rviz/...` -> `rviz_common/...` / `rviz_default_plugins/...`). Topics that the
  neonavigation nodes latch on ROS 1 use `transient_local` durability on ROS 2, so
  the corresponding displays request `Durability Policy: Transient Local`.
* **`trajectory_tracker_rviz_plugins/PathWithVelocity` has no ROS 2 port.** That
  display is replaced by a standard `rviz_default_plugins/PoseArray` display on
  `/planner_3d/path_poses`, which `planner_3d` publishes regardless of
  `use_path_with_velocity`. The plain `rviz_default_plugins/Path` display on
  `/path` is kept for `use_path_with_velocity:=false`.
* **`linear_expand` / `linear_spread` launch arguments** are still declared for
  command line compatibility, but — exactly as in the ROS 1 `navigate.launch` —
  they are not wired to anything. Set them in `config/navigate_ros2.yaml`.
* **`footprint`** is a launch argument (flat `[x0, y0, x1, y1, ...]` list) and is
  applied to `costmap_3d` / `safety_limiter` only when `simulate:=true`, mirroring
  the conditional `<rosparam param="footprint">` of the ROS 1 file.
* `map_file` has no default and is required whenever `use_map_server` is true;
  `navigate.launch.py` raises a launch error instead of silently starting a
  map server without a map.

### Parameter representation

`config/navigate_ros2.yaml` is the ROS 2 form of `config/navigate.yaml`. Besides
the mandatory `<node name>: ros__parameters:` nesting, the hybridization changed
two representations (see `costmap_cspace/README.md` and
`planner_cspace/README.md`):

* **`costmap_3d` layers.** ROS 1 used arrays of dictionaries. ROS 2 parameters are
  flat, so the ordered layer names live in the `layers` / `static_layers` string
  arrays and each setting is a `layer.<name>.<key>` / `static_layer.<name>.<key>`
  parameter:

  ```yaml
  # ROS 1
  layers:
    - name: overlay1
      type: Costmap3dLayerFootprint
      overlay_mode: max

  # ROS 2
  layers: ["overlay1"]
  layer:
    overlay1:
      type: Costmap3dLayerFootprint
      overlay_mode: max
  ```

* **`footprint`.** ROS 1 `[[x, y], [x, y], ...]` -> ROS 2 flat
  `[x0, y0, x1, y1, ...]` double array. Applies to `costmap_3d` and
  `safety_limiter`.

* **`planner_3d`.** The `dynamic_reconfigure` parameters became plain ROS 2
  parameters with the same names; `ros2 param set` replaces `rqt_reconfigure`.

The node names used as YAML keys (`costmap_3d`, `planner_3d`, `spur`) must match
the node names in the launch files.
