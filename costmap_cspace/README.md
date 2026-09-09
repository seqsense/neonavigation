# costmap_cspace package

The topic names will be migrated to ROS recommended namespace model.
Set `/neonavigation_compatible` parameter to `1` to use new topic names.

## costmap_3d

costmap_3d node converts 2-D (x, y) OccupancyGrid to 2-D/3-DOF (x, y, yaw) configuration space based on given footprint.

### single layer mode (simple version)

#### Subscribed topics

* map [nav_msgs::OccupancyGrid]
* map_overlay [nav_msgs::OccupancyGrid]

#### Published topics

* ~/costmap (new: costmap) [costmap_cspace_msgs::CSpace3D]
* ~/costmap_update (new: costmap_update) [costmap_cspace_msgs::CSpace3DUpdate]
* ~/footprint [geometry_msgs::PolygonStamped]
* ~/debug [sensor_msgs::PointCloud]

#### Parameters

* "ang_resolution" (int, default: 16)
* "linear_expand" (double, default: 0.2f)
* "linear_spread" (double, default: 0.5f)
* "linear_spread_min_cost" (int, default: 0)
* "unknown_cost" (int, default: 0)
* "overlay_mode" (string, default: std::string(""))
* "footprint" (?, default: footprint_xml)

### multiple layer mode

#### Subscribed topics

* map [nav_msgs::OccupancyGrid]
* **layer name** [nav_msgs::OccupancyGrid]: Subscribed topics are named according to the layers parameters

#### Published topics

* ~/costmap (new: costmap) [costmap_cspace_msgs::CSpace3D]
* ~/costmap_update (new: costmap_update) [costmap_cspace_msgs::CSpace3DUpdate]
* ~/footprint [geometry_msgs::PolygonStamped]
* ~/debug [sensor_msgs::PointCloud]

#### Parameters

* "ang_resolution" (int, default: 16): for root layer
* "linear_expand" (double, default: 0.2f): for root layer
* "linear_spread" (double, default: 0.5f): for root layer
* "linear_spread_min_cost" (int, default: 0)
* "footprint" (?, default: footprint_xml): for root layer
* "static_layers": array of layer configurations
* "layers": array of layer configurations

Each layer configuration contains:
* "name" (string) layer name
* "type" (string) layer type name
* "parameters" layer specific parameters

Available layer types and parameters are:
- **Costmap3dLayerFootprint**: Configuration space costmap layer according to the given footprint.
  - "linear_expand" (double)
  - "linear_spread" (double)
  - "linear_spread_min_cost" (int, default: 0)
  - "footprint" (?, default: root layer's footprint)
- **Costmap3dLayerPlain**: Costmap layer without considering footpring.
  - "linear_expand" (double)
  - "linear_spread" (double)
  - "linear_spread_min_cost" (int, default: 0)
- **Costmap3dLayerOutput**: Output generated costmap at this point. In most case, this is placed at the last layer.
- **Costmap3dLayerStopPropagation**: Stop propagating parent layer's cost to the child. This can be used at the beginning of layer to ignore changes in static layers.
- **Costmap3dLayerUnknownHandle**: Set unknown cell's cost.
  - "unknown_cost" (int)

See [example parameters](https://github.com/at-wat/neonavigation/blob/master/neonavigation_launch/config/navigate.yaml).

### ROS 2 parameter representation

`costmap_3d` reads the footprint and the layer chain from nested `XmlRpc` values on
ROS 1. ROS 2 parameters are strictly flat and typed, so the same configuration is
expressed as follows on the ROS 2 side (the topics and the runtime behaviour are
identical on both):

* `footprint`: a flat `double` array `[x0, y0, x1, y1, ...]` (at least three
  vertices) instead of an array of `[x, y]` pairs. The same applies to the
  per-layer `footprint` override.
* `layers` / `static_layers`: a `string` array holding the **ordered layer names**,
  and one parameter per setting under the `layer.<name>.` / `static_layer.<name>.`
  prefix. The available keys are unchanged: `type`, `overlay_mode`, `footprint`,
  `linear_expand`, `linear_spread`, `linear_spread_min_cost`, `keep_unknown` and
  `unknown_cost`.
* Omitting `layers` (or leaving it unset) keeps the single-layer backward
  compatibility mode driven by the `overlay_mode` parameter, exactly as on ROS 1.
  Setting `layers` to an explicitly empty list is an error, as it is on ROS 1.

```yaml
# ROS 1
costmap_3d:
  footprint: [[0.2, -0.1], [0.2, 0.1], [-0.2, 0.1], [-0.2, -0.1]]
  static_layers:
    - name: unknown
      type: Costmap3dLayerUnknownHandle
      unknown_cost: 100
  layers:
    - name: overlay
      type: Costmap3dLayerFootprint
      overlay_mode: max
```

```yaml
# ROS 2
costmap_3d:
  ros__parameters:
    footprint: [0.2, -0.1, 0.2, 0.1, -0.2, 0.1, -0.2, -0.1]
    static_layers: ["unknown"]
    static_layer:
      unknown:
        type: Costmap3dLayerUnknownHandle
        unknown_cost: 100
    layers: ["overlay"]
    layer:
      overlay:
        type: Costmap3dLayerFootprint
        overlay_mode: max
```

The overlay map of each layer is still subscribed on a topic named after the
layer (`unknown`, `overlay`, ... in the example above), and `~/footprint`,
`~/debug`, `costmap` and `costmap_update` are published as on ROS 1. Topics that
were latched on ROS 1 use `transient_local` durability on ROS 2, so subscribers
have to request `transient_local` as well.

All four nodes (`costmap_3d`, `laserscan_to_map`, `pointcloud2_to_map`,
`largemap_to_map`) are available on ROS 2 both as standalone executables and as
`rclcpp` components (`costmap_cspace::Costmap3DOFNode`,
`costmap_cspace::LaserscanToMapNode`, `costmap_cspace::Pointcloud2ToMapNode`,
`costmap_cspace::LargeMapToMapNode`).

The diagram below shows how "linear_expand", "linear_spread" and "linear_spread_min_cost" work.
"linear_spread_min_cost" can make the costs of grids near obstacles higher to avoid other costs such as preferences overriding the costs of these grids.

![Screenshot from 2023-09-12 17-19-52](https://github.com/at-wat/neonavigation/assets/8833040/58001b49-b603-47e9-92e7-0517f0aaf8ba)

----
## laserscan_to_map

stub

----
## pointcloud2_to_map

stub

----
## largemap_to_map

stub
