# RRTX Dynamic Path Planner for TurtleBot3 - ROS 2

A map-aware, two-dimensional **RRTX-style dynamic path planner for TurtleBot3 using ROS 2**.

The planner uses a saved occupancy map for global obstacle information and overlays live LiDAR detections as temporary dynamic obstacles. Instead of rebuilding the entire planning graph every time the environment changes, the implementation retains its existing graph, invalidates affected nodes and edges, repairs inconsistent costs, and continues growing the graph to obtain a new collision-free path.

The ROS 2 node also contains a lightweight waypoint-following controller that publishes velocity commands directly to the TurtleBot3.

> **Note:** This project implements a simplified practical RRTX-style planner. It uses the core ideas of graph reuse, `g`/`lmc` consistency, rewiring and graph repair for changing environments, but it is not intended to be a line-for-line implementation of the complete RRTX research algorithm.

---

## Features

* 2-D sampling-based path planning
* Goal-rooted RRTX-style graph
* Incremental graph reuse between planning cycles
* `g` and `lmc` cost consistency
* Priority-queue-based graph repair
* Neighbor-based rewiring
* Static obstacle handling from `/map`
* Dynamic obstacle detection from `/scan`
* Temporary obstacle expiration
* Obstacle inflation for robot clearance
* Collision checking for both nodes and edges
* Path smoothing using line-of-sight checks
* Continuous replanning while the robot moves
* Automatic rejection of unsafe previous paths
* Direct waypoint following through `/cmd_vel`
* ROS 2 `Path` publication for RViz visualization
* Combined inflated occupancy-grid publication for debugging

---

# System Architecture

The project is divided into two main modules:

```text
rrtx_planner/
├── rrtx_algorithm.py
└── rrtx_node.py
```

## `rrtx_algorithm.py`

Contains the planning algorithm and map representation.

Main components:

```text
OccupancyGrid2D
RRTXNode
RRTX
combine_grids()
is_collision_free()
is_edge_collision_free()
```

Responsibilities include:

* occupancy-grid representation
* obstacle inflation
* world-to-grid conversion
* node and edge collision checking
* random sampling
* steering
* neighbor discovery
* graph construction
* parent selection and rewiring
* `g`/`lmc` consistency repair
* dynamic graph synchronization
* path extraction
* path smoothing
* graph pruning

## `rrtx_node.py`

Contains the ROS 2 interface and robot controller.

Responsibilities include:

* receiving the static map
* receiving LiDAR scans
* obtaining the robot pose through TF
* transforming laser observations into the map coordinate system
* maintaining temporary dynamic obstacles
* combining static and dynamic maps
* updating the RRTX graph
* extracting a safe path
* publishing the path
* publishing the planner occupancy grid
* selecting forward waypoints
* generating TurtleBot3 velocity commands

---

# How the Planner Works

## 1. Static map

The node subscribes to:

```text
/map
```

using `nav_msgs/msg/OccupancyGrid`.

The received ROS occupancy map is converted into the planner's internal `OccupancyGrid2D`.

Cells are interpreted as:

```text
value >= 65    -> occupied
value < 0      -> unknown and treated as occupied
otherwise      -> free
```

Unknown cells are deliberately blocked so the global planner does not plan through unexplored regions.

The resolution and dimensions used by the planner are obtained directly from the incoming `/map`.

---

## 2. Obstacle inflation

The robot is represented using a circular safety region.

Configured values:

```text
Robot radius   = 0.105 m
Safety margin  = 0.080 m
--------------------------------
Inflation      = 0.185 m
```

Therefore:

```text
INFLATION_RADIUS = ROBOT_RADIUS + SAFETY_MARGIN
                 = 0.105 + 0.080
                 = 0.185 m
```

Occupied cells are expanded using this radius before collision checking.

This allows the planner to plan using the robot's physical footprint and an additional safety margin rather than treating the robot as an ideal point.

---

# Dynamic Obstacle Detection

The planner subscribes to:

```text
/scan
```

using:

```text
sensor_msgs/msg/LaserScan
```

Valid LiDAR returns up to:

```text
3.5 m
```

are converted from the laser coordinate frame into `base_footprint` and then into the `map` coordinate frame.

The resulting obstacle hit positions are inserted into a separate dynamic occupancy grid.

The planner therefore maintains:

```text
Static Grid
    +
Dynamic LiDAR Grid
    =
Combined Planning Grid
```

The static and dynamic grids are combined using a logical OR operation before obstacle inflation is recomputed.

---

# Temporary Dynamic Obstacles

LiDAR obstacles are temporary.

Each dynamic obstacle cell stores the time at which it was last observed.

The configured lifetime is:

```text
DYNAMIC_OBSTACLE_TTL = 2.5 seconds
```

If a dynamic obstacle is not observed again for more than 2.5 seconds, its dynamic-grid cell is cleared.

This allows obstacles that move away from their previous positions to disappear automatically from the planning environment.

The implementation records LiDAR hit endpoints; it does not perform free-space ray tracing along each laser beam.

---

# Coordinate Frames

Planning is performed entirely in:

```text
map
```

The node requires a valid TF transform:

```text
map -> base_footprint
```

to determine the current robot position and heading.

For LiDAR observations, the node also requests the transformation between the laser frame contained in:

```text
LaserScan.header.frame_id
```

and:

```text
base_footprint
```

The resulting laser points are then transformed into the map frame using the current robot pose.

---

# RRTX-Style Planning

## Goal-rooted graph

Unlike a conventional forward RRT that begins at the robot, this implementation initializes the graph at the **goal**.

The goal node starts with:

```text
g   = 0
lmc = 0
```

All other nodes initially have:

```text
g   = infinity
lmc = infinity
```

As the graph grows, nodes obtain parents leading toward lower-cost nodes and ultimately toward the goal.

The current robot position is continuously updated as the start position.

This means that the graph can remain in memory while the robot moves.

---

# Sampling

During tree growth, samples are selected either randomly from the map or at the robot's current position.

Configured sampling parameters:

```text
STEP_SIZE                 = 0.50 m
NEIGHBOR_RADIUS           = 1.20 m
GOAL_BIAS                 = 0.15
DUPLICATE_NODE_TOLERANCE  = 0.05 m
```

In this goal-rooted implementation, the `GOAL_BIAS` variable causes a 15% sampling bias toward the **current start/robot position**.

Random sampling uses:

```python
np.random.seed(42)
```

which makes the random sequence reproducible from a fresh process.

---

# Steering

The nearest existing graph node is identified for each sample.

If the sample is farther than:

```text
0.50 m
```

the new position is limited to one `STEP_SIZE` in the sample direction.

Otherwise, the sampled position itself is used.

---

# Neighbor Connections

Each new node searches for graph nodes within:

```text
1.20 m
```

Only neighbors connected by collision-free straight-line edges are accepted.

The implementation maintains bidirectional neighbor relationships:

```text
new_node <-> neighbor
```

These neighbor relationships allow alternative parents to be selected during rewiring and graph repair.

---

# Collision Checking

Two levels of collision detection are used.

## Point collision checking

A position is valid only when it:

1. lies inside the map boundaries, and
2. lies outside the inflated obstacle grid.

## Edge collision checking

Potential graph edges are sampled at multiple intermediate points.

The implementation checks at least:

```text
10 samples
```

per edge and increases the number of samples for longer edges according to the map resolution.

An edge is rejected immediately if any sampled point lies inside an inflated obstacle.

---

# `g` and `lmc` Costs

Each RRTX node maintains:

```text
g
lmc
```

`g` represents the node's current graph cost.

`lmc` represents its locally calculated one-step-lookahead cost.

The priority key is:

```text
(min(g, lmc), g)
```

Nodes whose values become inconsistent are inserted into a priority queue for repair.

---

# Graph Repair

When a node satisfies:

```text
g > lmc
```

its new locally calculated cost is accepted:

```text
g = lmc
```

and the improvement can propagate to neighboring nodes.

When a previously valid connection becomes invalid, affected nodes can instead have their cost returned to:

```text
infinity
```

The planner then searches their remaining collision-free neighbors for the best replacement parent.

This allows part of the existing graph to be repaired instead of discarding the complete planning structure.

---

# Responding to New Obstacles

On each planning cycle, the current static and dynamic maps are combined.

The existing planner is then updated using the latest grid.

The planner checks its graph for:

* nodes that are now inside obstacles
* edges that now intersect obstacles
* parent-child connections invalidated by obstacles

Invalid connections are removed.

Affected nodes are marked inconsistent and placed into the repair queue.

The queue is processed so alternative valid routes can propagate through the graph.

After repairing the existing graph, additional samples are generated to expand the graph into newly useful regions.

---

# Continuous Planning Cycle

The ROS node uses:

```text
PLANNING_PERIOD = 0.25 seconds
```

which corresponds to:

```text
1 / 0.25 = 4 planning callbacks per second
```

Each callback performs up to:

```text
RRTX_ITERATIONS = 300
```

tree-growth iterations.

Tree growth can terminate earlier once the graph contains at least 250 nodes and the current robot position can already connect to a finite-cost region of the graph.

---

# Graph Size Management

The configured graph threshold is:

```text
MAX_TREE_NODES = 2500
```

When the graph exceeds this value, the pruning routine removes unreachable nodes whose:

```text
g = infinity
```

The goal node is never removed.

Because pruning specifically targets unreachable nodes, `MAX_TREE_NODES` should be understood as a pruning threshold rather than an unconditional hard cap if no removable unreachable nodes are available.

---

# Path Extraction

To create a path, the planner:

1. finds the graph node closest to the current robot position
2. checks that the robot can connect to it without collision
3. verifies that the node has a finite cost
4. inserts the current robot position as the beginning of the path
5. follows parent links through the graph
6. obtains a route leading toward the goal

If no finite collision-free connection from the robot to the graph exists, no new path is returned.

---

# Path Smoothing

The raw parent chain can contain many unnecessary intermediate points.

A line-of-sight smoothing stage therefore attempts to connect each retained waypoint directly to the farthest later waypoint that can be reached without intersecting an obstacle.

Conceptually:

```text
Raw graph path:

Start -> A -> B -> C -> D -> Goal

If Start can directly reach C:

Start ---------> C -> D -> Goal
```

This reduces unnecessary turns and produces a simpler waypoint path for the controller.

---

# Previous Path Safety

Failure to produce a new path does not automatically cause the robot to discard its previous route.

The node first checks whether the existing path remains collision-free in the current combined map.

If the old path remains safe, it can continue to be used while RRTX continues searching.

If the old path has become unsafe:

```text
latest_path = []
```

and the robot is stopped.

This behavior prevents the robot from continuing along a route that has become blocked.

---

# Robot Self-Clearance

Before planning, the node clears a small region of the **inflated planning grid** around the current robot position.

The radius used is:

```text
ROBOT_RADIUS + 0.03
= 0.105 + 0.03
= 0.135 m
```

This prevents inflation or local LiDAR observations from trapping the robot inside its own occupied region.

---

# Path Following

The planner contains its own lightweight waypoint controller.

It does not require a separate path-following controller inside these two modules.

The robot follows the next forward waypoint from the generated path.

Configured tolerances are:

```text
Goal tolerance      = 0.18 m
Waypoint tolerance  = 0.25 m
```

Once the robot comes within 0.18 m of the final goal, a zero `Twist` command is published and the robot stops.

---

# Motion Control

TurtleBot3 velocity limits represented in the node are:

```text
MAX_LINEAR_VEL   = 0.22 m/s
MAX_ANGULAR_VEL  = 2.84 rad/s
```

The planner deliberately uses more conservative command limits:

```text
CMD_LINEAR_LIMIT   = 0.18 m/s
CMD_ANGULAR_LIMIT  = 0.60 rad/s
```

If the heading error is greater than:

```text
0.65 rad
```

the robot rotates in place.

Otherwise, forward motion is enabled and scaled according to both waypoint distance and heading error.

Angular velocity is proportional to heading error and limited to the configured command range.

---

# ROS 2 Interfaces

## Subscribed Topics

| Topic   | Message Type                 | Purpose                     |
| ------- | ---------------------------- | --------------------------- |
| `/map`  | `nav_msgs/msg/OccupancyGrid` | Static global occupancy map |
| `/scan` | `sensor_msgs/msg/LaserScan`  | Live obstacle observations  |

## Published Topics

| Topic        | Message Type                 | Purpose                         |
| ------------ | ---------------------------- | ------------------------------- |
| `/cmd_vel`   | `geometry_msgs/msg/Twist`    | TurtleBot3 velocity commands    |
| `/rrtx_path` | `nav_msgs/msg/Path`          | Current planned path            |
| `/rrtx_grid` | `nav_msgs/msg/OccupancyGrid` | Combined inflated planning grid |

---

# ROS 2 Parameters

The goal is provided in the map coordinate frame using two node parameters:

```text
goal_x
goal_y
```

Default values are:

```text
goal_x = 2.0
goal_y = 2.0
```

The current implementation reads these parameters when the node is initialized.

For example, if the package's `setup.py` registers the node executable as `rrtx_node`, a goal could be supplied using:

```bash
ros2 run rrtx_planner rrtx_node --ros-args \
  -p goal_x:=2.0 \
  -p goal_y:=2.0
```

The exact `ros2 run` executable name depends on the console-script entry configured in the package's `setup.py`, which is outside these two source files.

---

# Dependencies

The code directly uses the following Python and ROS 2 packages:

```text
Python 3
NumPy

rclpy
geometry_msgs
nav_msgs
sensor_msgs
tf2_ros
```

A robot or simulation must additionally provide the interfaces required by the node:

```text
/map
/scan
map -> base_footprint TF
Laser frame -> base_footprint TF
/cmd_vel
```

---

# Building in a ROS 2 Workspace

Place the package inside the `src` directory of a ROS 2 workspace.

Example:

```text
ros2_ws/
└── src/
    └── rrtx_planner/
        └── rrtx_planner/
            ├── __init__.py
            ├── rrtx_algorithm.py
            └── rrtx_node.py
```

The import used by the ROS node requires the algorithm module to be available as:

```python
rrtx_planner.rrtx_algorithm
```

After the package metadata and console entry point are configured, build the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

# Running the Planner

Before starting the planner, ensure that:

```text
1. A /map is being published.
2. /scan is available.
3. map -> base_footprint TF is available.
4. The LiDAR frame can be transformed into base_footprint.
5. The robot accepts velocity commands through /cmd_vel.
```

Then start the planner with the desired goal coordinates.

Example, assuming the console executable is named `rrtx_node`:

```bash
ros2 run rrtx_planner rrtx_node --ros-args \
  -p goal_x:=2.0 \
  -p goal_y:=2.0
```

The goal coordinates are interpreted in the `map` frame.

---

# RViz Visualization

Set the RViz fixed frame to:

```text
map
```

Useful displays include:

## Planner grid

Add a **Map** display:

```text
Topic: /rrtx_grid
```

This shows the combined and inflated occupancy grid used for collision checking.

## Planned path

Add a **Path** display:

```text
Topic: /rrtx_path
```

This shows the current smoothed RRTX path.

## LaserScan

The existing:

```text
/scan
```

topic can also be displayed to compare live LiDAR observations with the planner's dynamic obstacle grid.

---

# Example Planning Sequence

A normal planning cycle is approximately:

```text
/map
  |
  v
Static occupancy grid
  |
  +----------------------------+
                               |
/scan                          |
  |                            |
  v                            |
Temporary dynamic grid         |
  |                            |
  +------------+---------------+
               |
               v
       Combine both grids
               |
               v
       Inflate obstacles
               |
               v
    Update existing RRTX graph
               |
               v
  Invalidate blocked nodes/edges
               |
               v
      Repair g/lmc values
               |
               v
       Grow graph further
               |
               v
         Extract path
               |
               v
          Smooth path
               |
               v
        Validate safety
               |
               v
        /rrtx_path
               |
               v
       Waypoint controller
               |
               v
          /cmd_vel
```

---

# Behavior When an Obstacle Appears

Suppose the TurtleBot3 is following:

```text
Start ---------- A ---------- B ---------- Goal
```

and a new obstacle is detected between `A` and `B`.

The new LiDAR observation is inserted into the dynamic map:

```text
Start ---------- A ---- X ---- B ---------- Goal
                       obstacle
```

During the next planning cycle:

1. the dynamic map is combined with the static map
2. the affected edge fails collision checking
3. the edge is removed from the RRTX graph
4. affected costs are made inconsistent
5. the repair queue propagates the change
6. alternative valid parents are searched
7. additional graph samples can be generated
8. a new path is extracted if one becomes available

For example:

```text
                   C -------- D
                  /            \
Start ---------- A              B ---------- Goal
                  \____________/
                     new route
```

The important point is that the complete graph does not have to be intentionally discarded every time the environment changes.

---

# Behavior When No Safe Path Exists

If no new path is found but the previous path remains collision-free, the robot may continue using the previous path.

If:

```text
No new path exists
        AND
Existing path is unsafe
```

the node:

```text
clears the current path
stops the robot
continues planning
```

This provides a basic fail-safe against knowingly following a blocked path.

---

# Main Configuration Values

## Planner

| Setting                      |        Value |
| ---------------------------- | -----------: |
| Default grid resolution      |     `0.05 m` |
| Robot radius                 |    `0.105 m` |
| Safety margin                |     `0.08 m` |
| Inflation radius             |    `0.185 m` |
| Step size                    |     `0.50 m` |
| Neighbor radius              |     `1.20 m` |
| Start-position sampling bias |       `0.15` |
| Duplicate-node tolerance     |     `0.05 m` |
| Graph pruning threshold      | `2500 nodes` |

The ROS node uses the actual `/map` resolution when creating its planning grid; `0.05 m` is the default `OccupancyGrid2D` resolution.

## ROS node

| Setting                             |        Value |
| ----------------------------------- | -----------: |
| Planning period                     |     `0.25 s` |
| Nominal planning callback frequency |       `4 Hz` |
| Maximum RRTX iterations/callback    |        `300` |
| Dynamic obstacle lifetime           |      `2.5 s` |
| Maximum processed scan range        |      `3.5 m` |
| Goal tolerance                      |     `0.18 m` |
| Waypoint tolerance                  |     `0.25 m` |
| Controller linear limit             |   `0.18 m/s` |
| Controller angular limit            | `0.60 rad/s` |

---

# Implementation Limitations

The current implementation intentionally remains relatively compact and understandable.

Important limitations include:

* The planner operates only in 2-D Cartesian position space.
* Robot orientation is not part of the planning state.
* Vehicle dynamics are not included in graph expansion.
* LiDAR processing records obstacle hit cells rather than performing full inverse sensor-model ray tracing.
* Old dynamic obstacle locations remain occupied until their 2.5-second lifetime expires.
* Unknown map cells are treated as occupied.
* Goal parameters are read during node initialization rather than through a runtime goal-action interface.
* The included controller is a simple waypoint tracker rather than a trajectory optimizer.
* No acceleration or jerk limiting is implemented in these two modules.
* The path message stores waypoint positions but does not calculate a desired orientation for every path pose.
* Graph-neighbor searches currently inspect the stored node coordinates directly rather than using a spatial indexing structure such as a k-d tree.
* Obstacle inflation is performed directly on the NumPy occupancy grid and can become computationally expensive for large maps with many occupied cells.
* The implementation should be described as **RRTX-style** rather than as a complete implementation of every mechanism in the formal RRTX algorithm.

These limitations also provide clear directions for future development.

---

# Possible Future Improvements

Potential extensions include:

* RViz interactive goal selection
* ROS 2 goal subscriptions or action interfaces
* k-d tree or spatial-hash neighbor search
* more efficient occupancy-grid inflation
* ray-traced LiDAR free-space updates
* separate dynamic-obstacle tracking
* configurable planner constants through ROS parameters
* adaptive sampling
* informed sampling after an initial solution
* improved graph pruning
* visualization of RRTX nodes and graph edges
* visualization of invalidated and rewired edges
* path-cost publication
* velocity and acceleration profiling
* a more advanced path-tracking controller
* automated simulation tests
* planner performance benchmarking

---

# Safety Notice

This project is experimental robotics software.

When testing on a physical TurtleBot3:

* keep the robot within a controlled area
* maintain access to an emergency stop
* begin with conservative velocity limits
* verify `/map`, `/scan` and TF alignment before enabling motion
* verify obstacle inflation against the actual robot footprint
* do not assume that successful simulation testing guarantees safe physical operation

---

# Summary

This project demonstrates a practical ROS 2 implementation of a **dynamic RRTX-style path planner for TurtleBot3**.

Its main design idea is to combine:

```text
Saved Occupancy Map
        +
Live LiDAR Obstacles
        +
Reusable Sampling Graph
        +
Incremental Graph Repair
        +
Path Smoothing
        +
Waypoint Control
```

to allow a TurtleBot3 to plan toward a goal while responding to newly detected obstacles without intentionally rebuilding the complete planning graph during every planning cycle.

The implementation provides a compact foundation for studying sampling-based planning, incremental replanning, occupancy-grid collision checking, ROS 2 TF transformations, dynamic obstacle handling and mobile-robot path following.
