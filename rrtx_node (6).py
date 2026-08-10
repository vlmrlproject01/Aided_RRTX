"""Map-aware RRTX ROS 2 node.

Static obstacles come from /map. Live /scan points are overlaid as temporary
obstacles. Planning, path publication and robot pose all use the map frame.
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from rrtx_planner.rrtx_algorithm import (
    INFLATION_RADIUS,
    ROBOT_RADIUS,
    OccupancyGrid2D,
    RRTX,
    combine_grids,
    is_collision_free,
)


class RRTXPlanner(Node):
    MAX_LINEAR_VEL = 0.22
    MAX_ANGULAR_VEL = 2.84
    CMD_LINEAR_LIMIT = 0.18
    CMD_ANGULAR_LIMIT = 0.60

    GOAL_TOLERANCE = 0.18
    WAYPOINT_TOLERANCE = 0.25
    MAX_SCAN_RANGE = 3.5
    PLANNING_PERIOD = 0.25
    RRTX_ITERATIONS = 300
    DYNAMIC_OBSTACLE_TTL = 2.5

    def __init__(self):
        super().__init__("rrtx_planner")
        self.get_logger().info("Map-aware RRTX Planner started.")

        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 2.0)
        self.goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
        )
        self.get_logger().info(f"Goal in map frame: {self.goal}")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_subscription = self.create_subscription(
            OccupancyGridMsg,
            "/map",
            self.map_callback,
            map_qos,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/rrtx_path", 10)
        self.grid_pub = self.create_publisher(
            OccupancyGridMsg,
            "/rrtx_grid",
            10,
        )

        self.static_grid: Optional[OccupancyGrid2D] = None
        self.dynamic_grid: Optional[OccupancyGrid2D] = None
        self.combined_grid: Optional[OccupancyGrid2D] = None
        self.dynamic_timestamps = None

        self.current_position: Optional[Tuple[float, float]] = None
        self.current_yaw = 0.0

        self.planner: Optional[RRTX] = None
        self.latest_path = []
        self.current_waypoint_idx = 1
        self.last_log_time = self.get_clock().now()

        self.plan_timer = self.create_timer(
            self.PLANNING_PERIOD,
            self.plan_callback,
        )

    def map_callback(self, msg: OccupancyGridMsg) -> None:
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        resolution = float(msg.info.resolution)
        max_x = origin_x + msg.info.width * resolution
        max_y = origin_y + msg.info.height * resolution

        static_grid = OccupancyGrid2D(
            origin_x,
            max_x,
            origin_y,
            max_y,
            resolution,
        )

        data = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height,
            msg.info.width,
        )

        # Occupied >= 65. Unknown cells are blocked for safe global planning.
        static_grid.raw = np.logical_or(data >= 65, data < 0)
        static_grid.compute_inflated(INFLATION_RADIUS)

        self.static_grid = static_grid
        self.dynamic_grid = static_grid.copy_geometry()
        self.dynamic_timestamps = np.full(
            (static_grid.height, static_grid.width),
            -np.inf,
            dtype=float,
        )
        self.combined_grid = combine_grids(
            self.static_grid,
            self.dynamic_grid,
        )

        self.planner = None
        self.latest_path = []
        self.current_waypoint_idx = 1

        self.get_logger().info(
            f"Loaded /map: {msg.info.width} x {msg.info.height}, "
            f"resolution={resolution:.3f}, "
            f"origin=({origin_x:.2f}, {origin_y:.2f})"
        )

    def lookup_robot_pose(self) -> bool:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                Time(),
            )
        except TransformException as exc:
            self.throttled_log(
                f"Waiting for map -> base_footprint TF: {exc}",
                standard=False,
            )
            return False

        self.current_position = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        )
        q = transform.transform.rotation
        self.current_yaw = self.euler_from_quaternion(
            q.x,
            q.y,
            q.z,
            q.w,
        )
        return True

    def scan_callback(self, msg: LaserScan) -> None:
        if self.dynamic_grid is None or self.dynamic_timestamps is None:
            return
        if not self.lookup_robot_pose():
            return

        try:
            laser_tf = self.tf_buffer.lookup_transform(
                "base_footprint",
                msg.header.frame_id,
                Time(),
            )
            tx = laser_tf.transform.translation.x
            ty = laser_tf.transform.translation.y
            q = laser_tf.transform.rotation
            sensor_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        except TransformException as exc:
            self.throttled_log(
                f"Waiting for laser TF: {exc}",
                standard=False,
            )
            return

        now_seconds = self.get_clock().now().nanoseconds / 1e9
        robot_x, robot_y = self.current_position
        angle = msg.angle_min

        for distance in msg.ranges:
            valid = (
                np.isfinite(distance)
                and msg.range_min <= distance <= self.MAX_SCAN_RANGE
            )
            if valid:
                laser_x = distance * math.cos(angle)
                laser_y = distance * math.sin(angle)

                base_x = tx + (
                    laser_x * math.cos(sensor_yaw)
                    - laser_y * math.sin(sensor_yaw)
                )
                base_y = ty + (
                    laser_x * math.sin(sensor_yaw)
                    + laser_y * math.cos(sensor_yaw)
                )

                hit_x = robot_x + (
                    base_x * math.cos(self.current_yaw)
                    - base_y * math.sin(self.current_yaw)
                )
                hit_y = robot_y + (
                    base_x * math.sin(self.current_yaw)
                    + base_y * math.cos(self.current_yaw)
                )

                gx, gy = self.dynamic_grid.world_to_grid(hit_x, hit_y)
                if self.dynamic_grid.in_grid_bounds(gx, gy):
                    self.dynamic_grid.raw[gy, gx] = True
                    self.dynamic_timestamps[gy, gx] = now_seconds

            angle += msg.angle_increment

    def expire_dynamic_obstacles(self) -> None:
        if self.dynamic_grid is None or self.dynamic_timestamps is None:
            return

        now_seconds = self.get_clock().now().nanoseconds / 1e9
        expired = (
            now_seconds - self.dynamic_timestamps
            > self.DYNAMIC_OBSTACLE_TTL
        )
        self.dynamic_grid.raw[expired] = False

    def plan_callback(self) -> None:
        if self.static_grid is None:
            self.throttled_log("Waiting for /map.", standard=False)
            return
        if not self.lookup_robot_pose():
            return

        self.expire_dynamic_obstacles()
        self.combined_grid = combine_grids(
            self.static_grid,
            self.dynamic_grid,
        )
        self.combined_grid.clear_around(
            self.current_position[0],
            self.current_position[1],
            ROBOT_RADIUS + 0.03,
        )

        if self.planner is None:
            if not is_collision_free(self.goal, self.combined_grid):
                self.throttled_log(
                    f"Goal {self.goal} is occupied or outside /map.",
                    standard=False,
                )
                self.stop_robot()
                return

            self.planner = RRTX(
                self.current_position,
                self.goal,
                self.combined_grid,
            )
            self.get_logger().info(
                "RRTX graph initialized from saved /map."
            )
        else:
            self.planner.update_start(self.current_position)
            self.planner.update_grid(self.combined_grid)
            self.planner.sync_with_grid()

        self.planner.grow_tree(self.RRTX_ITERATIONS)
        new_path = self.planner.extract_path()

        if new_path:
            self.latest_path = new_path
            self.current_waypoint_idx = self.closest_forward_waypoint(
                new_path
            )
        elif not self.path_is_safe(self.latest_path):
            self.latest_path = []
            self.stop_robot()
            self.throttled_log(
                "No safe path available; RRTX is continuing to search.",
                standard=False,
            )

        self.publish_path(self.latest_path)
        self.publish_grid()
        self.follow_path()

    def closest_forward_waypoint(self, path) -> int:
        if len(path) < 2 or self.current_position is None:
            return 1

        distances = [
            math.hypot(
                point[0] - self.current_position[0],
                point[1] - self.current_position[1],
            )
            for point in path
        ]
        closest = int(np.argmin(distances))
        return min(max(closest + 1, 1), len(path) - 1)

    def path_is_safe(self, path) -> bool:
        if (
            self.combined_grid is None
            or not path
            or self.current_position is None
        ):
            return False

        from rrtx_planner.rrtx_algorithm import is_edge_collision_free

        previous = self.current_position
        for point in path[1:]:
            if not is_edge_collision_free(
                previous,
                point,
                self.combined_grid,
            ):
                return False
            previous = point
        return True

    def follow_path(self) -> None:
        if (
            self.current_position is None
            or len(self.latest_path) < 2
        ):
            self.stop_robot()
            return

        robot_x, robot_y = self.current_position
        goal_distance = math.hypot(
            self.goal[0] - robot_x,
            self.goal[1] - robot_y,
        )
        if goal_distance <= self.GOAL_TOLERANCE:
            self.stop_robot()
            self.throttled_log("Goal reached successfully.")
            return

        self.current_waypoint_idx = min(
            max(self.current_waypoint_idx, 1),
            len(self.latest_path) - 1,
        )

        target_x, target_y = self.latest_path[
            self.current_waypoint_idx
        ]
        target_distance = math.hypot(
            target_x - robot_x,
            target_y - robot_y,
        )

        if (
            target_distance < self.WAYPOINT_TOLERANCE
            and self.current_waypoint_idx < len(self.latest_path) - 1
        ):
            self.current_waypoint_idx += 1
            target_x, target_y = self.latest_path[
                self.current_waypoint_idx
            ]
            target_distance = math.hypot(
                target_x - robot_x,
                target_y - robot_y,
            )

        desired_yaw = math.atan2(
            target_y - robot_y,
            target_x - robot_x,
        )
        yaw_error = math.atan2(
            math.sin(desired_yaw - self.current_yaw),
            math.cos(desired_yaw - self.current_yaw),
        )

        command = Twist()
        if abs(yaw_error) > 0.65:
            command.angular.z = math.copysign(
                self.CMD_ANGULAR_LIMIT,
                yaw_error,
            )
        else:
            heading_scale = max(
                0.25,
                1.0 - abs(yaw_error) / 0.65,
            )
            command.linear.x = min(
                self.CMD_LINEAR_LIMIT,
                0.7 * target_distance,
            ) * heading_scale
            command.angular.z = max(
                -self.CMD_ANGULAR_LIMIT,
                min(self.CMD_ANGULAR_LIMIT, 1.5 * yaw_error),
            )

        command.linear.x = max(
            -self.MAX_LINEAR_VEL,
            min(self.MAX_LINEAR_VEL, command.linear.x),
        )
        command.angular.z = max(
            -self.MAX_ANGULAR_VEL,
            min(self.MAX_ANGULAR_VEL, command.angular.z),
        )
        self.cmd_pub.publish(command)

    def publish_path(self, path) -> None:
        message = Path()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()

        for x, y in path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        self.path_pub.publish(message)

    def publish_grid(self) -> None:
        if self.combined_grid is None:
            return

        message = OccupancyGridMsg()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = self.combined_grid.resolution
        message.info.width = self.combined_grid.width
        message.info.height = self.combined_grid.height
        message.info.origin.position.x = self.combined_grid.origin_x
        message.info.origin.position.y = self.combined_grid.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = np.where(
            self.combined_grid.inflated,
            100,
            0,
        ).astype(np.int8).flatten().tolist()

        self.grid_pub.publish(message)

    def stop_robot(self) -> None:
        if rclpy.ok():
            self.cmd_pub.publish(Twist())

    @staticmethod
    def euler_from_quaternion(x, y, z, w) -> float:
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    def throttled_log(self, text: str, standard: bool = True) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds <= 2_000_000_000:
            return
        if standard:
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)
        self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = RRTXPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("RRTX Planner stopped by user.")
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
