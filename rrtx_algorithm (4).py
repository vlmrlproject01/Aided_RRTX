"""Simplified 2-D RRTX-style planner for TurtleBot3.

The planner operates on a supplied OccupancyGrid2D. The ROS node can build that
grid from a saved /map and overlay live /scan obstacles.
"""

import heapq
import math
from typing import List, Optional, Set, Tuple

import numpy as np

np.random.seed(42)

GRID_RESOLUTION = 0.05
ROBOT_RADIUS = 0.105
SAFETY_MARGIN = 0.08
INFLATION_RADIUS = ROBOT_RADIUS + SAFETY_MARGIN

STEP_SIZE = 0.5
NEIGHBOR_RADIUS = 1.2
GOAL_BIAS = 0.15
DUPLICATE_NODE_TOLERANCE = 0.05
MAX_TREE_NODES = 2500


class OccupancyGrid2D:
    def __init__(
        self,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        resolution: float = GRID_RESOLUTION,
    ):
        self.resolution = float(resolution)
        self.origin_x = float(min_x)
        self.origin_y = float(min_y)
        self.width = max(1, int(math.ceil((max_x - min_x) / self.resolution)))
        self.height = max(1, int(math.ceil((max_y - min_y) / self.resolution)))
        self.raw = np.zeros((self.height, self.width), dtype=bool)
        self.inflated = np.zeros((self.height, self.width), dtype=bool)

    @property
    def max_x(self) -> float:
        return self.origin_x + self.width * self.resolution

    @property
    def max_y(self) -> float:
        return self.origin_y + self.height * self.resolution

    def copy_geometry(self) -> "OccupancyGrid2D":
        return OccupancyGrid2D(
            self.origin_x,
            self.max_x,
            self.origin_y,
            self.max_y,
            self.resolution,
        )

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def in_grid_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def in_world_bounds(self, x: float, y: float) -> bool:
        return (
            self.origin_x <= x < self.max_x
            and self.origin_y <= y < self.max_y
        )

    def mark_occupied(self, x: float, y: float) -> None:
        gx, gy = self.world_to_grid(x, y)
        if self.in_grid_bounds(gx, gy):
            self.raw[gy, gx] = True

    def mark_free(self, x: float, y: float) -> None:
        gx, gy = self.world_to_grid(x, y)
        if self.in_grid_bounds(gx, gy):
            self.raw[gy, gx] = False

    def clear(self) -> None:
        self.raw.fill(False)
        self.inflated.fill(False)

    def compute_inflated(self, radius_m: float = INFLATION_RADIUS) -> None:
        radius_cells = max(1, int(math.ceil(radius_m / self.resolution)))
        occupied = np.argwhere(self.raw)
        inflated = self.raw.copy()

        if occupied.size:
            offsets = [
                (dy, dx)
                for dy in range(-radius_cells, radius_cells + 1)
                for dx in range(-radius_cells, radius_cells + 1)
                if dy * dy + dx * dx <= radius_cells * radius_cells
            ]
            for gy, gx in occupied:
                for dy, dx in offsets:
                    ny, nx = gy + dy, gx + dx
                    if 0 <= ny < self.height and 0 <= nx < self.width:
                        inflated[ny, nx] = True

        self.inflated = inflated

    def clear_around(self, x: float, y: float, radius: float) -> None:
        gx, gy = self.world_to_grid(x, y)
        if not self.in_grid_bounds(gx, gy):
            return

        r_cells = max(1, int(math.ceil(radius / self.resolution)))
        y0 = max(0, gy - r_cells)
        y1 = min(self.height, gy + r_cells + 1)
        x0 = max(0, gx - r_cells)
        x1 = min(self.width, gx + r_cells + 1)

        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = (yy - gy) ** 2 + (xx - gx) ** 2 <= r_cells ** 2
        self.inflated[y0:y1, x0:x1][mask] = False

    def is_free_world(self, x: float, y: float) -> bool:
        if not self.in_world_bounds(x, y):
            return False
        gx, gy = self.world_to_grid(x, y)
        return self.in_grid_bounds(gx, gy) and not self.inflated[gy, gx]


def combine_grids(
    static_grid: OccupancyGrid2D,
    dynamic_grid: OccupancyGrid2D,
) -> OccupancyGrid2D:
    if (
        static_grid.width != dynamic_grid.width
        or static_grid.height != dynamic_grid.height
        or static_grid.resolution != dynamic_grid.resolution
        or static_grid.origin_x != dynamic_grid.origin_x
        or static_grid.origin_y != dynamic_grid.origin_y
    ):
        raise ValueError("Static and dynamic grids must have identical geometry.")

    combined = static_grid.copy_geometry()
    combined.raw = np.logical_or(static_grid.raw, dynamic_grid.raw)
    combined.compute_inflated(INFLATION_RADIUS)
    return combined


def is_collision_free(
    point: Tuple[float, float],
    grid: OccupancyGrid2D,
) -> bool:
    return grid.is_free_world(point[0], point[1])


def is_edge_collision_free(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    grid: OccupancyGrid2D,
) -> bool:
    distance = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    checks = max(10, int(distance / max(grid.resolution * 0.5, 1e-6)))

    for i in range(checks + 1):
        t = i / checks
        x = p1[0] * (1.0 - t) + p2[0] * t
        y = p1[1] * (1.0 - t) + p2[1] * t
        if not is_collision_free((x, y), grid):
            return False
    return True


class RRTXNode:
    def __init__(self, position: Tuple[float, float]):
        self.position = position
        self.parent: Optional["RRTXNode"] = None
        self.children: Set["RRTXNode"] = set()
        self.neighbors: Set["RRTXNode"] = set()
        self.g = float("inf")
        self.lmc = float("inf")

    def __lt__(self, other: "RRTXNode") -> bool:
        return (min(self.g, self.lmc), self.g) < (
            min(other.g, other.lmc),
            other.g,
        )


class RRTX:
    def __init__(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        grid: OccupancyGrid2D,
    ):
        self.start_pos = start
        self.goal_pos = goal
        self.grid = grid

        self.nodes: List[RRTXNode] = []
        self.goal_node = RRTXNode(goal)
        self.goal_node.g = 0.0
        self.goal_node.lmc = 0.0
        self.nodes.append(self.goal_node)

        self.queue: List[Tuple[Tuple[float, float], RRTXNode]] = []
        self.queue_set: Set[RRTXNode] = set()

    def update_start(self, new_start: Tuple[float, float]) -> None:
        self.start_pos = new_start

    def update_grid(self, grid: OccupancyGrid2D) -> None:
        self.grid = grid

    @staticmethod
    def distance(p1, p2) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def _get_nearby_nodes(
        self,
        position: Tuple[float, float],
        radius: float,
    ) -> List[RRTXNode]:
        if not self.nodes:
            return []
        coords = np.asarray([node.position for node in self.nodes])
        distances = np.hypot(
            coords[:, 0] - position[0],
            coords[:, 1] - position[1],
        )
        return [
            self.nodes[index]
            for index in np.where(distances <= radius)[0]
        ]

    def steer(self, from_point, to_point):
        distance = self.distance(from_point, to_point)
        if distance <= STEP_SIZE:
            return to_point
        theta = math.atan2(
            to_point[1] - from_point[1],
            to_point[0] - from_point[0],
        )
        return (
            from_point[0] + STEP_SIZE * math.cos(theta),
            from_point[1] + STEP_SIZE * math.sin(theta),
        )

    @staticmethod
    def get_key(node: RRTXNode):
        return min(node.g, node.lmc), node.g

    def insert_or_update_queue(self, node: RRTXNode) -> None:
        if node in self.queue_set:
            self.queue = [
                (key, queued)
                for key, queued in self.queue
                if queued is not node
            ]
            heapq.heapify(self.queue)

        heapq.heappush(self.queue, (self.get_key(node), node))
        self.queue_set.add(node)

    @staticmethod
    def make_parent(
        child: RRTXNode,
        parent: Optional[RRTXNode],
    ) -> None:
        if child is parent:
            return
        if child.parent is not None and child.parent is not parent:
            child.parent.children.discard(child)
        child.parent = parent
        if parent is not None:
            parent.children.add(child)

    def verify_queue(self) -> None:
        while self.queue:
            _, node = heapq.heappop(self.queue)
            if node not in self.queue_set:
                continue
            self.queue_set.remove(node)

            if node.g > node.lmc:
                node.g = node.lmc
                for neighbor in list(node.neighbors):
                    if neighbor is self.goal_node:
                        continue
                    if not is_edge_collision_free(
                        neighbor.position,
                        node.position,
                        self.grid,
                    ):
                        continue
                    candidate = node.g + self.distance(
                        neighbor.position,
                        node.position,
                    )
                    if candidate < neighbor.lmc:
                        neighbor.lmc = candidate
                        self.make_parent(neighbor, node)
                        self.insert_or_update_queue(neighbor)
            else:
                node.g = float("inf")
                for child in list(node.children):
                    child.lmc = float("inf")
                    self.insert_or_update_queue(child)

                best_parent = None
                best_lmc = float("inf")
                for neighbor in node.neighbors:
                    if neighbor.g == float("inf"):
                        continue
                    if not is_edge_collision_free(
                        node.position,
                        neighbor.position,
                        self.grid,
                    ):
                        continue
                    candidate = neighbor.g + self.distance(
                        node.position,
                        neighbor.position,
                    )
                    if candidate < best_lmc:
                        best_lmc = candidate
                        best_parent = neighbor

                node.lmc = best_lmc
                self.make_parent(node, best_parent)

                for neighbor in list(node.neighbors):
                    if neighbor.parent is node:
                        neighbor.lmc = float("inf")
                        self.insert_or_update_queue(neighbor)

                if node.g != node.lmc:
                    self.insert_or_update_queue(node)

    def _start_is_reachable(self) -> bool:
        if not is_collision_free(self.start_pos, self.grid):
            return False
        closest = min(
            self.nodes,
            key=lambda node: self.distance(node.position, self.start_pos),
        )
        return (
            closest.g != float("inf")
            and is_edge_collision_free(
                self.start_pos,
                closest.position,
                self.grid,
            )
        )

    def grow_tree(self, max_iter: int = 250) -> None:
        min_x = self.grid.origin_x
        max_x = self.grid.max_x
        min_y = self.grid.origin_y
        max_y = self.grid.max_y

        for _ in range(max_iter):
            if len(self.nodes) >= 250 and self._start_is_reachable():
                break

            if np.random.random() < GOAL_BIAS:
                random_point = self.start_pos
            else:
                random_point = (
                    np.random.uniform(min_x, max_x),
                    np.random.uniform(min_y, max_y),
                )

            nearest = min(
                self.nodes,
                key=lambda node: self.distance(node.position, random_point),
            )
            new_position = self.steer(nearest.position, random_point)

            if self._get_nearby_nodes(
                new_position,
                DUPLICATE_NODE_TOLERANCE,
            ):
                continue
            if not is_collision_free(new_position, self.grid):
                continue

            new_node = RRTXNode(new_position)
            nearby = self._get_nearby_nodes(
                new_position,
                NEIGHBOR_RADIUS,
            )

            for neighbor in nearby:
                if is_edge_collision_free(
                    new_node.position,
                    neighbor.position,
                    self.grid,
                ):
                    new_node.neighbors.add(neighbor)
                    neighbor.neighbors.add(new_node)

            if not new_node.neighbors:
                continue

            self.nodes.append(new_node)

            for neighbor in new_node.neighbors:
                candidate = neighbor.g + self.distance(
                    new_node.position,
                    neighbor.position,
                )
                if candidate < new_node.lmc:
                    new_node.lmc = candidate
                    self.make_parent(new_node, neighbor)

            if new_node.parent is not None:
                new_node.g = new_node.lmc
                for neighbor in new_node.neighbors:
                    candidate = new_node.g + self.distance(
                        neighbor.position,
                        new_node.position,
                    )
                    if candidate < neighbor.lmc:
                        neighbor.lmc = candidate
                        self.make_parent(neighbor, new_node)
                        self.insert_or_update_queue(neighbor)

            self.verify_queue()

        self.prune_tree(MAX_TREE_NODES)

    def sync_with_grid(self) -> None:
        invalid_nodes = {
            node
            for node in self.nodes
            if not is_collision_free(node.position, self.grid)
        }

        for node in invalid_nodes:
            node.lmc = float("inf")
            self.insert_or_update_queue(node)

        for node in self.nodes:
            for neighbor in list(node.neighbors):
                if (
                    node in invalid_nodes
                    or neighbor in invalid_nodes
                    or not is_edge_collision_free(
                        node.position,
                        neighbor.position,
                        self.grid,
                    )
                ):
                    node.neighbors.discard(neighbor)
                    neighbor.neighbors.discard(node)

                    if node.parent is neighbor:
                        node.lmc = float("inf")
                        self.insert_or_update_queue(node)
                    if neighbor.parent is node:
                        neighbor.lmc = float("inf")
                        self.insert_or_update_queue(neighbor)

        self.verify_queue()

    def prune_tree(self, max_nodes: int) -> None:
        if len(self.nodes) <= max_nodes:
            return

        removable = [
            node
            for node in self.nodes
            if node.g == float("inf") and node is not self.goal_node
        ]

        while len(self.nodes) > max_nodes and removable:
            dead = removable.pop()
            if dead.parent is not None:
                dead.parent.children.discard(dead)
            for neighbor in list(dead.neighbors):
                neighbor.neighbors.discard(dead)
            self.queue_set.discard(dead)
            self.nodes.remove(dead)

        self.queue = [
            (self.get_key(node), node)
            for _, node in self.queue
            if node in self.queue_set
        ]
        heapq.heapify(self.queue)

    def extract_path(self) -> List[Tuple[float, float]]:
        if not self.nodes or not is_collision_free(self.start_pos, self.grid):
            return []

        closest = min(
            self.nodes,
            key=lambda node: self.distance(node.position, self.start_pos),
        )
        if (
            closest.g == float("inf")
            or not is_edge_collision_free(
                self.start_pos,
                closest.position,
                self.grid,
            )
        ):
            return []

        path = [self.start_pos]
        current = closest
        visited = set()

        while current is not None and current not in visited:
            if self.distance(path[-1], current.position) > 0.02:
                path.append(current.position)
            visited.add(current)
            current = current.parent

        if len(path) < 3:
            return path

        smoothed = [path[0]]
        current_index = 0

        while current_index < len(path) - 1:
            next_index = len(path) - 1
            while next_index > current_index + 1:
                if is_edge_collision_free(
                    smoothed[-1],
                    path[next_index],
                    self.grid,
                ):
                    break
                next_index -= 1
            smoothed.append(path[next_index])
            current_index = next_index

        return smoothed
