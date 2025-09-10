#!/usr/bin/env python3
import math
import heapq
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf2_geometry_msgs import do_transform_pose


import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw/2); q.z = math.sin(yaw/2)
    return q

class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__('dijkstra_planner')
		
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_waypoint')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('treat_unknown_as_obstacle', True)
        self.declare_parameter('use_8_connected', True)
        self.declare_parameter('inflate_radius', 0.15)  # meters
        self.declare_parameter('traversal_cost_weight', 0.0)
        
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)
                    
        qos_map = QoSProfile(depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, qos_map)
        self._map: Optional[OccupancyGrid] = None
        self._grid: Optional[np.ndarray] = None           # raw occupancy values [-1,0..100], shape (H, W)
        self._obstacles: Optional[np.ndarray] = None      # boolean mask, shape (H, W)

        # TF buffer/listener to get start pose from robot
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Dijkstra planner ready.')
    
    def map_cb(self, msg: OccupancyGrid):
        self._map = msg
        W = msg.info.width
        H = msg.info.height
        grid = np.array(msg.data, dtype=np.int16).reshape((H, W))  # row-major: y first
        self._grid = grid

        occ_th = self.get_parameter('occupied_threshold').get_parameter_value().integer_value
        unknown_as_obs = self.get_parameter('treat_unknown_as_obstacle').get_parameter_value().bool_value
        obstacles = (grid >= occ_th)
        if unknown_as_obs:
            obstacles = np.logical_or(obstacles, grid == -1)

        # Inflate obstacles if requested
        inflate_radius = float(self.get_parameter('inflate_radius').get_parameter_value().double_value)
        if inflate_radius > 1e-6:
            obstacles = self.inflate_obstacles(obstacles, inflate_radius, msg.info.resolution)

        self._obstacles = obstacles
        # self.get_logger().info(f'Map received: {W}x{H}, res={msg.info.resolution:.3f} m/px')

    def goal_cb(self, goal_msg: PoseStamped):
        # Compute path on each new goal
        if self._map is None or self._grid is None or self._obstacles is None:
            self.get_logger().warn('Goal received but map not available yet.')
            return

        global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Ensure goal is in map frame (transform if needed)
        if goal_msg.header.frame_id != global_frame:
            try:
                goal_msg = self.transform_pose(goal_msg, global_frame)
            except Exception as e:
                self.get_logger().error(f'Failed to transform goal to {global_frame}: {e}')
                return

        # Get current robot pose in map frame (start)
        try:
            start_pose = self.lookup_robot_pose(global_frame, base_frame)
        except Exception as e:
            self.get_logger().error(f'Could not get robot pose: {e}')
            return

        path = self.plan_path(start_pose, goal_msg)
        if path is None:
            self.get_logger().warn('No path found.')
            return

        self.path_pub.publish(path)
        self.get_logger().info(f'Path published with {len(path.poses)} poses.')

    def lookup_robot_pose(self, target_frame: str, base_frame: str) -> PoseStamped:
        stamp = rclpy.time.Time()
        trans = self.tf_buffer.lookup_transform(target_frame, base_frame, stamp)
        p = PoseStamped()
        p.header.stamp = trans.header.stamp
        p.header.frame_id = target_frame
        p.pose.position.x = trans.transform.translation.x
        p.pose.position.y = trans.transform.translation.y
        p.pose.position.z = trans.transform.translation.z
        p.pose.orientation = trans.transform.rotation
        return p
    
    def transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped:
        trans = self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id, rclpy.time.Time())
        return do_transform_pose(pose, trans)

    def plan_path(self, start: PoseStamped, goal: PoseStamped) -> Optional[Path]:
        """Plan with plain Dijkstra on an occupancy grid.
        - Uses 4- or 8-connected neighbors
        - No cost penalties, just step cost (1 or sqrt(2))
        - Returns nav_msgs/Path in the map frame
        """
        # 1) Shortcuts to map metadata
        info = self._map.info
        res = info.resolution
        x0 = info.origin.position.x
        y0 = info.origin.position.y
        W, H = info.width, info.height

        # 2) Convert start/goal world -> grid
        s = self.world_to_map(start.pose.position.x, start.pose.position.y, x0, y0, res, W, H)
        g = self.world_to_map(goal.pose.position.x,  goal.pose.position.y,  x0, y0, res, W, H)
        if s is None or g is None:
            self.get_logger().warn('Start or goal outside map bounds.')
            return None
        sx, sy = s
        gx, gy = g

        # Blocked goal? quit early.
        if self._obstacles[gy, gx]:
            self.get_logger().warn('Goal cell is occupied.')
            return None

        # 3) Neighbor set (toggle 8-connected here)
        use_eight = self.get_parameter('use_8_connected').get_parameter_value().bool_value
        neighbors = [(1,0,1.0), (-1,0,1.0), (0,1,1.0), (0,-1,1.0)]
        if use_eight:
            d = math.sqrt(2.0)
            neighbors += [(1,1,d), (1,-1,d), (-1,1,d), (-1,-1,d)]

        # 4) Dijkstra data
        dist = np.full((H, W), np.inf, dtype=np.float64)
        visited = np.zeros((H, W), dtype=bool)
        parent_x = -np.ones((H, W), dtype=np.int32)
        parent_y = -np.ones((H, W), dtype=np.int32)

        dist[sy, sx] = 0.0
        pq: List[Tuple[float, Tuple[int, int]]] = []
        heapq.heappush(pq, (0.0, (sx, sy)))

        # 5) Main loop
        while pq:
            cost, (cx, cy) = heapq.heappop(pq)
            if visited[cy, cx]:
                continue
            visited[cy, cx] = True

            if (cx, cy) == (gx, gy):
                break  # reached goal with final (minimal) cost

            for dx, dy, step_cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < W and 0 <= ny < H):
                    continue
                if self._obstacles[ny, nx]:
                    continue

                new_cost = cost + step_cost
                if new_cost < dist[ny, nx]:
                    dist[ny, nx] = new_cost
                    parent_x[ny, nx] = cx
                    parent_y[ny, nx] = cy
                    heapq.heappush(pq, (new_cost, (nx, ny)))

        if not visited[gy, gx]:
            return None  # no path

        # 6) Reconstruct grid path (goal -> start)
        cells: List[Tuple[int, int]] = []
        cx, cy = gx, gy
        while cx != -1 and cy != -1:
            cells.append((cx, cy))
            px, py = parent_x[cy, cx], parent_y[cy, cx]
            if px == -1 and py == -1:
                break
            cx, cy = px, py
        cells.reverse()

        # 7) Convert to Path message (poses at cell centers, with simple heading)
        path = Path()
        path.header.frame_id = self.get_parameter('global_frame').get_parameter_value().string_value
        path.header.stamp = self.get_clock().now().to_msg()

        for i, (ix, iy) in enumerate(cells):
            x, y = self.map_to_world(ix, iy, x0, y0, res)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y

            # Heading: look toward next point (or keep last heading)
            if i + 1 < len(cells):
                nx, ny = self.map_to_world(cells[i+1][0], cells[i+1][1], x0, y0, res)
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)

        return path

    def world_to_map(self, x: float, y: float, x0: float, y0: float, res: float,
                    W: int, H: int) -> Optional[Tuple[int, int]]:
        """World (meters) -> grid (indices)."""
        ix = int(math.floor((x - x0) / res))
        iy = int(math.floor((y - y0) / res))
        if 0 <= ix < W and 0 <= iy < H:
            return ix, iy
        return None

    def map_to_world(self, ix: int, iy: int, x0: float, y0: float, res: float) -> Tuple[float, float]:
        """Grid (indices) -> world (meters), at cell center."""
        x = x0 + (ix + 0.5) * res
        y = y0 + (iy + 0.5) * res
        return x, y
    
     # ---------- Utilities ----------

    def inflate_obstacles(self, obstacles: np.ndarray, radius_m: float, res: float) -> np.ndarray:
        if radius_m <= 0.0:
            return obstacles
        radius_cells = max(1, int(math.ceil(radius_m / res)))
        H, W = obstacles.shape
        inflated = obstacles.copy()
        # Simple (not the fastest) disk dilation
        rr = radius_cells
        ys, xs = np.nonzero(obstacles)
        for y, x in zip(ys, xs):
            x0 = max(0, x - rr)
            x1 = min(W - 1, x + rr)
            y0 = max(0, y - rr)
            y1 = min(H - 1, y + rr)
            for ny in range(y0, y1 + 1):
                dy = ny - y
                for nx in range(x0, x1 + 1):
                    dx = nx - x
                    if dx*dx + dy*dy <= rr*rr:
                        inflated[ny, nx] = True
        return inflated
def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
