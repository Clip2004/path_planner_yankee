import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import tf2_ros
import tf_transformations
import numpy as np
import heapq

class AStarNode(Node):
    def __init__(self):
        super().__init__('a_star_node')
        # --- add these params at __init__ ---
        self.occ_thresh = self.declare_parameter('occ_thresh', 100).value
        self.robot_radius_m = self.declare_parameter('robot_radius_m', 0.21213203435).value
        self.safety_margin_m = self.declare_parameter('safety_margin_m', 0.1).value

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseStamped, '/goal_waypoint', self.goal_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage
        self.map = None
        self.goal = None
        print('A* Node Initialized')
    def map_callback(self, msg):
        self.map = msg
        
        # 2) compute how many cells to inflate
        res = self.map.info.resolution
        inflation_cells = max(0, int(np.floor((self.robot_radius_m + self.safety_margin_m) / res)))

        # 3) make mask of cells to inflate (True = will become occupied)
        inflated_mask = self.make_inflation_mask(self.map, inflation_cells, self.occ_thresh)

        # 4) write 100s into the map data (hard inflation)
        w, h = self.map.info.width, self.map.info.height
        data = np.array(self.map.data, dtype=np.int16).reshape((h, w))
        data[inflated_mask] = 100  # or 254, anything >= occ_thresh
        self.map.data = data.flatten().tolist()
        
        print('Map received')
    def make_inflation_mask(self, occ_grid, inflation_cells: int, occ_thresh: int):
        """Return boolean mask (h,w): True where cells are within `inflation_cells` of any obstacle."""
        w, h = occ_grid.info.width, occ_grid.info.height
        data = np.array(occ_grid.data, dtype=np.int16).reshape((h, w))
        occ = (data >= occ_thresh).astype(np.uint8)

        # Multi-source BFS (brushfire) distance in cells
        INF = 1_000_000
        dist = np.full((h, w), INF, dtype=np.int32)

        from collections import deque
        q = deque()
        ys, xs = np.where(occ == 1)
        for y, x in zip(ys, xs):
            dist[y, x] = 0
            q.append((x, y))

        # 4-connected neighbors for distance
        for_yx = [(-1,0),(1,0),(0,-1),(0,1)]
        while q:
            x, y = q.popleft()
            d = dist[y, x]
            for dx, dy in for_yx:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and dist[ny, nx] > d + 1:
                    dist[ny, nx] = d + 1
                    q.append((nx, ny))

        # Inflate: cells with distance <= inflation_cells become occupied
        return dist <= inflation_cells
    def goal_callback(self, msg):
        self.goal = msg
        print('Goal received')
        self.plan_path()
    def plan_path(self):
        if not self.map or not self.goal:
            return

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return

        # Convert start and goal to grid coordinates
        start = self.world_to_grid(tf.transform.translation)
        goal = self.world_to_grid(self.goal.pose.position)

        path = self.a_star(start, goal)

        if not path or len(path) < 2:
            self.get_logger().warn("A* did not find a valid path!")
            return  # No publicar path vacío

        ros_path = Path()
        ros_path.header.frame_id = 'map'
        for (x, y) in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x * self.map.info.resolution + self.map.info.origin.position.x
            pose.pose.position.y = y * self.map.info.resolution + self.map.info.origin.position.y
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)
        self.get_logger().info("Path published")
    def world_to_grid(self, pos):
        mx = int((pos.x - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((pos.y - self.map.info.origin.position.y) / self.map.info.resolution)
        return (mx, my)
    def a_star(self, start, goal):
        # A* pathfinding algorithm implementation
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        if data[start[1]][start[0]] >= self.occ_thresh or data[goal[1]][goal[0]] >= self.occ_thresh:
            self.get_logger().warn("Start or goal is in an occupied cell!")
            return

        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def heuristic(self, a, b):
        # Manhattan distance heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]
    def get_neighbors(self, node):
        neighbors = []
        x, y = node
        width = self.map.info.width
        height = self.map.info.height
        data = np.array(self.map.data).reshape((height, width))

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and data[ny][nx] < self.occ_thresh:
                neighbors.append((nx, ny))
        return neighbors
def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    rclpy.spin(node)
    rclpy.shutdown()
