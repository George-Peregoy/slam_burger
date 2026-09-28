from path_planning import config
from path_planning.rrtsharp_c import RRTSharp, error
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import Bool
from path_planning.path_to_qr import path_to_qr
from path_planning.path_pruning_c import fit_to_qr
from path_planning.astar import Astar
from collections import deque
import numpy as np
import os

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        self.declare_parameter('qr_num', -1) # used for writing - matches world
        self.qr_num = self.get_parameter('qr_num').value

        self.publisher_ = self.create_publisher(Path, 'path', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.odom = None # inti odom 
        self.odom_x = None
        self.odom_y = None
        self.odom_theta = None

        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.map_data = None # init map data
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None

        self.path_msg = None  # current nav_msgs/Path being published

        self.at_end_sub = self.create_subscription(
            Bool, '/at_end', self.at_end_callback, 10)

        self.at_end = True  # trigger first sample on startup

        self.done = False
        self.saved = False

        self.world_goal = (config.GOAL[0] * config.WORLD_SCALE, config.GOAL[1] * config.WORLD_SCALE)
        
        self.visited_cells = set() # purely tracking metric
        self.visited_targets = set()

        self.is_sampling = False # don't sample while already sampling
        # if path invalid stop while replanning
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10) 


    # ------------------------------------------------------------------
    # Timer
    # ------------------------------------------------------------------

    def timer_callback(self):
        """
        If at goal save to qr else sample path.
        """

        if self.saved:
            return

        if self.done and self.at_end and not self.saved:
            self.get_logger().info("SAVING")

            # get final inflated map
            inflated_map_data = self._inflate_map(self.map_data, self.map_width, self.map_height, self.full_rad)
            inflated_map = self._build_occupancy_grid(inflated_map_data)
            path = self._get_path(inflated_map)

            # scale path up and round 
            new_path = [(int(i / config.WORLD_SCALE), int(j / config.WORLD_SCALE)) for i, j in path]

            # convert to string and save 
            path_str = fit_to_qr(path=new_path, map_obj=inflated_map, e=self.e,
                                  step_size=config.STEP_SIZE, char_limit=config.CHAR_LIMIT)
            self.get_logger().info(f"Saving path: {path}")
            self.get_logger().info(f"Path string: {path_str}")
            base_dir = os.path.abspath(os.path.dirname(__file__))
            root_dir = os.path.join(base_dir, '..', '..', '..', '..', '..', '..')
            qr_dir = os.path.join(root_dir, 'src', 'path_planning', 'qrcodes')
            path_to_qr(path=path_str, output_dir=qr_dir, env_number=self.qr_num)

            self.get_logger().info("DONE SAVING\n")
            self.saved = True
            return

        if self.at_end and not self.done:
            if self.is_sampling:
                return # skip sampling if already doing it
            self.at_end = False
            self.is_sampling = True
            self.get_logger().info("SAMPLING")
            self.path_msg = self.sample()
            self.get_logger().info("DONE SAMPLING\n")
            self.is_sampling = False
            
            if self.path_msg is None:
                self.at_end = True # keep sampling if nothing

        if self.path_msg and not self.at_end:
            if self._path_still_valid():
                self.publisher_.publish(self.path_msg)
            else:
                self.get_logger().info("PATH INVALIDATED — replanning\n")
                stop_msg = Twist()  # stop moving
                self.cmd_vel_publisher.publish(stop_msg)
                self.at_end = True  # trigger resample

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def odom_callback(self, odom_msg):
        """
        Stores odom

        Parameters
        ----------
        odom_msg : nav_msgs.msg.Odometry
            Odom message received.
        """
        self.odom = odom_msg
        self.odom_x     = self.odom.pose.pose.position.x
        self.odom_y     = self.odom.pose.pose.position.y
        self.odom_theta = self._quaternion_to_yaw(self.odom.pose.pose.orientation)

        if self.map_data is not None:
            row, col = self._world_to_grid(self.odom_x, self.odom_y)
            self.visited_cells.add((row, col)) # track cells visited 
            

    def at_end_callback(self, msg):
        """
        If at end of path, flip flag for resampling.

        msg : std_msgs.msg.Bool
            True if at end of Path else False
        """
        self.get_logger().info("IN AT_END CALLBACK")
        if msg.data and not self.at_end:
            self.get_logger().info("AT END — triggering resample\n")
            self.at_end = True

    def map_callback(self, msg):
        """
        Stores current map data, creates inflated maps for nav and frontier selection.

        Parameters
        ----------
        msg : nav_msgs.msg.OccupancyGrid
            Map object.
        """
        self.map_data = msg.data # store map fields
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        
        self.nav_rad  = int(np.floor(config.ROBOT_RADIUS / self.map_resolution)) # for A* path planning
        self.full_rad = int(np.ceil(config.ROBOT_RADIUS / self.map_resolution)) # used in frontier selection
        cost_rad = self.full_rad * 3  # used in cost gradient - A*

        # buffer maps 
        self.nav_inflated  = self._inflate_map(self.map_data, self.map_width, self.map_height, self.nav_rad) 
        self.full_inflated = self._inflate_map(self.map_data, self.map_width, self.map_height, self.full_rad)
        self.cost_map_data = self._make_cost_map(self.map_data, self.map_width, self.map_height, cost_rad)

        # build new nav_msgs.msg.OccupancyGrid with inflated data
        self.nav_map  = self._build_occupancy_grid(self.nav_inflated)   # collision checking in A*
        self.full_map = self._build_occupancy_grid(self.full_inflated)  # candidate filtering
        self.cost_map = self._build_occupancy_grid(self.cost_map_data)  # penalty only
        self.get_logger().info(f"nav_rad: {self.nav_rad}, full_rad: {self.full_rad}, resolution: {self.map_resolution:.4f}")

        # map debug stuff
        free    = sum(1 for c in self.map_data if c == 0)
        unknown = sum(1 for c in self.map_data if c == -1)
        self.get_logger().info(f"\nMap: {self.map_width}x{self.map_height} origin:({self.map_origin_x:.2f},{self.map_origin_y:.2f})")
        self.get_logger().info(f"Free: {free}  Unknown: {unknown}\n")

    # ------------------------------------------------------------------
    # Sampling
    # ------------------------------------------------------------------

    def sample(self):
        """
        Get frontiers, cluster, plan path to best frontier.

        Returns
        -------
        path : nav_msgs.msg.Path
            Path object.
        """
        if self.map_data is None:
            self.get_logger().warn("No map data\n")
            return None
        if self.odom is None:
            self.get_logger().warn("No odom\n")
            return None
        
        # get current distance from goal
        robot_to_goal = np.sqrt(
            (self.odom_x - self.world_goal[0])**2 +
            (self.odom_y - self.world_goal[1])**2
        )
        
        # if near goal DONE
        if robot_to_goal <= 0.3:
            self.get_logger().info("DONE\n")
            self.done    = True
            self.at_end  = True
            return None
        
        # check if goal is already mapped and reachable
        goal_row, goal_col = self._world_to_grid(self.world_goal[0], self.world_goal[1])
        goal_row = max(0, min(goal_row, self.map_height - 1))
        goal_col = max(0, min(goal_col, self.map_width - 1))
        goal_val = self.map_data[goal_row * self.map_width + goal_col]
        
        if goal_val == 0:  # goal is mapped and free
            path = self._plan_path(self.world_goal[0], self.world_goal[1], goal_row, goal_col)
            if path is not None and len(path) >= 2:
                self.get_logger().info("GOAL REACHABLE — planning directly")
                path = self._prune_path_los(path)
                return self._path_to_nav_path(path)
            

        self.get_logger().info(f"Getting candidates\n")
        candidates = self._get_candidates(self.full_inflated) # get frontiers

        if not candidates:
            self.get_logger().info("NO CANDIDATES\n")
            return None
        
        clusters = self._build_clusters(candidates) # cluster them

        if not clusters:
            self.get_logger().info("All clusters visited — clearing targets\n") # just in case
            self.visited_targets.clear() # clear visited if not done and no clusters
            clusters = self._build_clusters(candidates)
            self.get_logger().info(f"Num visited {len(self.visited_cells)}")
            self.get_logger().info(f"visited_targets size: {len(self.visited_targets)}")
            self.get_logger().info(f"Num clusters: {len(cluster)}\n")

        for cluster, score, best in clusters:
            if best in self.visited_targets:
                continue # if visited already ignore
            row, col = best
            x, y = self._grid_to_world(row, col) # get location of frontier
            path = self._plan_path(x, y, row, col) # use A* to plan path
            if path is None or len(path) < 2:
                continue
            self.get_logger().info(f"Path result: {len(path) if path is not None else None}")
            path = self._prune_path_los(path) # use line of sight pruning to minimize waypoints
            self.get_logger().info(f"Pruned path length: {len(path)}, first: {path[0]}, last: {path[-1]}\n")

            self.get_logger().info("SAMPLE FOUND")
            self.visited_targets.add((row, col)) # add best to visited
            return self._path_to_nav_path(path)

        return None

    # ------------------------------------------------------------------
    # Path helpers
    # ------------------------------------------------------------------

    def _path_to_nav_path(self, path: list):
        """
        Convert a list of (x, y) world-coordinate tuples to nav_msgs/Path.

        Parameters
        ----------
        path : list
            List of 2D points.

        Returns
        path : nav_msgs.msg.Path
            ROS path object.
        """
        nav_path = Path()
        nav_path.header.frame_id = 'map' # header stuff
        nav_path.header.stamp    = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(path): # for each point make PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp    = nav_path.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # heading toward next waypoint; last point keeps previous heading
            if i < len(path) - 1:
                nx, ny = path[i + 1]
                theta  = np.arctan2(ny - y, nx - x + 1e-8)
            else:
                px, py = path[i - 1]
                theta  = np.arctan2(y - py, x - px + 1e-8)

            q = self._yaw_to_quaternion(theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            nav_path.poses.append(pose)

        return nav_path

    def _get_path(self, map):
        """
        Final path planning using RRT# on filled out map. Used to create path for QR-Code.

        Parameters
        ----------
        map : nav_msgs.msg.OccupancyGrid
            Inflated map.

        Returns
        -------
        path : list
            List of 2D points for qr code.
        """
        error_matrix = np.zeros((int(config.ENV_X_BOUNDS[1] * config.WORLD_SCALE),
                                  int(config.ENV_Y_BOUNDS[1] * config.WORLD_SCALE)))
        e_env = 0.0
        e = error(e_env, error_matrix)

        start  = (config.START[0] * config.WORLD_SCALE, config.START[1] * config.WORLD_SCALE)
        bounds = ((config.BOUNDS[0][0] * config.WORLD_SCALE, config.BOUNDS[0][1] * config.WORLD_SCALE),
                  (config.BOUNDS[1][0] * config.WORLD_SCALE, config.BOUNDS[1][1] * config.WORLD_SCALE))

        rrt = RRTSharp(start=start, goal=self.world_goal, bounds=bounds,
                       map=map, e=e, step_size=1)
        path, nodes, self.e = rrt.rrt_sharp()
        return path

    def _prune_path_los(self, path: list):
        """
        Line of sight pruning.

        Parameters
        ----------
        path : list
            List of 2D points.
        
        Returns
        -------
        pruned : list
            List of 2D points pruned by line of sight
        """
        if len(path) < 2: # if two poitns return given
            return path

        pruned = [path[0]] # init with starting point
        i = 0

        while i < len(path) - 1:
            furthest = i + 1
            for j in range(i + 1, len(path)):
                if self._is_collision_free(
                    from_point=path[i],
                    to_point=path[j],
                    inflated_map=self.full_map
                ):
                    furthest = j
            pruned.append(path[furthest])
            i = furthest # furthest is next point to consider 

        return pruned
    
    def _path_still_valid(self):
        """
        Makes sure current path is valid with new map data.

        Returns
        -------
        True if path is free of collisions else False.
        """
        poses = self.path_msg.poses # called in sample so this is always defined
        for i in range(len(poses) - 1):
            p1 = (poses[i].pose.position.x, poses[i].pose.position.y)
            p2 = (poses[i+1].pose.position.x, poses[i+1].pose.position.y)
            if not self._is_collision_free(p1, p2, self.nav_map, explore=True):
                return False
        return True

    # ------------------------------------------------------------------
    # Candidate selection
    # ------------------------------------------------------------------

    def _get_candidates(self, inflated):
        """
        Find frontier cells on inlfated map.

        Parameters
        ----------
        inflated : list
            Inflated map data not map object.

        Returns
        -------
        candidates : list
            List with [x, y, row, col] for each candidate.
        """
        candidates = []

        for i, cell in enumerate(inflated):
            if cell != 0: # if object or unknown not frontier
                continue
            row = i // self.map_width
            col = i % self.map_width

            if (row, col) in self.visited_cells: # if visited not frontier
                continue

            if not self._is_frontier(row, col): # if not frontier not frontier
                continue

            x, y = self._grid_to_world(row, col) # get point
            
            world_x = x / config.WORLD_SCALE # make sure in world
            world_y = y / config.WORLD_SCALE
            if not (config.ENV_X_BOUNDS[0] <= world_x <= config.ENV_X_BOUNDS[1]):
                continue
            if not (config.ENV_Y_BOUNDS[0] <= world_y <= config.ENV_Y_BOUNDS[1]):
                continue
            candidates.append((x, y, row, col)) # append to candidates

        if candidates:
            self.get_logger().info(f"Num candidates: {len(candidates)}")
            return candidates

        self.get_logger().info("NO CANDIDATES\n")
        return []

    def _plan_path(self, goal_x, goal_y, row, col):
        """
        Uses A* to get to goal fron current position.

        Parameters
        ----------
        goal_x : float
            Goal x value in world scale.
        goal_y : float
            Goal y value in world scale.
        row : int
            Row in map.
        col : int

        Returns
        -------
        path : list
            List of 2D points from A* path planning.
        """
        start_row, start_col = self._world_to_grid(self.odom_x, self.odom_y)
        start_row = max(0, min(start_row, self.map_height - 1)) # get current pos
        start_col = max(0, min(start_col, self.map_width  - 1))

        raw_val = self.map_data[start_row * self.map_width + start_col]
        if raw_val == 100: # check to make sure not in obstacle
            self.get_logger().info("ROBOT IN OBSTACLE")
            return None

        sx, sy = self._grid_to_world(start_row, start_col)

        goal_row, goal_col = row, col # get goal pos
        goal_row = max(0, min(goal_row, self.map_height - 1))
        goal_col = max(0, min(goal_col, self.map_width  - 1))
        goal_val = self.full_inflated[goal_row * self.map_width + goal_col]

        goal_val = self.full_inflated[goal_row * self.map_width + goal_col]
        if goal_val != 0: # if goal isn't known don't plan
            self.get_logger().info(f"Goal cell not free in inflated ({goal_val}), skipping")
            return None
        
        astar = Astar((sx, sy), (goal_x, goal_y), self.nav_map, cost_map=self.cost_map, max_iter=10000)
        return astar.astar()

    # ------------------------------------------------------------------
    # Map utilities
    # ------------------------------------------------------------------

    def _build_clusters(self, candidates):
        """
        Cluster candidates to reduce frontier selection size.

        Parameters
        ----------
        candidates : list
            List of candidates.
        
        Returns
        -------
        clusters : list
            List of clusters [cluster, score, best_frontier].
        """

        frontier_set = {(row, col) for cx, cy, row, col in candidates}
        visited = set()
        clusters = []
        neighbors = [(i, j) for i in range(-1, 2) for j in range(-1, 2) if i !=0 or j != 0]

        for (row, col) in frontier_set: # for cell in frontiers
            if (row, col) in visited: # if visited continue
                continue
            queue = deque() # for BFS # else bfs adjecent frontiers
            cluster = [(row, col)]
            queue.append((row, col))
            visited.add((row, col)) # add visited
            while queue:
                q_row, q_col = queue.popleft() # get row, col
                for dr, dc in neighbors: # get surrounding cells
                    nr, nc = q_row + dr, q_col + dc
                    if (nr, nc) not in frontier_set:
                        continue # skip if not frontier
                    if (nr, nc) not in visited: 
                        visited.add((nr, nc))
                        cluster.append((nr, nc)) # if adjacent frontier add it cluster
                        queue.append((nr, nc)) # add to queue

            centroid_row = int(np.mean([r for r, c in cluster])) # get centroid pos
            centroid_col = int(np.mean([c for r, c in cluster]))

            # find closest frontier cell to centroid
            best = min(cluster, key=lambda rc: (rc[0]-centroid_row)**2 + (rc[1]-centroid_col)**2)

            size = len(cluster)
            x, y = self._grid_to_world(best[0], best[1])
            dist_to_goal = np.sqrt((self.world_goal[0]-x)**2 + (self.world_goal[1]-y)**2)
            dist_to_robot = np.sqrt((self.odom_x-x)**2 + (self.odom_y-y)**2)

            # score centroid
            alpha = 2.5 
            beta = 0.5
            gamma = 0.5
            score = alpha * dist_to_goal - beta * dist_to_robot - gamma * size
            clusters.append([cluster, score, best])

        clusters.sort(key=lambda c: c[1]) # sort clusters by score
        
        return clusters
                        
    def _build_occupancy_grid(self, data):
        """
        Build nav_msgs.msg.OccupancyGrid given map data.

        Returns
        -------
        nav_msgs.msg.OccupancyGrid
        """
        grid = OccupancyGrid()
        grid.data = data
        grid.info.width = self.map_width
        grid.info.height = self.map_height
        grid.info.resolution = self.map_resolution
        grid.info.origin.position.x = self.map_origin_x
        grid.info.origin.position.y = self.map_origin_y
        return grid

    def _is_frontier(self, row, col):
        """
        Checks adjacent cells and determines if frontier.

        Returns
        -------
        True if frontier else False.
        """
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = row + dr, col + dc # if cell has one unknown near it - frontier
            if 0 <= nr < self.map_height and 0 <= nc < self.map_width:
                if self.map_data[nr * self.map_width + nc] == -1:
                    return True
        return False

    def _inflate_map(self, map_data, width, height, radius):
        """
        Inflate map to account for robot size.

        Parameters
        ----------
        map_data : list
            List of grid values.
        width : int
            Map width.
        height : int
            Map height.
        radius : int
            Buffer radius.

        Returns
        -------
        inflated : list
            Inflated map data.
        """
        inflated = list(map_data)
        for i, cell in enumerate(map_data): # for eac cell
            if cell == 100: # if obstacle
                row = i // width
                col = i % width
                for dr in range(-radius, radius + 1): # make neighboring cells obstacle 
                    for dc in range(-radius, radius + 1):
                        if dr**2 + dc**2 <= radius**2:
                            nr, nc = row + dr, col + dc
                            if 0 <= nr < height and 0 <= nc < width:
                                inflated[nr * width + nc] = 100
        return inflated
    
    def _make_cost_map(self, map_data, width, height, radius, decay=0.4):
        """
        Make cost map for A*.

        Parameters
        ----------
        map_data : list
            List of grid values.
        width : int
            Map width.
        height : int
            Map height.
        radius : int
            Buffer radius.
        decay : float
            Degree of exponential decay.
        
        Returns
        -------
        cost : list
            Cost map data.
        """
        cost = [0] * len(map_data) # init map 

        for i, cell in enumerate(map_data):
            if cell != 100: # if obstacle make cost around
                continue
            row = i // width
            col = i % width

            for dr in range(-radius, radius + 1): # for each surroudning cell
                for dc in range(-radius, radius + 1):
                    dist = np.sqrt(dr**2 + dc**2)
                    if dist > radius:# if outside radius continue
                        continue
                    nr, nc = row + dr, col + dc
                    if not (0 <= nr < height and 0 <= nc < width): 
                        continue # if not in map continue
                    val = int(100 * np.exp(-decay * dist)) # add gradient layer
                    idx = nr * width + nc
                    if val > cost[idx]:
                        cost[idx] = val

        return cost

    def _is_collision_free(self, from_point, to_point, inflated_map, explore=False):
        """
        Uses Bresenham line algorithm to check for collisions.

        Parameters
        ----------
        from_point : tuple
            Starting point.
        to_point : tuple
            Goal point.
        inflated_map : nav_msgs.msg.OccupancyGrid
            Inflated map object.
        explore : bool
            If False consider unkown as obstacle.

        Returns
        -------
        True if collision else False.
        """
        x0, y0 = from_point
        x1, y1 = to_point
        dx = x1 - x0
        dy = y1 - y0
        dist  = np.sqrt(dx * dx + dy * dy)
        steps = int(dist / self.map_resolution) 

        for i in range(steps + 1): # go through line in steps
            t = i / steps if steps > 0 else 0
            x = x0 + t * dx
            y = y0 + t * dy
            row, col = self._world_to_grid(x, y) # get grid value
            if not (0 <= row < self.map_height and 0 <= col < self.map_width):
                return False
            cell_value = self.get_cell(row, col, inflated_map.data)
            if explore: # if obstacle collsion
                if cell_value == 100:
                    return False
            else:
                if cell_value == 100 or cell_value == -1:
                    return False
        return True

    def _world_to_grid(self, x, y):
        """
        Convert world values to grid cell.

        Parameters
        ----------
        x : float
            World x value.
        y : float
            World y value.

        Returns
        -------
        row : int 
            Row in grid.
        col : int
            Col in grid.
        """
        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)
        return row, col

    def _grid_to_world(self, row, col):
        """
        Gets world coordinate at cell location.

        Parameters
        ----------
        row : int 
            Row in grid.
        col : int
            Col in grid.
        Returns
        -------
        x : float
            World x value.
        y : float
            World y value.
        """
        x = col * self.map_resolution + self.map_origin_x
        y = row * self.map_resolution + self.map_origin_y
        return x, y

    def get_cell(self, row, col, map=None):
        """
        Gets cell value in grid.

        Parameters
        ----------
         row : int 
            Row in grid.
        col : int
            Col in grid.
        map : list
            Map data. Optional
        """
        if map is None:
            map = self.map_data
        return map[row * self.map_width + col]

    def _yaw_to_quaternion(self, yaw):
        """
        Converts yaw to quaternion.
        """
        return (0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0))

    def _quaternion_to_yaw(self, q):
        """
        Gets yaw from quaternion.
        """
        return np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()