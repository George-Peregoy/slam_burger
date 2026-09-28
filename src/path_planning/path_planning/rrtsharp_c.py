from path_planning.ellipses2 import Ellipse2, samp_ellipse
from path_planning import config
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import imageio
from shapely import Polygon, LineString
from math import sqrt, floor
import random
import time
import pickle
import os
import heapq

class Node:
    """
    Node class used in RRT#.

    Attributes
    ----------
    x : int
        X coord.
    y : int
        Y coord.
    parent : Node
        Parent node in graph.
    cost : float
        Distance from root score.
    lmc : float
        Local minimum cost.
    """
    
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = float('inf')  # g-value
        self.lmc = float('inf')   # locally minimum cost-to-come

    @property
    def state(self):
        return (self.x, self.y)

    def __lt__(self, other: "Node"):
        """
        For heapq: compare by min(lmc, cost)

        Parameters
        ----------
        other : Node
            Second node being compared.

        Returns
        -------
        True if self Node has smaller cost than other Node else False.
        """
        return min(self.lmc, self.cost) < min(other.lmc, other.cost)

class error:
    """
    Class to store error matrix.

    Attributes
    ----------
    e_env : float
        Ratio of bad samples per number of samples.
    e_matrix : list
        Creates grid of observation space and records collisions / num samples for each cell.
    num_samples : int
        Number of times RRT# has sampled the observation space.
    bad_samples : int
        Number of times sampling has failed.
    """
    
    def __init__(self, e_env, e_matrix):
        self.e_env = e_env
        self.e_matrix = e_matrix
        self.num_samples = 0
        self.bad_samples = 0

class RRTSharp:
    """
    Class to implement RRT#

    Attributes
    ----------
    start : tuple
        Start position. Ex (5,5).
    goal : tuple
        Goal position. Ex (45, 45).
    bounds : tuple. Ex ((0,50), (0,50)).
        Bounds of env using form ((x_min, x_max), (y_min, y_max))
    step_size : int
        How far to extend branches in steer. default=5
    time_limit : float
        Max time of RRT#. default=5
    map : nav_msgs.msg.OccupancyGrid
        Ros map object.
    nodes : list
        List of Nodes.
    best_goal : Node
        Best goal Node found.
    Ls : dict
        Dict with key time since best solution found and value cost of solution.
    e : error
        Error class to store e_env and e_matrix. default=None
    path : list
        List of coordinates. Ex [(5,5), (7,8)]
    ellipse : Ellipse2
        Used for sampling from ellipse. default=None.
    buffer_obstacles : list
        List of obstacles with vertice + buffer, used in collision detection for 2D objects.

    Methods
    -------
    rrt_sharp(start, goal, step_size, time_limit, obstacles, e, seed, ellipse)
        Uses RRT# to find path from start to goal. Stores path in attribute. 
    rrt_sharp_animate(env_path, save_dir, gif_name, data_name)
        Uses RRT# to find path from start to goal. Stores path in attribute. 
        Animates the process and stores in save dir.
    """
    
    def __init__(self, start: tuple, goal: tuple, bounds: tuple, step_size: int=5, time_limit: int=5, map: list=[], e: error=None, seed=None, ellipse: Ellipse2 | None=None):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.step_size = step_size
        self.time_limit = time_limit
        self.map = map # assume already inflated
        self.nodes = [self.start]
        self.start.cost = 0
        self.start.lmc = 0
        self.best_goal = None
        self.Ls = {}
        self.e = e
        self.path = []
        self.ellipse = ellipse
        if seed is not None:
            random.seed(seed)

        self.x_min = bounds[0][0]
        self.x_max = bounds[0][1]    
        self.y_min = bounds[1][0]    
        self.y_max = bounds[1][1]

        self.map_data = map.data  # flat list of cell values
        self.map_width = map.info.width
        self.map_height = map.info.height
        self.map_resolution = map.info.resolution
        self.map_origin_x = map.info.origin.position.x
        self.map_origin_y = map.info.origin.position.y

    def _euclidean_distance(self, node1: Node, node2: Node):
        """
        Computes euclidean distance

        Parameters
        ----------
        node1 : Node
            First Node.
        node2 : Node
            Second Node.

        Returns
        -------
        euclidean_distance : float.
        """
        return sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def _nearest_node(self, rnd_node: Node):
        """
        Finds nearest node in graph.

        Parameters
        ----------
        rnd_node : Node
            Randomly sampled Node.

        Returns
        -------
        nearest_node : Node
            Closest Node in graph. 
        """
        return min(self.nodes, key=lambda node: self._euclidean_distance(node, rnd_node))

    def _steer(self, from_node: Node, to_node: Node):
        """
        Creates potential branch from graph to sampled node of size self.step_size

        Parameters
        ----------
        from_node : Node
            Node in graph.
        to_node : Node
            Randomly sampled Node.
        
        Returns
        -------
        potential_node : Node
            Node located at end of potential branch.
        """
        dist = self._euclidean_distance(from_node, to_node)
        if dist < self.step_size:
            return Node(to_node.x, to_node.y)
        theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.step_size * np.cos(theta)
        new_y = from_node.y + self.step_size * np.sin(theta)
        return Node(new_x, new_y)

    def _get_nearby_nodes(self, new_node: Node):
        """
        Method to get nearby Nodes. Used in comparing local min cost.

        Parameters
        ----------
        new_node : Node
            Node added after steer.

        Returns
        -------
        nearby_nodes : list
            List of nearby Nodes.
        """
        n = len(self.nodes)
        d = 2  # for 2D
        gamma = 70  # suitable constant for your environment
        rn = min(gamma * (np.log(n) / n) ** (1/d), self.step_size)
        return [node for node in self.nodes if self._euclidean_distance(node, new_node) <= rn]

    def _is_collision_free(self, from_node: Node, to_node: Node):
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

        Returns
        -------
        True if collision else False.        
        """

        x0, y0 = from_node.x, from_node.y
        x1, y1 = to_node.x, to_node.y
        dx = x1 - x0
        dy = y1 - y0
        dist = np.sqrt(dx*dx + dy*dy)

        steps = int(dist / self.map_resolution)

        for i in range(steps + 1): # break line into many pieces
            t = i / steps if steps > 0 else 0
            x = x0 + t * dx
            y = y0 + t * dy

            # if any line is out of bounds or intersects ubstacle / unknown
            row, col = self._world_to_grid(x, y)
            if not 0 <= row < self.map_height or not 0 <= col < self.map_width:
                return False
            
            cell_value = self.get_cell(row, col)
            if cell_value == 100 or cell_value == -1:
                return False
            
        return True

    def _world_to_grid(self, x, y):

        # converts point from map to location in grid
        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)
        return row, col
    
    def _grid_to_world(self, row, col):
        # converts grid to location
        x = col * self.map_resolution + self.map_origin_x
        y = row * self.map_resolution + self.map_origin_y
        return x, y
    
    def get_cell(self, row, col):
        # gets cell data from grid
        return self.map_data[row * self.map_width + col]

    def _sample(self):
        """
        Samples from observation space. Can handle ellipse sampling.

        Returns
        -------
        sampled_node : Node
            Randomly sampled Node.
        """
        if self.ellipse is not None:
            # Sample from the ellipse using samp_ellipse
            x, y = samp_ellipse(self.ellipse, 1)[0] # returns list so grab first item
            return Node(x, y)
        else:
            # Uniform random sample in [0, 50] x [0, 50]
            return Node(random.uniform(self.x_min, self.x_max), random.uniform(self.y_min, self.y_max))

    def rrt_sharp(self):
        """
        Implements RRT# algorithm.

        Returns
        -------
        path : list
            List of best poses from start to goal.
        nodes : list
            List of all Nodes in graph.
        e : error
            Error class that stores e_env and e_matrix for graph.
        """
        start_time = time.time()
        t0 = 0
        ti = 0
        dt = 0.2
        goal_reached = False
        queue = []
        heapq.heapify(queue)

        def update_queue(node):
            if node not in queue:
                heapq.heappush(queue, node)

        while time.time() - start_time < self.time_limit:
            rnd_node = self._sample()
            self.e.num_samples += 1

            nearest = self._nearest_node(rnd_node)
            new_node = self._steer(nearest, rnd_node)

            if self._is_collision_free(nearest, new_node):
                # Find best parent (lowest lmc)
                nearby_nodes = self._get_nearby_nodes(new_node)
                min_lmc = nearest.cost + self._euclidean_distance(nearest, new_node)
                best_parent = nearest
                for node in nearby_nodes:
                    if self._is_collision_free(node, new_node):
                        cost = node.cost + self._euclidean_distance(node, new_node)
                        if cost < min_lmc:
                            min_lmc = cost
                            best_parent = node
                new_node.parent = best_parent
                new_node.lmc = min_lmc
                new_node.cost = float('inf')
                self.nodes.append(new_node)

                # Update lmc/cost for neighbors (rewiring)
                for node in nearby_nodes:
                    if node == best_parent:
                        continue
                    if self._is_collision_free(new_node, node):
                        cost_through_new = new_node.lmc + self._euclidean_distance(new_node, node)
                        if cost_through_new < node.lmc:
                            node.lmc = cost_through_new
                            node.parent = new_node
                            update_queue(node)

                # Cost propagation 
                update_queue(new_node)
                MAX_PROP_STEPS = 1000
                prop_steps = 0
                visited = set()
                while queue and prop_steps < MAX_PROP_STEPS:
                    u = heapq.heappop(queue)
                    if u in visited:
                        continue
                    visited.add(u)
                    if u.cost > u.lmc:
                        u.cost = u.lmc
                        for child in self.nodes:
                            if child.parent == u:
                                cost = u.cost + self._euclidean_distance(u, child)
                                if cost < child.lmc:
                                    child.lmc = cost
                                    update_queue(child)
                    else:
                        u.cost = float('inf')
                        for child in self.nodes:
                            if child.parent == u:
                                cost = u.cost + self._euclidean_distance(u, child)
                                if cost < child.lmc:
                                    child.lmc = cost
                                    update_queue(child)
                    prop_steps += 1

                # Check if the goal can be connected to the tree
                if self._euclidean_distance(new_node, self.goal) < self.step_size and self._is_collision_free(new_node, self.goal):
                    if self.best_goal is None or new_node.lmc + self._euclidean_distance(new_node, self.goal) < self.best_goal.lmc:
                        self.best_goal = Node(self.goal.x, self.goal.y)
                        self.best_goal.parent = new_node
                        self.best_goal.lmc = new_node.lmc + self._euclidean_distance(new_node, self.goal)
                        self.best_goal.cost = self.best_goal.lmc
                        if not goal_reached:
                            # print("Solution found!")
                            goal_reached = True
                            t0 = time.time()
                            self.Ls[0] = self.best_goal.cost

                if self.best_goal is not None:
                    t_curr = time.time() - t0
                    if t_curr > ti + dt:
                        ti = ti + dt
                        self.Ls[ti] = self.best_goal.cost

            else:
                # Handle new node collision
                tempx = floor(new_node.x + 0.5)
                tempy = floor(new_node.y + 0.5)
                if tempx >= 50: tempx -= 1
                if tempy >= 50: tempy -= 1
                try:
                    self.e.e_matrix[tempx][tempy] += 1
                    self.e.bad_samples += 1
                except IndexError:
                    print("Index out of range\n")
                    print(tempx, tempy)

        # If the goal was never reached, set solution_time to a large value
        if not goal_reached:
            self.soln_time = float('inf')
        else:
            self.soln_time = t0 - start_time
            print(f"Solution time: {self.soln_time}\n")

        # Finalize the best path found
        if self.best_goal:
            self.nodes.append(self.best_goal)
            self.goal = self.best_goal

        # Extract path
        self.path = self._extract_path()

        # Compute/compile errors
        self.e.e_matrix = [[cell / self.e.num_samples for cell in row] for row in self.e.e_matrix]
        self.e.e_env = self.e.bad_samples / self.e.num_samples

        return self.path, self.nodes, self.e

    def _extract_path(self):
        """
        Extracts best path from list of nodes.

        Returns
        -------
        path : list
            List of best poses from start to goal.
        """
        path = []
        if self.best_goal:
            node = self.best_goal
        else:
            node = min(self.nodes, key=lambda n: self._euclidean_distance(n, self.goal))
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]

    def plot_path(self, path=None):
        """
        Plots path, obstacles, and ellipse if ellipse is not None.

        Parameters
        ----------
        path : list
            List of poses. default=None.
        """
        plt.figure(figsize=(8, 8))
        # Use self.nodes and self.path if not provided
        nodes = self.nodes
        if path is None:
            path = self.path
            
        for node in nodes:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'k-', linewidth=0.5)
                
        if path and len(path) > 1:
            px, py = zip(*path)
            plt.plot(px, py, '-r', linewidth=2, label='RRT# Path')
            plt.scatter(px, py, color='red', marker='o')
            
        plt.plot(self.start.x, self.start.y, 'go', markersize=10, label='Start')
        plt.plot(self.goal.x, self.goal.y, 'bo', markersize=10, label='Goal')
        
        for obs in self.obstacles:
            x, y = Polygon(obs).exterior.xy
            plt.fill(x, y, facecolor='gray', edgecolor='black')
            
        # Plot ellipse if defined
        if self.ellipse is not None:
            
            xc, yc = self.ellipse.h, self.ellipse.k
            a = self.ellipse.a
            b = self.ellipse.b
            angle = self.ellipse.tilt if hasattr(self.ellipse, 'tilt') else 0
            ell_patch = Ellipse((xc, yc), 2*a, 2*b, angle=angle, edgecolor='blue', facecolor='none', linewidth=2, label='Ellipse')
            plt.gca().add_patch(ell_patch)
            
        plt.xlim(0, 50)
        plt.ylim(0, 50)
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.legend(loc='lower right')
        plt.show()

    def rrt_sharp_animate(self, env_path, save_dir='./animations/', gif_name='rrtsharp_anim.gif', data_name='rrt_data_anim.pickle'):
        """
        Alternate RRT# that builds an animation as the algorithm runs.
        Loads obstacles from env_path. No ellipse sampling.
        Saves animation GIF and path/nodes data to save_dir.

        Parameters
        ----------
        env : str
            path where envs are stored
        save_dir : str
            Path to save dir. default='./animations/'.
        gif_name : str
            Name of animation being saved. default='rrtsharp_anim.gif'.
        data_name : str
            Name of data being saved as pickle file. default='rrt_data_anim.pickle'
        """

        # Load obstacles from file
        with open(env_path, 'rb') as f:
            obstacles = pickle.load(f)
        obstacles = [np.array(poly) for poly in obstacles]
        self.obstacles = [Polygon(obstacle) for obstacle in obstacles]

        # Use uniform sampling (no ellipse)
        self.ellipse = None

        # Reset nodes
        self.nodes = [self.start]
        self.start.cost = 0
        self.start.lmc = 0
        self.best_goal = None
        self.Ls = {}
        self.path = []

        # Animation setup
        frames = []
        os.makedirs(save_dir, exist_ok=True)
        gif_path = os.path.join(save_dir, gif_name)
        data_path = os.path.join(save_dir, data_name)

        start_time = time.time()
        t0 = 0
        ti = 0
        dt = 0.2
        goal_reached = False
        queue = []
        heapq.heapify(queue)

        def update_queue(node):
            if node not in queue:
                heapq.heappush(queue, node)

        tree_edges = []

        while time.time() - start_time < self.time_limit:
            rnd_node = self._sample()
            self.e.num_samples += 1

            nearest = self._nearest_node(rnd_node)
            new_node = self._steer(nearest, rnd_node)

            if self._is_collision_free(nearest, new_node):
                # Find best parent (lowest lmc)
                nearby_nodes = self._get_nearby_nodes(new_node)
                min_lmc = nearest.cost + self._euclidean_distance(nearest, new_node)
                best_parent = nearest
                for node in nearby_nodes:
                    if self._is_collision_free(node, new_node):
                        cost = node.cost + self._euclidean_distance(node, new_node)
                        if cost < min_lmc:
                            min_lmc = cost
                            best_parent = node
                new_node.parent = best_parent
                new_node.lmc = min_lmc
                new_node.cost = float('inf')
                self.nodes.append(new_node)
                if new_node.parent:
                    tree_edges.append((new_node, new_node.parent))

                # Update lmc/cost for neighbors (rewiring)
                for node in nearby_nodes:
                    if node == best_parent:
                        continue
                    if self._is_collision_free(new_node, node):
                        cost_through_new = new_node.lmc + self._euclidean_distance(new_node, node)
                        if cost_through_new < node.lmc:
                            node.lmc = cost_through_new
                            node.parent = new_node
                            update_queue(node)

                # Cost propagation (LPA*-like)
                update_queue(new_node)
                MAX_PROP_STEPS = 1000
                prop_steps = 0
                visited = set()
                while queue and prop_steps < MAX_PROP_STEPS:
                    u = heapq.heappop(queue)
                    if u in visited:
                        continue
                    visited.add(u)
                    if u.cost > u.lmc:
                        u.cost = u.lmc
                        for child in self.nodes:
                            if child.parent == u:
                                cost = u.cost + self._euclidean_distance(u, child)
                                if cost < child.lmc:
                                    child.lmc = cost
                                    update_queue(child)
                    else:
                        u.cost = float('inf')
                        for child in self.nodes:
                            if child.parent == u:
                                cost = u.cost + self._euclidean_distance(u, child)
                                if cost < child.lmc:
                                    child.lmc = cost
                                    update_queue(child)
                    prop_steps += 1

                # Check if the goal can be connected to the tree
                if self._euclidean_distance(new_node, self.goal) < self.step_size and self._is_collision_free(new_node, self.goal):
                    if self.best_goal is None or new_node.lmc + self._euclidean_distance(new_node, self.goal) < self.best_goal.lmc:
                        self.best_goal = Node(self.goal.x, self.goal.y)
                        self.best_goal.parent = new_node
                        self.best_goal.lmc = new_node.lmc + self._euclidean_distance(new_node, self.goal)
                        self.best_goal.cost = self.best_goal.lmc
                        if not goal_reached:
                            goal_reached = True
                            t0 = time.time()
                            self.Ls[0] = self.best_goal.cost

                if self.best_goal is not None:
                    t_curr = time.time() - t0
                    if t_curr > ti + dt:
                        ti = ti + dt
                        self.Ls[ti] = self.best_goal.cost

            else:
                # Handle new node collision
                tempx = floor(new_node.x + 0.5)
                tempy = floor(new_node.y + 0.5)
                if tempx >= 50: tempx -= 1
                if tempy >= 50: tempy -= 1
                try:
                    self.e.e_matrix[tempx][tempy] += 1
                    self.e.bad_samples += 1
                except IndexError:
                    print("Index out of range\n")
                    print(tempx, tempy)

            # --- Animation frame ---
            fig, ax = plt.subplots(figsize=(6, 6))
            for obs in obstacles:
                x, y = zip(*obs)
                ax.fill(x, y, 'gray', alpha=0.5)
            for edge in tree_edges:
                ax.plot([edge[0].x, edge[1].x], [edge[0].y, edge[1].y], 'k-', linewidth=0.5)
            ax.plot(self.start.x, self.start.y, 'go', markersize=8)
            ax.plot(self.goal.x, self.goal.y, 'bo', markersize=8)
            ax.set_xlim(0, 50)
            ax.set_ylim(0, 50)
            ax.set_title('RRT# Tree Growth')
            fig.tight_layout()
            fig.canvas.draw()
            image = np.frombuffer(fig.canvas.buffer_rgba(), dtype='uint8')
            image = image.reshape(fig.canvas.get_width_height()[::-1] + (4,))
            frames.append(image)
            plt.close(fig)

        # If the goal was never reached, set solution_time to a large value
        if not goal_reached:
            self.soln_time = float('inf')
        else:
            self.soln_time = t0 - start_time
            print("Solution time:", self.soln_time)

        # Finalize the best path found
        if self.best_goal:
            self.nodes.append(self.best_goal)
            self.goal = self.best_goal

        # Extract path
        self.path = self._extract_path()

        # Compute/compile errors
        self.e.e_matrix = [[cell / self.e.num_samples for cell in row] for row in self.e.e_matrix]
        self.e.e_env = self.e.bad_samples / self.e.num_samples

        # --- Final frame: plot path ---
        fig, ax = plt.subplots(figsize=(6, 6))
        for obs in obstacles:
            x, y = zip(*obs)
            ax.fill(x, y, 'gray', alpha=0.5)
        for node in self.nodes:
            if node.parent:
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'k-', linewidth=0.5)
        if self.path and len(self.path) > 1:
            px, py = zip(*self.path)
            ax.plot(px, py, '-r', linewidth=2, label='RRT# Path')
            ax.scatter(px, py, color='red', marker='o')
        ax.plot(self.start.x, self.start.y, 'go', markersize=8)
        ax.plot(self.goal.x, self.goal.y, 'bo', markersize=8)
        ax.set_xlim(0, 50)
        ax.set_ylim(0, 50)
        ax.set_title('RRT# Final Path')
        fig.tight_layout()
        fig.canvas.draw()
        image = np.frombuffer(fig.canvas.buffer_rgba(), dtype='uint8')
        image = image.reshape(fig.canvas.get_width_height()[::-1] + (4,))
        frames.append(image)
        plt.close(fig)

        # Save GIF
        imageio.mimsave(gif_path, frames, duration=1)
        print(f"RRT# animation saved to {gif_path}")

        # Save RRT data for further use
        rrt_data = {
            'path': self.path,
            'nodes': [(n.x, n.y, n.parent.state if n.parent else None) for n in self.nodes],
            'obstacles': obstacles,
            'e': self.e,
            'step_size': self.step_size
        }
        with open(data_path, 'wb') as f:
            pickle.dump(rrt_data, f)
        print(f"RRT# data saved to {data_path}")
    
def main():
    
    # --- CONFIGURATION ---
    ENV_NUM = 0
    base_dir = os.path.abspath(os.path.dirname(__file__)) 
    env_dir = os.path.join(base_dir, '..', 'environments', f'environment_polygon_{ENV_NUM}.pickle')
    with open(env_dir, 'rb') as f:
        obstacles = pickle.load(f)
    obstacles = [np.array(poly) for poly in obstacles]
    
    start = config.START
    goal = config.GOAL
    bounds = config.BOUNDS
    
    # obstacle_vertices = [
    #     [[15, 30], [15, 35], [35, 35], [35, 30]],
    #     [[30, 20], [30, 25], [40, 25], [40, 20]],
    #     [[15, 10], [15, 25], [25, 25]],
    #     [[25, 12], [25, 22], [30, 12]]
    # ]
    error_matrix = [[0 for _ in range(50)] for _ in range(50)]
    e_env = 0.0
    e = error(e_env, error_matrix)
    
    # If using an ellipse:
    # s = sqrt((start.x - goal.x)**2 + (start.y - goal.y)**2)
    # ellipse = Ellipse2((start.x, start.y), (goal.x, goal.y), s)
    rrt = RRTSharp(start, goal, bounds, obstacles=obstacles, e=e, time_limit=5, step_size=6, seed=71)

    # rrt.rrt_sharp_animate(env_path=ENV_PATH, save_dir='./animations/', gif_name=f'rrtsharp_env{ENV_NUM}.gif', data_name=f'rrt_data_env{ENV_NUM}.pickle')
    path, nodes, e = rrt.rrt_sharp()
    rrt.plot_path(path)

# Usage in __main__:
if __name__ == "__main__":
    main()