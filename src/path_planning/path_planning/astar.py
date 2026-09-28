import numpy as np
import heapq

class Node:
    """
    General node class for A*.
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.f = np.inf # total cost
        self.g = np.inf # distance from start
        self.h = 0 # distance from goal

    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class Astar:
    """
    Astar from https://www.geeksforgeeks.org/dsa/a-search-algorithm/.
    """
     
    def __init__(self, start, goal, map, cost_map=None, max_iter=None):
        
        self.start_node = Node(start[0], start[1])
        self.goal_node = Node(goal[0], goal[1])
        
        self.map_data = map.data
        self.map_width = map.info.width
        self.map_height = map.info.height
        self.map_resolution = map.info.resolution
        self.map_origin_x = map.info.origin.position.x
        self.map_origin_y = map.info.origin.position.y

        self.cost_map_data = cost_map.data if cost_map is not None else None
        self.max_iter = max_iter if max_iter is not None else self.map_width * self.map_height

    def astar(self):
        """
        Implements A* path planning

        Returns
        -------
        path : list
            List of 2D points.
        """

        iter_count = 0

        open_list = [] # queue
        closed_list = set() # visited

        self.start_node.f = 0
        self.start_node.g = 0

        start_rc = self._world_to_grid(self.start_node.x, self.start_node.y)
        goal_rc = self._world_to_grid(self.goal_node.x, self.goal_node.y)

        if start_rc == goal_rc:
            return [(self.start_node.x, self.start_node.y)]

        heapq.heappush(open_list, self.start_node)

        neighbors = [(i, j) for i in range(-1, 2) for j in range(-1, 2) if (i != 0 or j != 0)]

        while len(open_list) > 0 and iter_count < self.max_iter:
            
            iter_count += 1
            q = heapq.heappop(open_list)

            q_row, q_col = self._world_to_grid(q.x, q.y)

            if (q_row, q_col) in closed_list: # if visited ignore
                continue

            closed_list.add((q_row, q_col))

            for dr, dc in neighbors:
                
                nr, nc = q_row + dr, q_col + dc

                new_x, new_y = self._grid_to_world(nr, nc)
                new_node = Node(new_x, new_y)
                new_node.parent = q

                if self._is_collision_free(q, new_node, dr, dc) and (nr, nc) not in closed_list:

                    if (nr, nc) == goal_rc:
                        self.goal_node.parent = q
                        return self.get_path(self.goal_node)
                    

                    move_cost = self._euclidean_distance(new_node, q)
                    if self.cost_map_data is not None:
                        cost_val = self.cost_map_data[nr * self.map_width + nc]
                        if cost_val > 0:
                            weight = 10
                            move_cost += weight * (cost_val / 100.0)
                        

                    # penalize proximity to walls using cost map
                    if self.cost_map_data is not None:
                        cost_val = self.cost_map_data[nr * self.map_width + nc]
                        if cost_val == 100:
                            move_cost += 2.0

                    new_node.g = q.g + move_cost
                    new_node.h = self._euclidean_distance(new_node, self.goal_node)
                    new_node.f = new_node.g + new_node.h

                    existing = next((n for n in open_list if self._world_to_grid(n.x, n.y) == (nr, nc)), None)

                    if existing is None:
                        heapq.heappush(open_list, new_node)
                    elif new_node.f < existing.f:
                        existing.f = new_node.f
                        existing.g = new_node.g
                        existing.parent = q
                        heapq.heappush(open_list, new_node)

        return None

    def get_path(self, goal_node: Node):
        """
        Walks back from goal building path.

        Parameters
        ----------
        goal_node : Node
            Final node in path.

        Returns
        -------
        path : list
            List of 2D points.
        """
        
        node = goal_node
        path = []
        while node.parent is not None:
            path.append((node.x, node.y))
            node = node.parent

        path.append((node.x, node.y))

        return path[::-1]

    def _is_collision_free(self, from_node, to_node, dr, dc):
        """
        Checks cell value of other cell and diagonals. If collide return False else True.
        """
        row, col = self._world_to_grid(to_node.x, to_node.y)
        from_row, from_col = self._world_to_grid(from_node.x, from_node.y)
        
        if not 0 <= row < self.map_height or not 0 <= col < self.map_width:
            return False
        
        if self.get_cell(row, col) == 100:
            return False
        
        if dr != 0 and dc != 0:
            if self.get_cell(from_row + dr, from_col) == 100:
                return False
            if self.get_cell(from_row, from_col + dc) == 100:
                return False
        
        return True

    def _euclidean_distance(self, node1: Node, node2: Node):
        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def _world_to_grid(self, x, y):
        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)
        return row, col
    
    def _grid_to_world(self, row, col):
        x = col * self.map_resolution + self.map_origin_x
        y = row * self.map_resolution + self.map_origin_y
        return x, y
    
    def get_cell(self, row, col, map=None):
        if map is None:
            map = self.map_data
        return map[row * self.map_width + col]

if __name__ == "__main__":

    def make_map(grid, width, height, resolution=0.05, origin_x=0.0, origin_y=0.0):
        from nav_msgs.msg import OccupancyGrid
        msg = OccupancyGrid()
        msg.info.width = width
        msg.info.height = height
        msg.info.resolution = resolution
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.data = [cell for row in grid for cell in row]
        return msg

    # Test 1: open field
    grid = [[0]*5 for _ in range(5)]
    m = make_map(grid, 5, 5)
    a = Astar((0.025, 0.025), (0.225, 0.225), m)
    path = a.astar()
    assert path is not None, "Test 1 failed: open field should find path"
    print(f"Test 1 passed: {path}")

    # Test 2: vertical wall at col 2 rows 0-3, path must go around via row 4
    grid = [[0]*5 for _ in range(5)]
    for r in range(4):
        grid[r][2] = 100
    m = make_map(grid, 5, 5)
    a = Astar((0.025, 0.025), (0.225, 0.025), m)
    path = a.astar()
    assert path is not None, "Test 2 failed: should find path around wall"
    for x, y in path:
        r, c = a._world_to_grid(x, y)
        assert a.get_cell(r, c) != 100, f"Test 2 failed: path hits wall at ({x},{y})"
    print(f"Test 2 passed: {path}")

    # Test 3: diagonal corner cutting
    grid = [[0]*5 for _ in range(5)]
    grid[1][1] = 100
    m = make_map(grid, 5, 5)
    a = Astar((0.025, 0.025), (0.025, 0.125), m)
    path = a.astar()
    assert path is not None, "Test 3 failed: should find path"
    for x, y in path:
        r, c = a._world_to_grid(x, y)
        assert a.get_cell(r, c) != 100, f"Test 3 failed: path hits obstacle at ({x},{y})"
    print(f"Test 3 passed: {path}")

    # Test 4: completely blocked
    grid = [[0]*5 for _ in range(5)]
    for r in range(5):
        grid[r][2] = 100
    m = make_map(grid, 5, 5)
    a = Astar((0.025, 0.025), (0.225, 0.025), m)
    path = a.astar()
    assert path is None, "Test 4 failed: should return None"
    print("Test 4 passed: no path correctly returned None")

    # Test 5: start == goal
    grid = [[0]*5 for _ in range(5)]
    m = make_map(grid, 5, 5)
    a = Astar((0.025, 0.025), (0.025, 0.025), m)
    path = a.astar()
    assert path is not None and len(path) >= 1, "Test 5 failed: start==goal should return immediately"
    print(f"Test 5 passed: {path}")

    # Test 6: cost map penalizes wall proximity
    grid = [[0]*5 for _ in range(5)]
    cost_grid = [[0]*5 for _ in range(5)]
    cost_grid[0][3] = 100  # penalty near wall
    m = make_map(grid, 5, 5)
    cost_m = make_map(cost_grid, 5, 5)
    a = Astar((0.025, 0.025), (0.225, 0.025), m, cost_map=cost_m)
    path = a.astar()
    assert path is not None, "Test 6 failed: should find path avoiding cost zone"
    print(f"Test 6 passed: {path}")

    print("\nAll tests passed.")