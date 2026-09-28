from path_planning import config
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from scipy.spatial import ConvexHull
import os
import pickle

def generate_obstacles(num_obstacles: int, 
                       min_sides: int,
                       max_sides: int, 
                       x_bounds: tuple, 
                       y_bounds: tuple, 
                       max_area_ratio: float, 
                       max_side_length: int):
    """
    Generates list of obstacles. Obstacles are list of non collinear vertices .

    Parameters
    ----------
    num_obstacles : int
        Number of obstacles in env.
    min_sides : int
        Min sides of obstacle.
    max_sides : int
        Max sides of obstacle.
    x_bounds : tuple
        X bounds of env.
    y_bounds : tuple
        Y bounds of env.
    max_area_ratio : float
        Max ratio of obstacle area to environment area.
    max_side_length : int
        Max length of obstacle side.
    
    Returns
    -------
    obstacles : list
        List of obstacles.
    """
    obstacles = []
    for _ in range(num_obstacles):
        while True:
            num_sides = np.random.randint(min_sides, max_sides + 1) # plus 1 since exclusive.
            obstacle = []
            for _ in range(num_sides):
                x = np.random.randint(x_bounds[0], x_bounds[1])
                y = np.random.randint(y_bounds[0], y_bounds[1])
                obstacle.append((x, y))
            obstacle = np.array(obstacle)
            if len(obstacle) < 3:
                continue
            if not _is_non_collinear(obstacle):
                continue
            hull = ConvexHull(obstacle)
            obstacle = obstacle[hull.vertices]
            area = 0.5 * np.abs(np.dot(obstacle[:,0], np.roll(obstacle[:,1], 1)) - np.dot(obstacle[:,1], np.roll(obstacle[:,0], 1)))
            area_ratio = area / ((x_bounds[1] - x_bounds[0]) * (y_bounds[1] - y_bounds[0]))
            if area_ratio < max_area_ratio:
                max_side_length_found = False
                for i in range(len(obstacle)):
                    x1, y1 = obstacle[i]
                    x2, y2 = obstacle[(i+1)%len(obstacle)]
                    side_length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                    if side_length > max_side_length:
                        max_side_length_found = True
                        break
                if not max_side_length_found:
                    obstacles.append(obstacle.tolist())
                    break
    return obstacles

def _is_non_collinear(points):
    """
    Checks if points are on same line.

    Parameters
    ----------
    points : list
        List of vertices of obstacle. 

    Returns
    -------
    bool
        True if points on same line else False.
    """
    if len(points) < 3:
        return False
    x0, y0 = points[0]
    x1, y1 = points[1]
    for x, y in points[2:]:
        if (x - x0) * (y1 - y0) != (x1 - x0) * (y - y0):
            return True
    return False

def save_environment(obstacles: list, environment_number: int):
    """
    Save environment with name enironment_polygon_EnvironmentNumber.pickle
    to Environment dir.

    Parameters
    ----------
    obstacles : list
        List of obstacles.
    environment_number : int
        Number env is being saved as.

    Returns
    -------
    None
    """
    base_dir = os.path.abspath(os.path.dirname(__file__)) # abs path to file
    env_dir = os.path.join(base_dir, '..', 'environments') # rel path to env dir

    if not os.path.exists(env_dir):
        os.makedirs(env_dir)
    
    filename = f'environment_polygon_{environment_number}.pickle'
    filepath = os.path.join(env_dir, filename)
    
    with open(filepath, 'wb') as f:
        pickle.dump(obstacles, f)

    return

def main():
    num_environments = 1
    num_obstacles = 8
    min_sides = 3
    max_sides = 4
    x_bounds = config.ENV_X_BOUNDS
    y_bounds = config.ENV_Y_BOUNDS
    max_area_ratio = 0.15
    max_side_length = 20

    for i in range(num_environments):
        obstacles = generate_obstacles(num_obstacles, min_sides, max_sides, x_bounds, y_bounds, max_area_ratio, max_side_length)
        save_environment(obstacles, i)
        
        # Optional: Plot the obstacles
        fig, ax = plt.subplots()
        for obstacle in obstacles:
            polygon = Polygon(np.array(obstacle), facecolor='black', edgecolor='black')
            ax.add_patch(polygon)
        ax.set_xlim(x_bounds[0], x_bounds[1])
        ax.set_ylim(y_bounds[0], y_bounds[1])
        plt.show()

    # Method to open obstacle file
    # with open('Environments/environment_0.pickle', 'rb') as f:
    #     obstacles = pickle.load(f)

if __name__ == "__main__":
    main()