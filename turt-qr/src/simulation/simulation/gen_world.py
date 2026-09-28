from simulation.env_to_world import env_to_world
from path_planning.gen_obstacles import generate_obstacles
from path_planning import config
from path_planning.rrtsharp import RRTSharp, error
import numpy as np
import os
import pickle


def main(num_worlds):
    """
    Steps
    -----
    1. Make environment using method from generate_obstacles.py
    2. Dump environment into src/path_planning/environments
    3. Generate qr code for path
    4. Save qr code to src/path_planning/qrcodes
    5. Convert 2D obstacles to 3D stl files
    6. Add stl files to world file
    7. Save world file to src/simulation/worlds
    """

    # set up params
    num_obstacles = 8
    min_sides = 3
    max_sides = 4
    x_bounds = config.ENV_X_BOUNDS
    y_bounds = config.ENV_Y_BOUNDS
    max_area_ratio = 0.15
    max_side_length = 20

    # set up dirs
    base_dir = os.path.abspath(os.path.dirname(__file__))
    root_dir = os.path.join(base_dir, '..', '..')
    env_dir = os.path.join(root_dir, 'path_planning', 'environments')

    i = 0
    while i < num_worlds:
        
        # make env
        file_name = os.path.join(env_dir, f'environment_polygon_{i}.pickle')

        obstacles = generate_obstacles(num_obstacles, min_sides, max_sides, \
                                       x_bounds, y_bounds, max_area_ratio, max_side_length)
        
        with open(file_name, 'wb') as f:
            pickle.dump(obstacles, f)
        
        # simulate robot 1 
        # gen path, encode
        error_matrix = np.zeros((config.ENV_X_BOUNDS[1], config.ENV_Y_BOUNDS[1]))
        e_env = 0.0
        e = error(e_env, error_matrix)

        rrt = RRTSharp(start = config.START, 
                       goal = config.GOAL, 
                       bounds = config.BOUNDS,
                       step_size = config.STEP_SIZE, 
                       obstacles=obstacles,
                       e = e,
                       )
        
        path, nodes, e = rrt.rrt_sharp()

        # make sure valid path
        if len(path) < 3:
            print(f"Path too short regenerating world {i}...\n")
            continue

        # make world
        env_to_world(i)

        i += 1
        
if __name__=="__main__":
    num_worlds = 5
    main(num_worlds=num_worlds)