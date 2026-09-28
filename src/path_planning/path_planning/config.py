"""
File used to store global variables.

Notes
-----
File is currenly being used for TurtleBot3 Burgerbots.
"""

ENV_X_BOUNDS = (0,50) 
ENV_Y_BOUNDS = (0,50)
BOUNDS = (ENV_X_BOUNDS, ENV_Y_BOUNDS)

START = (5, 5) # robot 1 start | robot 2 end
GOAL = (45, 45) # robot 2 start | robot 1 end

ROBOT_RADIUS = 0.105 # meters
RADIUS_BUFFER = ROBOT_RADIUS * 0.1
BUFFER = ROBOT_RADIUS + RADIUS_BUFFER # in meters

WORLD_SCALE = 0.1 # world_units / env_units

CHAR_LIMIT = 25 # for qr codes
STEP_SIZE = 5 # used for consistency 