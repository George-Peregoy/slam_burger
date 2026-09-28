from path_planning import config
from path_planning.rrtsharp import RRTSharp, error
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from  std_msgs.msg import Bool
from path_planning.path_to_qr import path_to_qr, path_to_qr_printer
from path_planning.path_pruning import fit_to_qr
from shapely import Polygon
import numpy as np
import pickle
import os

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        # get env_file from launch 
        self.declare_parameter('env_file', '')
        self.env_file = self.get_parameter('env_file').value

        self.declare_parameter('world_num', -1)
        self.world_num = self.get_parameter('world_num').value

        self.publisher_ = self.create_publisher(Path, 'path', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.path_points = self._get_path()
        self.path_msg = self._path_to_poses(self.path_points)

        # subscribe to at_goal and then generate qr
        self.at_goal_sub = self.create_subscription(
            Bool,
            '/at_goal',
            self.goal_callback,
            10
        )

    def timer_callback(self):
        self.publisher_.publish(self.path_msg)
        if self.i == 0:
            self.i+= 1
            start = self.path_msg.poses[0].pose.position
            goal = self.path_msg.poses[-1].pose.position
            self.get_logger().info(
                f'Publishing path: ({start.x:.2f}, {start.y:.2f}) -> ({goal.x:.2f}, {goal.y:.2f})'
            )

    def _get_path(self):
        """
        Finds path using RRT#, converts to nav_msgs.msg.Path for publishing.

        Returns
        -------
        path : list
            List of points from RRT#.
        """
        with open(self.env_file, 'rb')as f:
            obstacles = pickle.load(f)

        self.obstacles = [Polygon(np.array(poly)) for poly in obstacles]

        error_matrix = np.zeros((config.ENV_X_BOUNDS[1], config.ENV_Y_BOUNDS[1]))
        e_env = 0.0
        e = error(e_env, error_matrix)
        
        # odom start is (0, 0) this is accounted for in controller
        rrt = RRTSharp(start = config.START, 
                       goal = config.GOAL,
                       bounds = config.BOUNDS, 
                       obstacles=self.obstacles,
                       e = e,
                       )
        
        path, nodes, self.e = rrt.rrt_sharp()
        return path

    def _path_to_poses(self, path_points):
        """
        Converts path from RRT# to nav_msgs.msgs.Path

        Parameters
        ----------
        path_points : list
            List of points from RRT#.

        Returnsx
        -------
        nav_path : Path
            nav_msgs.msg.Path object, contains array of PoseStamped.
        """

        # Path needs pose.orientation x|y|z|w
        # pose.postion x|y|z, set z=0 for all poses.

        path_msg = Path()
        path_msg.header.frame_id = 'map' # header and stamp needed for msg
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # convert each point to PoseStamped
        for i in range(len(path_points)): 
            pose = PoseStamped()
            pose.header.frame_id = 'map' # header and stamp needed for msg
            pose.header.stamp = self.get_clock().now().to_msg()

            # assign position
            pose.pose.position.x = float(path_points[i][0]) * config.WORLD_SCALE # convert to world size
            pose.pose.position.y = float(path_points[i][1]) * config.WORLD_SCALE# must be float for msg
            pose.pose.position.z = 0.0 # always 0 for this case

            # assign orientation 
            if i < len(path_points) - 1:
                dx = path_points[i + 1][0] - path_points[i][0]
                dy = path_points[i + 1][1] - path_points[i][1]
                theta = np.arctan2(dy, dx)
            else:
                if i > 0:
                    # if last point, use orientation of previous point 
                    # doesn't matter where it's facing at end
                    dx = path_points[i][0] - path_points[i - 1][0]
                    dy = path_points[i][1] - path_points[i - 1][1]
                else:
                    theta = 0.0
                
            # convert to quaternion. (yaw=theta)
            q = self._yaw_to_quaternion(theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            # append to path 
            path_msg.poses.append(pose)

        return path_msg
    
    def _yaw_to_quaternion(self, yaw):
        """
        Convert yaw to quaternion (x, y, z, w)

        Parameters
        ----------
        yaw : float
            Yaw angle in radians.
        
        Returns
        x, y, z, w
        """
        return (
            0.0,
            0.0,
            np.sin(yaw / 2.0),
            np.cos(yaw / 2.0)
        )

    def goal_callback(self, at_goal_msg):

        if at_goal_msg.data == True:

            self.get_logger().info(f"PATH POINTS {self.path_points}\n")

            path_str = fit_to_qr(path=self.path_points, 
                                obstacles=self.obstacles, 
                                e=self.e,
                                step_size=config.STEP_SIZE,
                                char_limit=config.CHAR_LIMIT)
            
            self.get_logger().info(f"Saving path as {path_str}\n")
            
            base_dir = os.path.abspath(os.path.dirname(__file__)) # reads share dir 
            root_dir = os.path.join(base_dir, '..', '..', '..', '..', '..', '..')
            qr_dir = os.path.join(root_dir, 'src', 'path_planning', 'qrcodes')
            
            path_to_qr_printer(path=path_str, output_dir=qr_dir, env_number=self.world_num)

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
