from path_planning import config
from path_planning.rrtsharp import RRTSharp, error
from path_planning.ellipses2 import Ellipse2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from shapely import Polygon
import numpy as np
import pickle

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher_node')

        self.publisher_ = self.create_publisher(Path, 'path', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # get qr_data from qr_reader_node
        self.subscription = self.create_subscription(
            String,
            'qr_data',
            self.listener_callback,
            10
        )

        self.qr_data = None
        self.path_msg = None

        # get env_file from launch - used if ellipse 
        self.declare_parameter('env_file', '')
        self.env_file = self.get_parameter('env_file').value

    def timer_callback(self):
        if self.path_msg is None:
            return
        
        self.publisher_.publish(self.path_msg)
        if len(self.path_msg.poses) > 0:
            if self.i == 0:
                self.i += 1
                start = self.path_msg.poses[0].pose.position
                goal = self.path_msg.poses[-1].pose.position
                self.get_logger().info(
                    f'Publishing path: ({start.x:.2f}, {start.y:.2f}) -> ({goal.x:.2f}, {goal.y:.2f})\n'
            )

    def listener_callback(self, msg):

        if self.qr_data is None:
            self.get_logger().info(f"Reading QR data {msg.data}\n")
            self.qr_data = msg.data
            path_points, ellipse = self._get_path(msg.data) # convert to path
            path_points = path_points[::-1] # flip message, robot 1 goal is robot 2 start
            self.path_msg = self._path_to_poses(path_points) # convert to poses

    def _get_path(self, qr_msg):
        """
        Converts String from qr to path.
        
        Parameters
        ----------
        qr_msg : str
            QR data from qr_reader_node

        Returns
        -------
        path : list
            List of points from QR code.
        ellipse : Ellipse2
            Ellipse data if present in QR.
    
        Notes
        -----
        Decode the QR payload created by _build_qr_for_indices:
        - Triples (x,y,s) everywhere EXCEPT for a single f2 pair immediately after f1's triple
            (unless f2 == goal, then that pair is omitted).
        - Goal is never encoded; always appended.
        - The string ends with either (x,y) or (x,y,s) but never a single x.
        """
    
        data = list(map(int, qr_msg.split()))
        path = []
        ellipse = None
        f1 = f2 = None
        s_val = None

        i = 0
        while i < len(data):
            # If we have at least a triple available, try to read (x,y,s)
            if i + 2 < len(data):
                x, y, s = data[i], data[i + 1], data[i + 2]
                if s != 0 and ellipse is None:
                    # This is f1
                    f1 = (x, y)
                    s_val = s
                    path.append((x, y))

                    # Expect f2 pair immediately after f1 triple (if present)
                    if i + 4 < len(data):
                        f2 = (data[i + 3], data[i + 4])
                        path.append(f2)
                        i += 5  # consumed triple + pair
                    else:
                        # No pair after f1: f2 must be the known goal
                        f2 = config.GOAL
                        i += 3  # consumed triple
                    continue
                else:
                    # Regular triple (x,y,0)
                    path.append((x, y))
                    i += 3
                    continue
            # If only 2 numbers remain, treat them as a plain (x,y) pair (e.g., f2 at end)
            elif i + 1 < len(data):
                path.append((data[i], data[i + 1]))
                i += 2
                continue
            else:
                break

        # Append the known goal at the end
        path.append(config.GOAL)

        if f1 is not None and f2 is not None:
            ellipse = Ellipse2(f1, f2, s_val)

        # patch together ellipse
        if ellipse is not None:
            self.get_logger().info(f"Ellipse detected: f1={f1}, f2={f2}")
           
            with open(self.env_file, 'rb') as f:
                obstacles = pickle.load(f)

            self.get_logger().info(f"Reading env file: {self.env_file}\n")
            obstacles = [Polygon(np.array(poly)) for poly in obstacles]

            error_matrix = [[0 for _ in range(50)] for _ in range(max(config.ENV_X_BOUNDS))]
            e_env = 0.0
            e = error(e_env, error_matrix)

            # use RRT# to get path from f1 to f2
            self.get_logger().info(f"Running RRT# in ellipse")
            rrt = RRTSharp(
                start = f1,
                goal = f2,
                bounds = config.BOUNDS,
                step_size = 2,
                obstacles = obstacles,
                e=e,
                ellipse = ellipse
            )

            ell_path, nodes, e = rrt.rrt_sharp()

            f1_idx = path.index(f1)
            f2_idx = path.index(f2)

            path = path[:f1_idx + 1] + ell_path[1:-1] + path[f2_idx:]

        return path, ellipse

    def _path_to_poses(self, path_points):
        """
        Converts path from RRT# to nav_msgs.msgs.Path

        Parameters
        ----------
        path_points : list
            List of points from RRT#.

        Returns
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

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()