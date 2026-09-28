import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        self.path_subscription = self.create_subscription(
            Path,
            'path',
            self.path_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.at_end_publisher = self.create_publisher(Bool, '/at_end', 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.path = None
        self.path_index = 0
        self.path_start = None  # (x, y) of path[0] — used to detect new path
        self.path_end = None    # (x, y) of path[-1] — used to detect new path
        self.odom = None

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.epsilon = 0.3
        self.kp = 0.8
        self.k_theta = 2.5

        self.at_end_sent = False # help prevent sampling before finished

    def path_callback(self, msg):
        """
        Receives nav_msgs.msg.Path object from path planning node.
        """
        if not msg.poses:
            return

        new_start = (msg.poses[0].pose.position.x, msg.poses[0].pose.position.y)
        new_end   = (msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y)

        if new_start != self.path_start or new_end != self.path_end:
            self.get_logger().info("NEW PATH — resetting index\n")
            self.path = msg.poses
            self.path_index = 0
            self.path_start = new_start
            self.path_end = new_end
            self.at_end_sent = False

    def odom_callback(self, odom_msg):
        """
        Stores odom.

        Parameters
        ----------
        odom_msg : nav_msgs.msg.Odometry
        """
        self.odom = odom_msg

    def timer_callback(self):
        """
        Get position and calculate cmd_vel.
        """
        if self.odom is None:
            self.get_logger().warn("No odom yet")
            return

        if self.path is None:
            self.get_logger().warn("No path yet")
            return

        if self.path_index >= len(self.path): # if at path end don't move, trigger end of path
            if self.at_end_sent:
                return # if sent once skip
            vel_msg = Twist()
            self.publisher_.publish(vel_msg)
            at_end_msg = Bool()
            at_end_msg.data = True
            self.at_end_publisher.publish(at_end_msg)
            self.get_logger().info("AT END OF PATH")
            self.at_end_sent = True
            return

        odom_x = self.odom.pose.pose.position.x
        odom_y = self.odom.pose.pose.position.y
        odom_theta = self._quaternion_to_yaw(self.odom.pose.pose.orientation)

        goal_x = self.path[self.path_index].pose.position.x
        goal_y = self.path[self.path_index].pose.position.y
        goal_theta = np.arctan2(goal_y - odom_y, goal_x - odom_x)

        self.get_logger().info(f"Current Odom -> x: {odom_x:.3f} y: {odom_y:.3f} theta: {odom_theta:.3f}")
        self.get_logger().info(f"Waypoint [{self.path_index}/{len(self.path)-1}] -> x: {goal_x:.3f} y: {goal_y:.3f}")

        position_error = np.sqrt((goal_x - odom_x)**2 + (goal_y - odom_y)**2) # get pos and heading error
        heading_error  = goal_theta - odom_theta

        while heading_error >  np.pi: heading_error -= 2 * np.pi # normalize heading
        while heading_error < -np.pi: heading_error += 2 * np.pi

        if self.path_index < len(self.path) - 1: # follow point 1 ahead for smooth turning
            next_wp = self.path[self.path_index + 1]
            to_current = np.array([goal_x - odom_x, goal_y - odom_y])
            to_next = np.array([next_wp.pose.position.x - odom_x, next_wp.pose.position.y - odom_y])
            if np.dot(to_current, to_next) < 0 or position_error < self.epsilon:
                self.get_logger().info(f"Passed waypoint {self.path_index}, advancing")
                self.path_index += 1 # if past point increase path idx
        else:
            if position_error < self.epsilon: # if near point increase idx
                self.path_index += 1

        vel_msg = Twist() # set up cmd_vel
        vel_msg.linear.x  = np.clip(self.kp * position_error * np.cos(heading_error), 0, 0.10)
        vel_msg.angular.z = np.clip(self.k_theta * heading_error, -2.0, 2.0)

        self.get_logger().info(f"linear: {vel_msg.linear.x:.3f}  angular: {vel_msg.angular.z:.3f}")
        self.publisher_.publish(vel_msg)

    def _quaternion_to_yaw(self, q):
        return np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()