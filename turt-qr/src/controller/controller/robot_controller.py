import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        # list of path poses
        self.path_subscription = self.create_subscription(
            Path,
            'path',
            self.listener_callback,
            10
        )

        # get current pose from odom
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # publish if at goal
        self.at_goal_publisher = self.create_publisher(Bool, '/at_goal', 10)

        self.path = None
        self.odom = None
        self.at_goal = False

        # publish 'cmd_vel'
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        timer_period = 0.5 # 2hz (may increase later - lets me see begining logs)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self.path_idx = 0
        self.epsilon = 0.1 # 10 cm

        # coefficients for control
        self.kp = 0.8
        self.k_theta = 2.5

    def listener_callback(self, msg):
        self.path = msg.poses # get path
        if self.i == 0:
            self.i += 1
            self.get_logger().info(f"Starting path\n")

    def odom_callback(self, odom_msg):
        self.odom = odom_msg # just stores odom msg
            
    def timer_callback(self):
        if self.odom is None:
            self.get_logger().warn("No odom yet")
            return
        
        if self.path is None:
            self.get_logger().warn("No path yet")
            return
        
        if self.at_goal == True:
            vel_msg = Twist()
            self.publisher_.publish(vel_msg)
            return
 
        # get current pose
        odom_x = self.odom.pose.pose.position.x
        odom_y = self.odom.pose.pose.position.y
        odom_theta = self._quaternion_to_yaw(self.odom.pose.pose.orientation)
        
        self.get_logger().info(f"Current Odom -> x: {odom_x} y: {odom_y} theta: {odom_theta}\n")

        # get desired pose
        goal_x = self.path[self.path_idx].pose.position.x
        goal_y = self.path[self.path_idx].pose.position.y
        goal_theta = np.arctan2(goal_y - odom_y, goal_x - odom_x)

        self.get_logger().info(f"Current goal -> x: {goal_x} y: {goal_y} theta: {goal_theta}\n")

        # controller stuff
        position_error = np.sqrt((goal_x - odom_x)**2 + (goal_y - odom_y)**2)
        heading_error = goal_theta - odom_theta

        # normalize to [-pi, pi]
        while heading_error > np.pi:
            heading_error -= 2*np.pi
        while heading_error < -np.pi:
            heading_error += 2*np.pi

        self.get_logger().info(f"Heading error: {heading_error}\n")

        # assign cmd_vel
        vel_msg = Twist()
        vel_msg.linear.x = np.clip(self.kp * position_error * np.cos(heading_error), 0, 0.15) # max translation vel is 0.22 m/s
        vel_msg.angular.z = np.clip(self.k_theta * heading_error, -2.0, 2.0) # max rotational vel is 2.84 rad/s

        self.get_logger().info(f"linear_vel: {vel_msg.linear.x}\n")
        self.get_logger().info(f"Angular_vel: {vel_msg.angular.z}\n")

        if position_error < self.epsilon:
            # go to next pose if close enough to desired pose
            self.path_idx += 1
            if self.path_idx >= len(self.path):
                self.at_goal = True
                vel_msg = Twist() # stop moving
                self.get_logger().info("GOAL REACHED")
                at_goal_msg = Bool()
                at_goal_msg.data = True
                self.at_goal_publisher.publish(at_goal_msg)
                return
 
        self.publisher_.publish(vel_msg)

    def _quaternion_to_yaw(self, q):
        """
        Convert quaternion to heading.
        """
        theta = np.arctan2(
            2.0 * (q.w*q.z + q.x*q.y), 
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        return theta

def main(args=None):
    rclpy.init(args=args)
    path_publisher = RobotController()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()