from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # interact with actual bot
    robot_bringup = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('turtlebot3_bringup'),
            'launch',
            'robot.launch.py'
        ])
    )
    
    # create arg for world
    world_num_arg = DeclareLaunchArgument(
        'world_num',
        default_value='0',
        description='Arg for which world to use.'
        )

    world_num = LaunchConfiguration('world_num')

    # use env file associated with world 
    path_share = get_package_share_directory('path_planning')
    env_file = PathJoinSubstitution([
        path_share,
        'environments',
        ['environment_polygon_', world_num, '.pickle']
    ])

    path_publisher_node = Node(
        package = 'path_planning',
        executable = 'path_publisher_1b',
        parameters = [{
            'env_file': env_file,
            'world_num' : world_num
        }]
    )

    # controller node
    controller_node = Node(
        package = 'controller',
        executable = 'controller_node_1b'
    )

    return LaunchDescription([
        world_num_arg,
        robot_bringup,
        path_publisher_node,
        controller_node
    ])
