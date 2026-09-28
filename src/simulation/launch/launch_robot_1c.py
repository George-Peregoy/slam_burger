from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from path_planning import config
import os

def generate_launch_description():
 
    # create arg for model
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='TurtleBot3 model type'
    )  
    
    # where world dir is kept
    sim_share = get_package_share_directory('simulation')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        sim_share
    )
    
    # create arg for world
    world_num_arg = DeclareLaunchArgument(
        'world_num',
        default_value='0',
        description='Arg for which world to use.'
        )

    world_num = LaunchConfiguration('world_num')

    world_file = PathJoinSubstitution([
        sim_share,
        'worlds',
        ['world_', world_num, '.world']
    ])

    path_publisher_node = Node(
        package = 'path_planning',
        executable = 'path_publisher_1c',
        parameters=[{
            'qr_num' : world_num,
        }]
    )

    # controller node
    controller_node = Node(
        package = 'controller',
        executable = 'controller_node_c'
    )

    # resolve args
    model = LaunchConfiguration('model')

    # set env variable for model
    set_turtlebot_model = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL',
        model
    )

    # gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world' : world_file,
            'verbose' : 'false'
        }.items()
    )

    # debug world file
    log_world = LogInfo(
        msg=['Loading world file: ', world_file]
    )

    # spawn bot
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'spawn_turtlebot3.launch.py'
            ])
        ]),
        launch_arguments={
            'x_pose': f'{config.START[0]*config.WORLD_SCALE}',  # Robot 1 start position
            'y_pose': f'{config.START[1]*config.WORLD_SCALE}',
            'z_pose': '0.01',
            'robot_name': 'robot_1'
        }.items()
    )

    # slam node 
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time' : True,
            'odom_frame' : 'odom',
            'map_frame' : 'map',
            'base_frame' : 'base_footprint',
            'scan_topic': '/scan',
            'resolution' : 0.025,
            'max_laser_range': 6.0,
            'map_update_interval' : 0.5,
            'transform_publish_period' : 0.02,
            'loop_closure_enabled': True,
            'loop_search_maximum_distance': 5.0,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            'loop_match_minimum_response_coarse': 0.2,
            'loop_match_minimum_response_fine': 0.35,
            'minimum_travel_distance': 0.3,
            'minimum_travel_heading': 0.3,
            'map_update_interval': 0.3,
        }]
    )

    # maps robot base to lidar frame 
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_footprint', 'base_scan']
    )

    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', os.path.join(get_package_share_directory('simulation'), 'rviz', 'robot1.rviz')]
    )

    return LaunchDescription([
        set_model_path,
        model_arg,
        world_num_arg,
        set_turtlebot_model,
        log_world,
        gazebo,
        spawn_turtlebot, 
        rviz_node,
        slam_node,
        static_tf_node,
        path_publisher_node,
        controller_node
    ])