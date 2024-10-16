import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node 
import launch
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


# Share directories
pkg_ros2_astar = get_package_share_directory('ros2_astar')
pkg_astar = get_package_share_directory('astar')

def setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_frame = LaunchConfiguration('rviz_frame')
    map_file = LaunchConfiguration('map_file')


    map_server = Node(
        parameters=[
          {'use_sim_time': use_sim_time},
          {'yaml_filename':map_file}
        ],
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen'
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}
        ]
    )            

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', rviz_config_file,
            '-f', rviz_frame,
        ],
        output={'both': 'log'}
    )

    astar_algorithm = Node(
        package='astar',
        executable='astar',
        name='astar_node',
        output='screen',
        parameters=[os.path.join(pkg_astar, 'config', 'params.yaml')]
    )

    nodes_to_start = []
    nodes_to_start.append(map_server)
    nodes_to_start.append(start_rviz_cmd)
    nodes_to_start.append(nav2_lifecycle_manager)
    nodes_to_start.append(astar_algorithm)

    return nodes_to_start

def generate_launch_description():

    return LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rviz_config_file', 
            default_value=os.path.join(
                pkg_astar,
                'rviz',
                'config.rviz'
            ),
            description='Rviz config file path'
        ),
        launch.actions.DeclareLaunchArgument(
            name='map_file', 
            default_value=os.path.join(
                pkg_ros2_astar,
                'maps/empty_world',
                'scenario_1.yaml'
            ),
            description='Map file path'
        ),
        launch.actions.DeclareLaunchArgument(
            'rviz_frame',
            default_value='map',
            description='Fixed frame'
        ),

        OpaqueFunction(function=setup)
])