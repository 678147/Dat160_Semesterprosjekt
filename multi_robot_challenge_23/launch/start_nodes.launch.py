import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'multi_robot_challenge_23'

    # Robots (namespace configuration)
    robots = [
        {'ns': 'tb3_0'},
        {'ns': 'tb3_1'},
    ]

    # Launch arguments
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock')
    enable_aruco_arg = DeclareLaunchArgument('enable_aruco', default_value='false', description='Start ArUco detector per robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_aruco = LaunchConfiguration('enable_aruco')

    # NOTE: map_server, lifecycle_manager, robot spawns, and RViz are started
    # by the world-specific rescue_robots_wx.launch.py files.
    # This launcher only starts per-robot controller nodes and the global coordinator.

    # Optional ArUco includes per robot (only if enable_aruco=='true')
    aruco_includes = []
    for r in robots:
        aruco = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch'), '/aruco_recognition.launch.py']),
            launch_arguments={'namespace': r['ns']}.items(),
            condition=IfCondition(enable_aruco)
        )
        aruco_includes.append(aruco)

    # Per-robot nodes
    per_robot_nodes = []
    for r in robots:
        ns = r['ns']
        per_robot_nodes += [
            Node(package=package_name, executable='go_to_point', name=f'go_to_point_{ns}', output='screen', parameters=[{'robot_name': ns, 'use_sim_time': use_sim_time}]),
            Node(package=package_name, executable='wall_follower', name=f'wall_follower_{ns}', output='screen', parameters=[{'robot_name': ns, 'use_sim_time': use_sim_time}]),
            Node(package=package_name, executable='robot_controller', name=f'robot_controller_{ns}', output='screen', parameters=[{'robot_name': ns, 'use_sim_time': use_sim_time}]),
            #Node(package=package_name, executable='bug2_controller', name=f'bug2_controller_{ns}', output='screen', parameters=[{'robot_name': ns, 'use_sim_time': use_sim_time}]),
        ]

    # Coordinator
    # Start a small empty-map publisher that publishes a mostly-unknown map with
    # small free patches at robot start locations. This publishes on /test_map so
    # it doesn't conflict with any existing /map published by a map_server.

    coordinator_node = Node(
        package=package_name,
        executable='robot_coordinator',
        name='robot_coordinator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'map_topic': '/map'}]
    )

    actions = [
        sim_time_arg,
        enable_aruco_arg,
    ]
    actions.extend(aruco_includes)
    actions.extend(per_robot_nodes)
    actions.append(coordinator_node)

    return LaunchDescription(actions)
