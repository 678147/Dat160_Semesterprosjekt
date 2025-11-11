import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )
    enable_aruco_arg = DeclareLaunchArgument(
        'enable_aruco', default_value='true', description='Start ArUco detector per robot'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_aruco = LaunchConfiguration('enable_aruco')

    # Optional ArUco includes per robot (only if enable_aruco=='true')
    aruco_includes = []
    for r in robots:
        aruco = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(package_name), 'launch'),
                '/aruco_recognition.launch.py'
            ]),
            # Videresend både namespace og use_sim_time til under-launch
            launch_arguments={
                'namespace': r['ns'],
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(enable_aruco)
        )
        aruco_includes.append(aruco)

    # Per-robot nodes
    per_robot_nodes = []
    for r in robots:
        ns = r['ns']
        per_robot_nodes += [
            Node(
                package=package_name, executable='go_to_point',
                name=f'go_to_point_{ns}', output='screen',
                parameters=[{'robot_name': ns, 'use_sim_time': use_sim_time}]
            ),
            Node(
                package=package_name, executable='wall_follower',
                name=f'wall_follower_{ns}', output='screen',
                parameters=[{'robot_name': ns, 'use_sim_time': use_sim_time}]
            ),
            # Node(
            #     package=package_name, executable='robot_controller',
            #     name=f'robot_controller_{ns}', output='screen',
            #     parameters=[{'robot_name': ns, 'use_sim_time': use_sim_time}]
            # ),
            Node(
                package=package_name, executable='bug2_controller',
                name=f'bug2_controller_{ns}', output='screen',
                parameters=[{'robot_name': ns, 'use_sim_time': use_sim_time}]
            ),
        ]

    coordinator_node = Node(
        package=package_name,
        executable='coordinator',
        name='coordinator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'map_topic': '/map'}]
    )

    actions = [
        sim_time_arg,
        enable_aruco_arg,
        *aruco_includes,
        *per_robot_nodes,
        coordinator_node,
    ]

    return LaunchDescription(actions)
