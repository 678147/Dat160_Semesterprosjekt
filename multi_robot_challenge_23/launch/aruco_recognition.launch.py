import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument('namespace', default_value='tb3_0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    frame_prefix = [ns, '/aruco_']

    # aruco_node.py
    aruco_node = Node(
        package='multi_robot_challenge_23',
        executable='aruco_node',
        name='aruco_node',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_prefix': frame_prefix,     
            'image_topic': 'camera/image_raw',
            'camera_info_topic': 'camera/camera_info',
        }]
    )

    # marker_recognition.py
    marker_recognition = Node(
        package='multi_robot_challenge_23',
        executable='marker_recognition',
        name='marker_recognition',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': ns,                      
            'base_frame': [ns, '/base_link'],      
            'pose_topic': 'aruco/poses',         
            'ids_topic': 'aruco/ids',              
            'standoff': 0.6,
            'min_reobserve': 2,
            'goal_timeout_sec': 10.0,
            'aruco_prefix': frame_prefix,        
            'map_frame': 'map'
        }]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        aruco_node,
        marker_recognition,
    ])
