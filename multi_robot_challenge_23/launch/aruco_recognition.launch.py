import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='tb3_5'
    )

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        namespace=namespace,
        parameters=[{"marker_size": 0.5},
                    {"aruco_dictionary_id": "DICT_5X5_250"},
                    {"image_topic": "camera/image_raw"},
                    {"camera_info_topic": "camera/camera_info"}]
    )

    # The package-local marker processing node is optional and may not be
    # present in the workspace. This launch file only starts the third-party
    # ArUco detector node and exposes a 'namespace' argument so it can be
    # included per-robot from a top-level launch file.
    return LaunchDescription([
        namespace_launch_arg,
        aruco_node,
    ])