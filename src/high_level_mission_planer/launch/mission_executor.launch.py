from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_static_camera_tf = LaunchConfiguration("use_static_camera_tf")

    config = os.path.join(
        get_package_share_directory("high_level_mission_planer"),
        "config",
        "mission_executor.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_static_camera_tf",
            default_value="true",
            description=(
                "Publish the simple map -> camera_optical_frame transform used by "
                "the current DIL/simulation setup. Disable when the robot/DIL stack "
                "provides the transform itself."
            ),
        ),
        Node(
            package="high_level_mission_planer",
            executable="mission_executor",
            name="mission_executor",
            output="screen",
            parameters=[config]
        ),
        Node(
            package="high_level_mission_planer",
            executable="mission_visualizer",
            name="mission_visualizer",
            output="screen",
            parameters=[config]
        ),
        Node(
            package="high_level_mission_planer",
            executable="target_manager",
            name="target_manager",
            output="screen",
            parameters=[config]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.1", "-0.1", "0", "0", "0", "0", "map", "camera_optical_frame"],
            condition=IfCondition(use_static_camera_tf),
        )
    ])
