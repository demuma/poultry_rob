from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_robot_description = LaunchConfiguration("use_robot_description")
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")

    config = os.path.join(
        get_package_share_directory("high_level_mission_planer"),
        "config",
        "mission_executor.yaml"
    )

    robot_description_content = Command([
        "cat ",
        PathJoinSubstitution([
            FindPackageShare(robot_description_package),
            "urdf",
            robot_description_file,
        ]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_robot_description",
            default_value="false",
            description="Start robot_state_publisher for the poultry robot URDF.",
        ),
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="high_level_mission_planer",
            description="Package that provides the poultry robot URDF/Xacro.",
        ),
        DeclareLaunchArgument(
            "robot_description_file",
            default_value="poultry_robot_visual.urdf",
            description="URDF file below the package's urdf directory.",
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
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.1", "-0.1", "0", "0", "0", "0", "map", "camera_optical_frame"]
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            condition=IfCondition(use_robot_description),
            parameters=[{
                "robot_description": robot_description_content,
            }],
        )
    ])
