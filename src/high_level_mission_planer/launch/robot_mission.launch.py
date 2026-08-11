from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_robot_description = LaunchConfiguration("use_robot_description")
    use_rviz = LaunchConfiguration("use_rviz")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    rmw_implementation = LaunchConfiguration("rmw_implementation")

    mission_launch = PathJoinSubstitution([
        FindPackageShare("high_level_mission_planer"),
        "launch",
        "mission_executor.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_robot_description", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("ros_domain_id", default_value="75"),
        DeclareLaunchArgument("rmw_implementation", default_value="rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
        Node(
            package="poultry_rob_bridge",
            executable="uds_bridge_node",
            name="uds_proto_bridge",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            launch_arguments={
                "use_robot_description": use_robot_description,
                "use_rviz": use_rviz,
            }.items(),
        ),
    ])
