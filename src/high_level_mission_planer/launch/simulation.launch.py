from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
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
        DeclareLaunchArgument("scenario", default_value="new_near_hen"),
        DeclareLaunchArgument("use_robot_description", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("ros_domain_id", default_value="75"),
        DeclareLaunchArgument("rmw_implementation", default_value="rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
        SetEnvironmentVariable("UDS_SCENARIO", scenario),
        Node(
            package="poultry_rob_bridge",
            executable="scenario_uds_server",
            name="scenario_uds_server",
            output="screen",
        ),
        Node(
            package="poultry_rob_bridge",
            executable="uds_bridge_node",
            name="uds_proto_bridge",
            output="screen",
        ),
        Node(
            package="high_level_mission_planer",
            executable="fake_nav2_server",
            name="fake_nav2_server",
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
