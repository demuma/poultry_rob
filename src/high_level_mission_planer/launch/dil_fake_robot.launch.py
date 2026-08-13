from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_static_camera_tf = LaunchConfiguration("use_static_camera_tf")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    rmw_implementation = LaunchConfiguration("rmw_implementation")

    mission_launch = PathJoinSubstitution([
        FindPackageShare("high_level_mission_planer"),
        "launch",
        "mission_executor.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_static_camera_tf", default_value="true"),
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
        Node(
            package="high_level_mission_planer",
            executable="fake_nav2_server",
            name="fake_nav2_server",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            launch_arguments={
                "use_static_camera_tf": use_static_camera_tf,
            }.items(),
        ),
    ])
