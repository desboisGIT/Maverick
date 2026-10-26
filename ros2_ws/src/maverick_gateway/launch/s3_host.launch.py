"""Start the S3 host stack: mapping, memory, explore skill, gateway (and the flight client).

    ros2 launch maverick_gateway s3_host.launch.py                  # everything, ws on :8765
    ros2 launch maverick_gateway s3_host.launch.py flight:=false    # flight client run elsewhere
    ros2 launch maverick_gateway s3_host.launch.py port:=9000 altitude:=1.2 seed:=3

Inputs are the /maverick/camera/* topics (S3 plan, Integration contract) and the flight
client's /maverick/flight/status. The gateway needs no port besides `port`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    altitude = {"altitude": LC("altitude")}
    return LaunchDescription(
        [
            DeclareLaunchArgument("flight", default_value="true"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("altitude", default_value="1.2"),
            DeclareLaunchArgument("seed", default_value="0"),
            DeclareLaunchArgument("vo", default_value="true"),
            # arm only once PX4 fuses the external-vision position (mocap or vo localisation)
            DeclareLaunchArgument("require_ev", default_value="false"),
            DeclareLaunchArgument("vo_mode", default_value="icp"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("maverick_vo"), "launch", "vo.launch.py"
                    )
                ),
                launch_arguments={"vo": LC("vo"), "mode": LC("vo_mode")}.items(),
            ),
            Node(
                package="maverick_flight",
                executable="flight_client",
                name="flight_client",
                condition=IfCondition(LC("flight")),
                parameters=[
                    {
                        "altitude": 2.0,
                        "hover_seconds": 0.0,
                        "require_ev": ParameterValue(LC("require_ev"), value_type=bool),
                    }
                ],
                output="screen",
            ),
            Node(
                package="maverick_mapping",
                executable="mapping_node",
                parameters=[altitude, {"stride": 2}],  # depth is 320x240 (= 160x120 sampling)
                output="screen",
            ),
            Node(
                package="maverick_memory",
                executable="memory_node",
                parameters=[{"seed": LC("seed")}],
                output="screen",
            ),
            Node(
                package="maverick_skills",
                executable="skill_node",
                parameters=[altitude],
                output="screen",
            ),
            Node(
                package="maverick_gateway",
                executable="gateway",
                parameters=[{"port": LC("port")}],
                output="screen",
            ),
        ]
    )
