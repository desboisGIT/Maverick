"""Start the S3 host stack: mapping, memory, explore skill, gateway (and the flight client).

    ros2 launch maverick_gateway s3_host.launch.py                  # everything, ws on :8765
    ros2 launch maverick_gateway s3_host.launch.py flight:=false    # flight client run elsewhere
    ros2 launch maverick_gateway s3_host.launch.py port:=9000 altitude:=1.2 seed:=3

Inputs are the five /maverick/camera/* topics (S3 plan, Integration contract) and the flight
client's /maverick/flight/status. The gateway needs no port besides `port`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    altitude = {"altitude": LC("altitude")}
    return LaunchDescription(
        [
            DeclareLaunchArgument("flight", default_value="true"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("altitude", default_value="1.2"),
            DeclareLaunchArgument("seed", default_value="0"),
            Node(
                package="maverick_flight",
                executable="flight_client",
                name="flight_client",
                condition=IfCondition(LC("flight")),
                parameters=[{"altitude": 2.0, "hover_seconds": 0.0}],
                output="screen",
            ),
            Node(
                package="maverick_mapping",
                executable="mapping_node",
                parameters=[altitude],
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
