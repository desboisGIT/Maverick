"""Host-side camera unpacking (always) and visual odometry (vo:=true).

Nodes: camera_unpack, an rtabmap odometry node, vo_adapter.

    ros2 launch maverick_vo vo.launch.py                 # icp_odometry on a point cloud from depth
    ros2 launch maverick_vo vo.launch.py mode:=rgbd      # rgbd_odometry (ORB features + depth)
    ros2 launch maverick_vo vo.launch.py vo:=false       # only camera_unpack (mapping, memory)

The odometry node's frame_id is the camera optical frame itself (the images' frame), so no TF is
needed; its pose is relative to its start and vo_adapter anchors it in the map frame. The
parameter sets below are the ones measured in the generated worlds, see sim/px4/README.md "Visual
odometry". MAVERICK_VO_OVERRIDES (JSON object of rtabmap parameters, string values) merges over
them, a development knob for tuning runs.
"""

import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

COMMON = {
    "frame_id": "camera_optical",
    "odom_frame_id": "vo_odom",
    "publish_tf": False,
    "approx_sync": False,
    "sync_queue_size": 10,
    "qos": 2,  # best effort, like the camera topics
    "qos_camera_info": 2,
    "Odom/ResetCountdown": "1",  # lost -> restart at identity; vo_adapter re-anchors
    "wait_for_transform": 0.1,
}

RGBD = {
    **COMMON,
    "Odom/Strategy": "0",
    "Vis/FeatureType": "6",
    "Vis/MaxFeatures": "600",
    "Vis/MinInliers": "12",
    "Vis/MaxDepth": "6.0",
}

ICP = {
    **COMMON,
    "Odom/Strategy": "0",
    "Reg/Strategy": "1",
    "Icp/PointToPlane": "true",
    "Icp/VoxelSize": "0.05",
    "Icp/MaxCorrespondenceDistance": "0.3",
    "Icp/Iterations": "30",
    "Icp/PointToPlaneK": "10",
    "Icp/PointToPlaneRadius": "0",
    "Icp/MaxTranslation": "0.5",
    "Icp/CorrespondenceRatio": "0.3",
    "OdomF2M/ScanSubtractRadius": "0.05",
    "OdomF2M/ScanMaxSize": "8000",
}


def _nodes(context, *args, **kwargs):
    mode = context.perform_substitution(args[0])
    vo = context.perform_substitution(args[1]).lower() == "true"
    nodes = [Node(package="maverick_vo", executable="camera_unpack", output="screen")]
    if not vo:
        return nodes
    params = dict(ICP if mode == "icp" else RGBD)
    params.update(json.loads(os.environ.get("MAVERICK_VO_OVERRIDES", "{}")))
    camera = [
        ("rgb/image", "/maverick/camera/rgb"),
        ("depth/image", "/maverick/camera/depth"),
        ("rgb/camera_info", "/maverick/camera/info"),
    ]
    if mode == "icp":
        nodes.append(
            Node(
                package="rtabmap_util",
                executable="point_cloud_xyz",
                output="screen",
                parameters=[
                    {
                        "decimation": 1,
                        "max_depth": 6.0,
                        "min_depth": 0.2,
                        "voxel_size": 0.0,
                        "approx_sync": False,
                        "qos": 2,
                        "qos_camera_info": 2,
                    }
                ],
                remappings=[
                    ("depth/image", "/maverick/camera/depth"),
                    ("depth/camera_info", "/maverick/camera/info"),
                    ("cloud", "/maverick/vo/cloud"),
                ],
            )
        )
        nodes.append(
            Node(
                package="rtabmap_odom",
                executable="icp_odometry",
                name="icp_odometry",
                output="screen",
                parameters=[params],
                remappings=[("scan_cloud", "/maverick/vo/cloud"), ("odom", "/maverick/vo/odom")],
            )
        )
    else:
        nodes.append(
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                parameters=[params],
                remappings=[*camera, ("odom", "/maverick/vo/odom")],
            )
        )
    nodes.append(
        Node(
            package="maverick_vo",
            executable="vo_adapter",
            output="screen",
            parameters=[{"send_velocity": False}],
        )
    )
    return nodes


def generate_launch_description() -> LaunchDescription:
    from launch.substitutions import LaunchConfiguration as LC

    return LaunchDescription(
        [
            DeclareLaunchArgument("vo", default_value="true"),
            DeclareLaunchArgument("mode", default_value="icp"),
            OpaqueFunction(function=_nodes, args=[LC("mode"), LC("vo")]),
        ]
    )
