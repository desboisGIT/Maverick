"""Topic names of the S3 host stack (one place; no ROS imports).

Inputs (from the bridge, S3 plan "Integration contract"): /maverick/camera/{rgb/compressed, depth,
semantic, info, pose}. Everything below is produced by the S3 host nodes.
"""

CAM_RGB = "/maverick/camera/rgb/compressed"
CAM_DEPTH = "/maverick/camera/depth"
CAM_SEMANTIC = "/maverick/camera/semantic"
CAM_INFO = "/maverick/camera/info"
CAM_POSE = "/maverick/camera/pose"

MAP_GRID = "/maverick/map/grid"  # nav_msgs/OccupancyGrid, 1 Hz, latched
MAP_VOXELS = "/maverick/map/voxels"  # String JSON voxel delta, <= 5 Hz
MAP_VOXELS_REQUEST = "/maverick/map/voxels_request"  # std_msgs/Empty: please send a reset snapshot
MAP_FRONTIERS = "/maverick/map/frontiers"  # String JSON list, 1 Hz, latched

MEM_DETECTIONS = "/maverick/memory/detections"  # String JSON, one per processed frame
MEM_ROOMS = "/maverick/memory/rooms"  # String JSON list (gateway room format), latched
MEM_OBJECTS = "/maverick/memory/objects"  # String JSON list (gateway object format), latched
MEM_QUERY = "/maverick/memory/query"  # String JSON request
MEM_ANSWER = "/maverick/memory/answer"  # String JSON reply (echoes "id")

SKILL_REQUEST = "/maverick/skill/request"  # String JSON {"skill": "explore"|"goto_room"|"cancel"}
SKILL_PROGRESS = "/maverick/skill/progress"  # String JSON (gateway skill_progress), latched
SKILL_PATH = "/maverick/skill/path"  # String JSON (gateway path), latched

FLIGHT_GOAL = "/maverick/flight/goal"
FLIGHT_STATUS = "/maverick/flight/status"
FLIGHT_MISSION = "/maverick/flight/mission"
