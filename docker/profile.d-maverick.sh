# Installed as /etc/profile.d/maverick.sh (also sourced from /etc/bash.bashrc).
# Gives `docker compose exec sim bash -lc "<cmd>"` ROS 2, the px4_msgs underlay
# and the repo's own workspace, if it has been built.
if [ -n "${BASH_VERSION:-}" ]; then
    [ -f /opt/ros/jazzy/setup.bash ] && . /opt/ros/jazzy/setup.bash
    [ -f /opt/px4_ws/install/setup.bash ] && . /opt/px4_ws/install/setup.bash
    [ -f /maverick/ros2_ws/install/setup.bash ] && . /maverick/ros2_ws/install/setup.bash
fi
