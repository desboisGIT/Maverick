# syntax=docker/dockerfile:1
#
# Maverick image (S0 simulator, S1 adds the link tooling), serves sim, bridge, host and referee.
# PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy.
# Build:  docker compose -f docker/compose.yaml build sim
#
# ---------------------------------------------------------------------------
# Pinned versions and where each was verified (checked 2026-11-11)
# ---------------------------------------------------------------------------
# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic: PX4 v1.17 Tools/setup/ubuntu.sh
#   installs "gz-harmonic" on 24.04
#   https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.17.0/Tools/setup/ubuntu.sh
#   Jazzy install: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# PX4 v1.17.0 is the newest v1.17.x tag (https://github.com/PX4/PX4-Autopilot/tags).
# Micro XRCE-DDS Agent v2.4.3: the tag used in the PX4 v1.17 ROS 2 guide
#   https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.17.0/docs/en/ros2/user_guide.md
#   (docs.px4.io/v1.17/en/ros2/user_guide.html)
# px4_msgs v1.17.0: tag matching the firmware (px4_msgs has tags/branches per
#   release; the ROS 2 guide says msgs must match the firmware).
#
# Message versioning (PX4 >= 1.16, docs/en/middleware/uorb.md and
# docs/en/ros2/px4_ros2_msg_translation_node.md): in v1.17.0 the firmware's
# uXRCE-DDS client still publishes UNVERSIONED topic names
# (dds_topics.yaml: /fmu/out/vehicle_status, /fmu/out/vehicle_local_position,
# /fmu/in/trajectory_setpoint, ...). The "_v<N>" suffix (e.g.
# /fmu/out/vehicle_status_v1) only appears when the optional ROS 2 message
# translation node is run. We build px4_msgs from the SAME tag as the firmware,
# so no translation node is needed and topics have no suffix.
# ---------------------------------------------------------------------------
ARG UBUNTU_VERSION=24.04
FROM ubuntu:${UBUNTU_VERSION}

ARG ROS_DISTRO=jazzy
ARG PX4_VERSION=v1.17.0
ARG XRCE_AGENT_VERSION=v2.4.3
ARG PX4_MSGS_REF=v1.17.0
# Parallel compile jobs. Lower it if the build runs out of RAM.
ARG BUILD_JOBS=8

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    PX4_DIR=/opt/PX4-Autopilot \
    GZ_DISTRO=harmonic \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# --- base tools -------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl wget git gnupg lsb-release locales procps \
    && rm -rf /var/lib/apt/lists/*

# --- PX4-Autopilot v1.17.x, prebuilt for px4_sitl ----------------------------
# ubuntu.sh --no-nuttx: common deps, Python deps and Gazebo Harmonic, no NuttX
# toolchain. RUNS_IN_DOCKER=true makes it install its own prerequisites (sudo...).
# Tools/simulation/gz (models/worlds) is a git submodule, hence --recurse-submodules.
RUN git clone --depth 1 --branch ${PX4_VERSION} --recurse-submodules --shallow-submodules \
        --jobs 8 https://github.com/PX4/PX4-Autopilot.git ${PX4_DIR} \
    && RUNS_IN_DOCKER=true bash ${PX4_DIR}/Tools/setup/ubuntu.sh --no-nuttx \
    && rm -rf /var/lib/apt/lists/* /root/.cache/pip

# `make px4_sitl_default` builds the px4 binary and the Gazebo plugins
# (px4_gz_plugins is an ALL target). gz_x500 itself is only a run target.
RUN cd ${PX4_DIR} \
    && make px4_sitl_default -j${BUILD_JOBS} \
    && test -x build/px4_sitl_default/bin/px4 \
    && test -f build/px4_sitl_default/rootfs/gz_env.sh \
    && rm -rf /root/.ccache

# --- ROS 2 Jazzy -------------------------------------------------------------
# Repo setup follows the manual key method; the docs now also offer the
# ros2-apt-source package, which needs a "latest release" lookup, so not used.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# --- Micro XRCE-DDS Agent (standalone build, installed to /usr/local) --------
RUN git clone --depth 1 --branch ${XRCE_AGENT_VERSION} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
    && cmake -S /tmp/xrce -B /tmp/xrce/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/xrce/build -j${BUILD_JOBS} \
    && cmake --install /tmp/xrce/build \
    && ldconfig \
    && rm -rf /tmp/xrce \
    && test -x /usr/local/bin/MicroXRCEAgent

# --- px4_msgs underlay at /opt/px4_ws ----------------------------------------
RUN mkdir -p /opt/px4_ws/src \
    && git clone --depth 1 --branch ${PX4_MSGS_REF} https://github.com/PX4/px4_msgs.git /opt/px4_ws/src/px4_msgs \
    && cd /opt/px4_ws \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} colcon build --packages-select px4_msgs \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/px4_ws/build /opt/px4_ws/log

# --- runtime user and scripts -------------------------------------------------
# ubuntu:24.04 already ships user "ubuntu" with uid/gid 1000, matching the host
# user, so files written to the /maverick bind mount stay owned by uid 1000.
# PX4 writes parameters/logs into its rootfs at runtime.
RUN chown -R 1000:1000 ${PX4_DIR}/build/px4_sitl_default/rootfs \
    && mkdir -p /maverick && chown 1000:1000 /maverick

# --- stage S1: link, netem, referee tooling ---------------------------------
# zenoh-bridge-ros2dds 1.10.1: release of 2026-09-07, newest before the 2026-10-28 1.10.2
#   https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases (standalone x86_64 zip)
# Docker CLI 27.5.1 static binary (referee fault injection via the mounted socket):
#   https://download.docker.com/linux/static/stable/x86_64/
# iproute2 + libcap2-bin: `tc netem`; tc gets cap_net_admin as a file capability because the
#   image user (uid 1000) is not root and compose only adds the cap to the bounding set.
# python3-gz-transport13 (Gazebo Harmonic Python bindings for ground truth) is already
#   installed by PX4's ubuntu.sh.
ARG ZENOH_ROS2DDS_VERSION=1.10.1
ARG DOCKER_CLI_VERSION=27.5.1
RUN apt-get update && apt-get install -y --no-install-recommends unzip iproute2 libcap2-bin \
    && rm -rf /var/lib/apt/lists/* \
    && curl -fsSL -o /tmp/zenoh.zip https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_ROS2DDS_VERSION}/zenoh-plugin-ros2dds-${ZENOH_ROS2DDS_VERSION}-x86_64-unknown-linux-gnu-standalone.zip \
    && unzip -o /tmp/zenoh.zip -d /usr/local/bin zenoh-bridge-ros2dds \
    && chmod 0755 /usr/local/bin/zenoh-bridge-ros2dds && rm /tmp/zenoh.zip \
    && curl -fsSL https://download.docker.com/linux/static/stable/x86_64/docker-${DOCKER_CLI_VERSION}.tgz \
        | tar -xz -C /usr/local/bin --strip-components=1 docker/docker \
    && setcap cap_net_admin+ep "$(readlink -f "$(command -v tc)")" \
    && zenoh-bridge-ros2dds --version && docker --version

# Lint tool for `just lint` (not in Ubuntu 24.04's archive at a useful version).
RUN python3 -m pip install --no-cache-dir --break-system-packages ruff

# --- stage S3: camera driver (sim container) ---------------------------------
# The driver needs gz-transport13 + gz-msgs10 Python bindings (from PX4's ubuntu.sh, apt),
# numpy (downsampling) and Pillow (JPEG). The last two are already pulled in by PX4's Python
# requirements; listing them keeps the dependency explicit (no-op when satisfied).
RUN python3 -m pip install --no-cache-dir --break-system-packages numpy pillow \
    && python3 -c "import numpy, PIL.Image; from gz.transport13 import Node; from gz.msgs10.image_pb2 import Image"

# --- stage S3b: host-side RGB-D visual odometry (rtabmap) ----------------------------------
# ros-jazzy-rtabmap-odom 0.23.13 (checked 2026-11-11, packages.ros.org Jazzy): rgbd_odometry,
# icp_odometry, stereo_odometry. Runs in the host container, fed by the bridge's camera stream;
# the result goes back to PX4 as external vision (docs/s3-plan.md "Visual odometry").
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rtabmap-odom \
    && rm -rf /var/lib/apt/lists/*

# --- stage S4a: agent backends (DeepSeek via openai SDK = default, Claude via anthropic SDK);
# both imported lazily, the scripted backend needs nothing ---
RUN python3 -m pip install --no-cache-dir --break-system-packages anthropic openai \
    && python3 -c "import anthropic, openai"

COPY docker/profile.d-maverick.sh /etc/profile.d/maverick.sh
COPY docker/sim-entrypoint.sh docker/bridge-entrypoint.sh docker/host-entrypoint.sh /usr/local/bin/
RUN chmod 0644 /etc/profile.d/maverick.sh \
    && chmod 0755 /usr/local/bin/sim-entrypoint.sh /usr/local/bin/bridge-entrypoint.sh \
        /usr/local/bin/host-entrypoint.sh \
    && echo '[ -f /etc/profile.d/maverick.sh ] && . /etc/profile.d/maverick.sh' >> /etc/bash.bashrc

USER 1000:1000
ENV HOME=/home/ubuntu
WORKDIR /maverick
ENTRYPOINT ["/usr/local/bin/sim-entrypoint.sh"]
