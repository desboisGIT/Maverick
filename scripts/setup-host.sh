#!/usr/bin/env bash
# One-time host setup for Maverick (Ubuntu 26.04): Docker, Compose, NVIDIA Container Toolkit, just.
# Run it yourself:  sudo bash scripts/setup-host.sh
# Then log out and back in (or run `newgrp docker`) so your user can use Docker without sudo.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo: sudo bash $0" >&2
  exit 1
fi
TARGET_USER="${SUDO_USER:-}"

echo "==> Docker, Compose and just from the Ubuntu archive"
apt-get update
apt-get install -y docker.io docker-compose-v2 docker-buildx just curl gpg

echo "==> NVIDIA Container Toolkit (NVIDIA's apt repository)"
KEYRING=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor --yes -o "$KEYRING"
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed "s#deb https://#deb [signed-by=$KEYRING] https://#g" \
  > /etc/apt/sources.list.d/nvidia-container-toolkit.list
apt-get update
apt-get install -y nvidia-container-toolkit
nvidia-ctk runtime configure --runtime=docker

systemctl enable --now docker
systemctl restart docker

if [[ -n "$TARGET_USER" ]]; then
  # Note: membership of the docker group is equivalent to root access on this machine.
  usermod -aG docker "$TARGET_USER"
  echo "==> Added $TARGET_USER to the docker group (log out and back in to apply)"
fi

echo "==> Checking GPU access from a container"
docker run --rm --gpus all ubuntu:24.04 nvidia-smi -L

echo "Done."
