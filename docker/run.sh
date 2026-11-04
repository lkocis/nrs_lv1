#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if docker container inspect nrs_lv1 >/dev/null 2>&1; then
    docker start nrs_lv1 >/dev/null
    exec docker exec -it nrs_lv1 bash
fi

exec docker run -it \
    --name nrs_lv1 \
    --network host \
    --gpus all \
    --shm-size 1g \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-31}" \
    -e XAUTHORITY=/tmp/nrs.xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/nrs.xauth:ro" \
    -v "${PROJECT_ROOT}:/workspaces/nrs_lv1" \
    nrs_lv1:latest bash
