#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
docker build --file "${SCRIPT_DIR}/Dockerfile" \
    --build-arg "USER_UID=$(id -u)" --build-arg "USER_GID=$(id -g)" \
    --tag "${IMAGE_NAME:-nrs_lv1:latest}" "${SCRIPT_DIR}"
