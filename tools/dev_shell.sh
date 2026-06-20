#!/usr/bin/env bash
# Drop into the MiniBot dev container with the repo mounted at /ws.
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker build -t minibot-dev:latest -f "${REPO_ROOT}/deploy/docker/Dockerfile.dev" "${REPO_ROOT}"
docker run --rm -it \
  --network host \
  -v "${REPO_ROOT}:/ws" \
  -e ROS_DOMAIN_ID=42 \
  minibot-dev:latest bash
