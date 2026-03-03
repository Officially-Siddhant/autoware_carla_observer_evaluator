#!/bin/bash

# determine directory structure
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

# Building Dockerfile
echo "Building Dockerfile"
docker build -t gitlab.lrz.de:5005/safety/autoware_carla_leaderboard:0.9.16 --progress=plain -f "$SCRIPT_DIR"/Dockerfile --no-cache "$PARENT_DIR"
