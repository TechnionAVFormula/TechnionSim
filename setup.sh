#!/bin/bash

# Determine platform based on machine architecture
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    export PLATFORM="linux/arm64"
else
    export PLATFORM="linux/amd64"
fi

# ensures no dangling containers are blocking the port
docker stop $(docker ps -q --filter "publish=6080")

 docker build -t eufs_sim .

# Run docker-compose
docker run -d --privileged -p 6080:80 -v "$(pwd)/scripts/eufs_simulator.desktop:/home/ubuntu/Desktop/eufs_simulator.desktop"  eufs_sim:latest



