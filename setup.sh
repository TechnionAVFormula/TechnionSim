#!/bin/bash

# Determine platform based on machine architecture
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    export PLATFORM="linux/arm64"
else
    export PLATFORM="linux/amd64"
fi

# Export DISPLAY for X11 forwarding
export DISPLAY=:0

# Run docker-compose
docker-compose up --build
