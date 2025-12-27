#!/bin/bash
set -e

# Resolve the directory this script is in
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Use paths relative to the script
DOCKER_COMPOSE_FILE="$SCRIPT_DIR/docker-compose.daq.yml"

# Build and start the DAQ container
docker-compose -f "$DOCKER_COMPOSE_FILE" build
docker-compose -f "$DOCKER_COMPOSE_FILE" up -d

# Optional: systemd service setup
SERVICE_FILE="/etc/systemd/system/ros2-daq.service"
if [ ! -f "$SERVICE_FILE" ]; then
    sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=ROS2 DAQ Container
After=docker.service
Requires=docker.service

[Service]
Restart=always
ExecStart=/usr/local/bin/docker-compose -f $DOCKER_COMPOSE_FILE up
ExecStop=/usr/local/bin/docker-compose -f $DOCKER_COMPOSE_FILE down
WorkingDirectory=$SCRIPT_DIR
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable ros2-daq.service
fi

# Start service
sudo systemctl start ros2-daq.service
