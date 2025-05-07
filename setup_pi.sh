#!/bin/bash

# Exit on any error
set -e

# Define workspace path
WORKSPACE_DIR="$HOME/ERC-Mars-Rover"
SERVICE_NAME="mars-rover-nodes.service"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"
LAUNCH_FILE="$WORKSPACE_DIR/src/navigation/launch/pi.launch.py"

# Check if workspace directory exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Error: Workspace directory $WORKSPACE_DIR does not exist."
    exit 1
fi

# Source ROS 2 setup file (adjust for your ROS 2 distribution, e.g., humble)
source /opt/ros/humble/setup.bash || {
    echo "Error: Failed to source ROS 2 setup.bash. Ensure ROS 2 is installed."
    exit 1
}

# Build specific packages
echo "Building interfaces, gpio_controller, and navigation packages..."
colcon build --packages-select interfaces gpio_controller navigation || {
    echo "Error: Failed to build packages."
    exit 1
}

# Source the workspace
source "$WORKSPACE_DIR/install/setup.bash"

# Set executable permissions for the scripts
echo "Setting executable permissions for scripts..."
chmod +x "$WORKSPACE_DIR/src/navigation/scripts/cmd_repeater.py" || {
    echo "Error: Failed to set permissions for cmd_repeater.py."
    exit 1
}
chmod +x "$WORKSPACE_DIR/src/gpio_controller/scripts/gpio_node.py" || {
    echo "Error: Failed to set permissions for gpio_node.py."
    exit 1
}

# Check if the launch file exists
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "Error: Launch file $LAUNCH_FILE does not exist. Please create it first."
    exit 1
fi

# Create systemd service file if it doesn't exist
if [ ! -f "$SERVICE_FILE" ]; then
    echo "Creating systemd service file $SERVICE_FILE..."
    sudo bash -c "cat > $SERVICE_FILE" << EOL
[Unit]
Description=ROS 2 Mars Rover Startup Nodes
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=leorover
WorkingDirectory=$WORKSPACE_DIR
ExecStart=/bin/bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch navigation pi.launch.py"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOL
else
    echo "Systemd service file $SERVICE_FILE already exists."
fi

# Reload systemd daemon
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload || {
    echo "Error: Failed to reload systemd daemon."
    exit 1
}

# Enable the service
echo "Enabling $SERVICE_NAME..."
sudo systemctl enable "$SERVICE_NAME" || {
    echo "Error: Failed to enable $SERVICE_NAME."
    exit 1
}

# Start the service
echo "Starting $SERVICE_NAME..."
sudo systemctl start "$SERVICE_NAME" || {
    echo "Error: Failed to start $SERVICE_NAME."
    exit 1
}

# Check service status
echo "Checking status of $SERVICE_NAME..."
sudo systemctl status "$SERVICE_NAME" --no-pager || {
    echo "Warning: $SERVICE_NAME is not running as expected."
}

echo "Setup complete! The interfaces, gpio_controller, and navigation packages are built, and $SERVICE_NAME is enabled and active."