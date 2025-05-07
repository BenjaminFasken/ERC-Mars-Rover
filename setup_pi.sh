#!/bin/bash

# Exit on any error
set -e

# Define workspace path and user
USER="pi"  # Change to "leorover" if that's the correct user
WORKSPACE_DIR="/home/$USER/ERC-Mars-Rover"
SERVICE_NAME="mars-rover-nodes.service"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"
LAUNCH_FILE="$WORKSPACE_DIR/src/navigation/launch/pi.launch.py"

# Check if running user matches expected user
if [ "$(whoami)" != "$USER" ]; then
    echo "Error: Script must be run as user $USER (current user: $(whoami))."
    exit 1
fi

# Check if workspace directory exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Error: Workspace directory $WORKSPACE_DIR does not exist."
    exit 1
fi

# Source ROS 2 setup file (assumes ROS 2 Humble)
source /opt/ros/humble/setup.bash || {
    echo "Error: Failed to source ROS 2 setup.bash. Ensure ROS 2 is installed."
    exit 1
}

# Install dependencies
echo "Installing dependencies..."
cd "$WORKSPACE_DIR"
rosdep install --from-paths src --ignore-src -r -y || {
    echo "Error: Failed to install dependencies."
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

# Check if the launch file exists, create if it doesn't
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "Creating launch file $LAUNCH_FILE..."
    mkdir -p "$WORKSPACE_DIR/src/navigation/launch"
    cat > "$LAUNCH_FILE" << EOL
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='cmd_repeater.py',
            name='cmd_repeater',
            output='screen'
        ),
        Node(
            package='gpio_controller',
            executable='gpio_node.py',
            name='gpio_node',
            output='screen'
        )
    ])
EOL
    echo "Created $LAUNCH_FILE."
else
    echo "Launch file $LAUNCH_FILE already exists."
fi

# Create or update systemd service file
echo "Creating/updating systemd service file $SERVICE_FILE..."
sudo bash -c "cat > $SERVICE_FILE" << EOL
[Unit]
Description=ROS 2 Mars Rover Startup Nodes
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER
Environment="HOME=/home/$USER"
WorkingDirectory=$WORKSPACE_DIR
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/$USER/ERC-Mars-Rover/install/setup.bash && ros2 launch navigation pi.launch.py"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOL

# Set permissions for service file
sudo chmod 644 "$SERVICE_FILE"

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

# Stop the service if it's running (to ensure clean start)
sudo systemctl stop "$SERVICE_NAME" || true

# Start the service
echo "Starting $SERVICE_NAME..."
sudo systemctl start "$SERVICE_NAME" || {
    echo "Error: Failed to start $SERVICE_NAME."
    journalctl -u "$SERVICE_NAME" -n 50 --no-pager
    exit 1
}

# Check service status
echo "Checking status of $SERVICE_NAME..."
if ! sudo systemctl is-active --quiet "$SERVICE_NAME"; then
    echo "Error: $SERVICE_NAME is not active."
    sudo systemctl status "$SERVICE_NAME" --no-pager
    journalctl -u "$SERVICE_NAME" -n 50 --no-pager
    exit 1
fi
sudo systemctl status "$SERVICE_NAME" --no-pager

echo "Setup complete! The interfaces, gpio_controller, and navigation packages are built, and $SERVICE_NAME is enabled and active."