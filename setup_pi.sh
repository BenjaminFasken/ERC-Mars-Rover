#!/bin/bash

# Exit on any error
set -e

# Define workspace path and user
USER="pi"  # Change to "leorover" if that's the correct user
WORKSPACE_DIR="/home/$USER/ERC-Mars-Rover"
SERVICE_NAME="mars-rover-nodes.service"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"
LAUNCH_FILE="$WORKSPACE_DIR/src/navigation/launch/pi.launch.py"
UDEV_RULE_FILE="/etc/udev/rules.d/99-gpio.rules"

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

# Ensure user is in the gpio group for non-root GPIO access
echo "Adding user $USER to gpio group..."
if ! groups "$USER" | grep -q "gpio"; then
    sudo usermod -aG gpio "$USER" || {
        echo "Error: Failed to add $USER to gpio group."
        exit 1
    }
    echo "User $USER added to gpio group. You may need to log out and back in or reboot for changes to take effect."
else
    echo "User $USER is already in gpio group."
fi

# Create udev rule for GPIO access
echo "Creating udev rule for GPIO access..."
sudo bash -c "cat > $UDEV_RULE_FILE" << EOL
SUBSYSTEM=="gpio*", PROGRAM="/bin/sh -c 'chown -R root:gpio /sys/class/gpio && chmod -R ug+rw /sys/class/gpio'"
EOL
sudo chmod 644 "$UDEV_RULE_FILE"
sudo udevadm control --reload-rules || {
    echo "Error: Failed to reload udev rules."
    exit 1
}
sudo udevadm trigger || {
    echo "Error: Failed to trigger udev rules."
    exit 1
}
echo "Udev rule created at $UDEV_RULE_FILE."

# Check if pigpiod is needed and enable it (optional, uncomment if using pigpio)
# echo "Enabling and starting pigpiod service for pigpio library..."
# sudo systemctl enable pigpiod || {
#     echo "Error: Failed to enable pigpiod service."
#     exit 1
# }
# sudo systemctl start pigpiod || {
#     echo "Error: Failed to start pigpiod service."
#     exit 1
# }

# Source ROS 2 setup file (assumes ROS 2 Humble)
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
# Uncomment the following line if using pigpiod
# After=network-online.target pigpiod.service
# Wants=network-online.target pigpiod.service

[Service]
Type=simple
User=$USER
Group=gpio
Environment="HOME=/home/$USER"
WorkingDirectory=$WORKSPACE_DIR
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /home/$USER/ERC-Mars-Rover/install/setup.bash && ros2 launch navigation pi.launch.py"
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

echo "Setup complete! The interfaces, gpio_controller, and navigation packages are built, GPIO permissions are configured, and $SERVICE_NAME is enabled and active."