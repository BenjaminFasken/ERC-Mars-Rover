#!/bin/bash

# Exit on any error
set -e

# Define workspace path and user
USER="pi"  # Change to "leorover" if that's the correct user
WORKSPACE_DIR="/home/$USER/ERC-Mars-Rover"
CMD_REPEATER_SERVICE="mars-rover-cmd-repeater.service"
GPIO_NODE_SERVICE="mars-rover-gpio-node.service"
CMD_REPEATER_SERVICE_FILE="/etc/systemd/system/$CMD_REPEATER_SERVICE"
GPIO_NODE_SERVICE_FILE="/etc/systemd/system/$GPIO_NODE_SERVICE"
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

# Create systemd service file for cmd_repeater (Cyclone DDS)
echo "Creating/updating systemd service file $CMD_REPEATER_SERVICE_FILE..."
sudo bash -c "cat > $CMD_REPEATER_SERVICE_FILE" << EOL
[Unit]
Description=ROS 2 Mars Rover cmd_repeater Node (Cyclone DDS)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER
Group=gpio
Environment="HOME=/home/$USER"
WorkingDirectory=$WORKSPACE_DIR
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /home/$USER/ERC-Mars-Rover/install/setup.bash && ros2 run navigation cmd_repeater.py"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOL

# Create systemd service file for gpio_node (Fast RTPS)
echo "Creating/updating systemd service file $GPIO_NODE_SERVICE_FILE..."
sudo bash -c "cat > $GPIO_NODE_SERVICE_FILE" << EOL
[Unit]
Description=ROS 2 Mars Rover gpio_node (Fast RTPS)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER
Group=gpio
Environment="HOME=/home/$USER"
WorkingDirectory=$WORKSPACE_DIR
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && source /home/$USER/ERC-Mars-Rover/install/setup.bash && ros2 run gpio_controller gpio_node.py"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOL

# Set permissions for service files
sudo chmod 644 "$CMD_REPEATER_SERVICE_FILE"
sudo chmod 644 "$GPIO_NODE_SERVICE_FILE"

# Reload systemd daemon
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload || {
    echo "Error: Failed to reload systemd daemon."
    exit 1
}

# Enable the services
echo "Enabling $CMD_REPEATER_SERVICE and $GPIO_NODE_SERVICE..."
sudo systemctl enable "$CMD_REPEATER_SERVICE" || {
    echo "Error: Failed to enable $CMD_REPEATER_SERVICE."
    exit 1
}
sudo systemctl enable "$GPIO_NODE_SERVICE" || {
    echo "Error: Failed to enable $GPIO_NODE_SERVICE."
    exit 1
}

# Stop the services if they're running (to ensure clean start)
sudo systemctl stop "$CMD_REPEATER_SERVICE" || true
sudo systemctl stop "$GPIO_NODE_SERVICE" || true

# Start the services
echo "Starting $CMD_REPEATER_SERVICE..."
sudo systemctl start "$CMD_REPEATER_SERVICE" || {
    echo "Error: Failed to start $CMD_REPEATER_SERVICE."
    journalctl -u "$CMD_REPEATER_SERVICE" -n 50 --no-pager
    exit 1
}
echo "Starting $GPIO_NODE_SERVICE..."
sudo systemctl start "$GPIO_NODE_SERVICE" || {
    echo "Error: Failed to start $GPIO_NODE_SERVICE."
    journalctl -u "$GPIO_NODE_SERVICE" -n 50 --no-pager
    exit 1
}

# Check service status
echo "Checking status of $CMD_REPEATER_SERVICE..."
if ! sudo systemctl is-active --quiet "$CMD_REPEATER_SERVICE"; then
    echo "Error: $CMD_REPEATER_SERVICE is not active."
    sudo systemctl status "$CMD_REPEATER_SERVICE" --no-pager
    journalctl -u "$CMD_REPEATER_SERVICE" -n 50 --no-pager
    exit 1
fi
sudo systemctl status "$CMD_REPEATER_SERVICE" --no-pager

echo "Checking status of $GPIO_NODE_SERVICE..."
if ! sudo systemctl is-active --quiet "$GPIO_NODE_SERVICE"; then
    echo "Error: $GPIO_NODE_SERVICE is not active."
    sudo systemctl status "$GPIO_NODE_SERVICE" --no-pager
    journalctl -u "$GPIO_NODE_SERVICE" -n 50 --no-pager
    exit 1
fi
sudo systemctl status "$GPIO_NODE_SERVICE" --no-pager

echo "Setup complete! The interfaces, gpio_controller, and navigation packages are built, GPIO permissions are configured, and $CMD_REPEATER_SERVICE (Cyclone DDS) and $GPIO_NODE_SERVICE (Fast RTPS) are enabled and active."