#!/bin/bash

# Verify that the script is being run from the workspace root
if [ ! -d "install" ]; then
    echo "Error: This script must be run from the workspace root (missing 'install' directory)."
    exit 1
fi

# Define Yamcs Studio installation directory and version
YAMCS_VERSION="1.7.7"
YAMCS_TAR="yamcs-studio-${YAMCS_VERSION}-linux.gtk.x86_64.tar.gz"
YAMCS_URL="https://github.com/yamcs/yamcs-studio/releases/download/v${YAMCS_VERSION}/${YAMCS_TAR}"
YAMCS_INSTALL_DIR="${HOME}/yamcs-studio/yamcs-studio-${YAMCS_VERSION}"
YAMCS_EXECUTABLE="${YAMCS_INSTALL_DIR}/Yamcs Studio"
YAMCS_GUI_DIR="${YAMCS_INSTALL_DIR}/gui"

# Define GUI files directory
GUI_FILES_DIR="$(pwd)/src/gcs/gcs_gui"

# Define RViz2 config file
RVIZ_CONFIG="$(pwd)/src/gcs/rviz2/rviz-config.rviz"

# Check if Yamcs Studio is installed and executable
if [ ! -x "${YAMCS_EXECUTABLE}" ]; then
    echo "Yamcs Studio not found or not executable at ${YAMCS_EXECUTABLE}."
    echo "Installing Yamcs Studio ${YAMCS_VERSION}..."

    # Create installation directory
    mkdir -p "${YAMCS_INSTALL_DIR}" || {
        echo "Error: Failed to create directory ${YAMCS_INSTALL_DIR}."
        exit 1
    }

    # Download Yamcs Studio
    echo "Downloading ${YAMCS_TAR}..."
    wget -O "/tmp/${YAMCS_TAR}" "${YAMCS_URL}" || {
        echo "Error: Failed to download Yamcs Studio from ${YAMCS_URL}."
        exit 1
    }

    # Extract the tarball
    echo "Extracting ${YAMCS_TAR} to ${YAMCS_INSTALL_DIR}..."
    tar -xzf "/tmp/${YAMCS_TAR}" -C "${YAMCS_INSTALL_DIR}" --strip-components=1 || {
        echo "Error: Failed to extract ${YAMCS_TAR}."
        exit 1
    }

    # Clean up
    rm -f "/tmp/${YAMCS_TAR}"

    # Verify installation
    if [ ! -x "${YAMCS_EXECUTABLE}" ]; then
        echo "Error: Yamcs Studio installation failed. Executable not found at ${YAMCS_EXECUTABLE}."
        exit 1
    fi
    echo "Yamcs Studio ${YAMCS_VERSION} installed successfully at ${YAMCS_INSTALL_DIR}."
else
    echo "Yamcs Studio already installed at ${YAMCS_EXECUTABLE}."
fi

# Check if the GUI files directory exists
if [ ! -d "${GUI_FILES_DIR}" ]; then
    echo "Error: GUI files directory ${GUI_FILES_DIR} not found."
    exit 1
fi

# Create the gui directory in Yamcs Studio if it doesn't exist
mkdir -p "${YAMCS_GUI_DIR}" || {
    echo "Error: Failed to create GUI directory ${YAMCS_GUI_DIR}."
    exit 1
}

# Check if GUI files are already present in Yamcs Studio
if [ -n "$(ls -A "${YAMCS_GUI_DIR}")" ]; then
    echo "GUI files already present in ${YAMCS_GUI_DIR}. Skipping copy."
else
    # Copy GUI files to Yamcs Studio
    echo "Copying GUI files to Yamcs Studio..."
    cp -r "${GUI_FILES_DIR}"/* "${YAMCS_GUI_DIR}/" || {
        echo "Error: Failed to copy GUI files to ${YAMCS_GUI_DIR}."
        exit 1
    }

    # Check if GUI files were copied
    if [ -z "$(ls -A "${YAMCS_GUI_DIR}")" ]; then
        echo "Error: No GUI files found in ${YAMCS_GUI_DIR} after copying."
        exit 1
    fi
    echo "GUI files copied successfully to ${YAMCS_GUI_DIR}."
fi

# Add Yamcs Studio to PATH for this session (optional, remove if not needed)
export PATH="${YAMCS_INSTALL_DIR}:${PATH}"

# Source the ROS 2 workspace
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble setup file not found at /opt/ros/humble/setup.bash."
    exit 1
fi

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: Workspace setup file not found at install/setup.bash."
    exit 1
fi

# Function to clean up all processes
cleanup() {
    echo "Terminating all processes..."
    for pid in $YAMCS_PID $RVIZ_PID $GCS_PID; do
        if [ -n "$pid" ] && ps -p $pid > /dev/null; then
            kill -TERM $pid 2>/dev/null
        fi
    done
    exit 0
}

# Trap interrupts to ensure cleanup
trap cleanup SIGINT SIGTERM

# Run the GCS ROS2 launch file
echo "Running GCS launch file..."
ros2 launch gcs gui_server.launch.py &
GCS_PID=$!
sleep 5
if ! ps -p $GCS_PID > /dev/null; then
    echo "Error: GCS launch file failed to start."
    cleanup
fi
echo "GCS launch is running with PID ${GCS_PID}."

# Run Yamcs Studio
echo "Running Yamcs Studio..."
"${YAMCS_EXECUTABLE}" &
YAMCS_PID=$!
sleep 5
if ! ps -p $YAMCS_PID > /dev/null; then
    echo "Error: Yamcs Studio failed to start."
    cleanup
fi
echo "Yamcs Studio is running with PID ${YAMCS_PID}."

# Run RViz2 with config file if it exists
echo "Running RViz2..."
if [ -f "${RVIZ_CONFIG}" ]; then
    ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" &
else
    echo "Warning: RViz config file ${RVIZ_CONFIG} not found, launching RViz with default settings."
    ros2 run rviz2 rviz2 &
fi
RVIZ_PID=$!
sleep 5
if ! ps -p $RVIZ_PID > /dev/null; then
    echo "Error: RViz failed to start."
    cleanup
fi
echo "RViz is running with PID ${RVIZ_PID}."

# Monitor processes and terminate all if any one exits
while true; do
    if ! ps -p $YAMCS_PID > /dev/null; then
        echo "Yamcs Studio terminated. Closing all processes."
        cleanup
    fi
    if ! ps -p $RVIZ_PID > /dev/null; then
        echo "RViz terminated. Closing all processes."
        cleanup
    fi
    if ! ps -p $GCS_PID > /dev/null; then
        echo "GCS launch terminated. Closing all processes."
        cleanup
    fi
    sleep 1
done