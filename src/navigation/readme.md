# Navigation Package

## Overview

The `Navigation` package is developed to enable frontier exploration while avoiding obstacles, and recover if needed.

The package includes a node for generating frontiers, a configuration file for Nav2, an XML file defining the behavior tree of the navigation system, and a node for localizing probes globally.

### Key Components

1. **`algolism.py`**: The script used to generate new frontiers.
2. **`nav2_params.yaml`**: A configuration file for the entire Nav2 stack.
3. **`navigate_to_pose_w_replanning_and_recovery.xml`**: A configuration file for defining the behavior tree of the navigation system.
4. **`probe_filtering.cpp`**: A node responsible for localizing the detected probes globally.
---

## Package Structure

### 1. **`assets` Folder**
Contains a model of a probe that is used for visualization in RViz.

### 2. **`config` Folder**
Contains the three first scripts mentioned in key Components and a config file for what parts of the Nav2 stack is launched.

### 3. **`launch` Folder**
Contains two launch scipts. One for launching the navigation system and the other for launching the cmd_repeater and gpio_node handling the status LED.

### 4. **`scripts` Folder**
Contains the cmd_repeater responsible for republishing the velocity commands from the navigation system to the Leo Rover.

### 5. **`src` Folder**
Contains the probe_filtering script

### 6. **`urdf` Folder**
This folder contains xacro file of the modelled Leo Rover


