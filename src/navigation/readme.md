# probe detection Package

## Overview

The `probe detection` package is designed to find the probes in its FOV, and determine their local position.

The package includes one main node and two other support nodes, which are used to campture datasets.

### Key Components

1. **`detect_probe.py`**: The main program, which is run to detect probes
2. **`Capture_test_set.py`**: A node that captures a test set of images and corresponding pointcloud for the probe detection algorithm.
3. **`ImageCaptureNode.py`**: Used to capure only images from the camera. This node is used to capture images for the training set of the probe detection algorithm.
---

## Package Structure

### 1. **`models` Folder**
Contains the trained models for the probe detection algorithm. Six models are included, in a varity of different sizes.
The YOLO11m.pt model is the one being used in the current implementation. The other models are included for future use, and to allow for testing of different models.

### 2. **`scripts` Folder**
Contains the scripts mentioned in key Components.

### 3. **`training` Folder**
The training folder host scripts that lets one train models.

### 4. **`verify_system` Folder**
Hold some scripts that can be used to verify that the system can correctly detect probes in its local frame. 

## Dependencies

The `probe_detection` package has the following dependencies:
1. **Core Dependencies**:
   - `interfaces` package
   Other dependencies are installed in the docker file.

