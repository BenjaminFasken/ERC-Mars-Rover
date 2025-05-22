#!/usr/bin/env python3
import numpy as np
from pathlib import Path

rotation = 20

def create_transformation_matrix():    
    # make a -20 degree rotation around the y-axis
    rotation_y = np.array([
        [np.cos(np.radians(-rotation)), 0, np.sin(np.radians(-rotation)), 0],
        [0, 1, 0, 0],
        [-np.sin(np.radians(-rotation)), 0, np.cos(np.radians(-rotation)), 0],
        [0, 0, 0, 1]
    ])
    
    trans_vector = np.array([0.99150, 2.98070, -37.41000 - 5.5, 1.0]) / 100
    
    # Create a homogeneous transformation matrix
    T_m = np.eye(4)  # Start with an identity matrix
    T_m[:3, :3] = rotation_y[:3, :3]  # Set the rotation part
    T_m[:3, 3] = trans_vector[:3]  # Set the translation part
    

    # Define output path
    output_dir = Path("~/ERC-Mars-Rover/src/probe_detection/verify_system/data").expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "transformation_matrix.npy"

    # Save to .npy file
    np.save(output_path, T_m)
    print(f"Saved transformation matrix to {output_path}")

if __name__ == "__main__":
    create_transformation_matrix()