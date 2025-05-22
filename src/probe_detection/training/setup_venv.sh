#!/bin/bash

# Update package lists
sudo apt-get update

# Install required system packages
sudo apt-get install -y python3-venv python3-pip python3-dev

# Create virtual environment
python3 -m venv ucloud_venv

# Activate environment
source ucloud_venv/bin/activate

# Upgrade pip and install requirements
pip install --upgrade pip
pip install -r requirements.txt

echo "Virtual environment setup complete!"
echo "Activate with: source ucloud_venv/bin/activate"
