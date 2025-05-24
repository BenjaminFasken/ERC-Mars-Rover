1. install necessary programs to run:
```
sudo apt-get install ros-humble-rosbridge-server
pip install pywebview
pip install python-nmap
pip install requests
```

2. Then you can launch the whole thing after a colcon build and source:
```
ros2 launch gcs gcs.launch.py
```
DO NOT RUN THIS IN A VSCODE TERMINAL
-run it a standard terminal

Make sure both you and the rover are connected to the same network and subnet.

If you have ROS2 connection issues, make sure to check your dds configuration of either fastrts or cyclone.

If problems persist, we have found that a reboot of pc or router often does the trick.

