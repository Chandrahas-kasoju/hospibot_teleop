# Hospibot Teleoperation

## Overview
This package provides a modern, web-based teleoperation interface for Hospibot.
It includes:
-   **Web Server**: Serves the frontend and streams video from `/dev/video0`.
-   **Frontend**: Modern dark-themed UI with virtual joystick and video feed.
-   **ROS Bridge**: Connects the web frontend to ROS 2.

## Prerequisites
-   ROS 2 (Humble/Iron/Jazzy)
-   `rosbridge_server` installed:
    ```bash
    sudo apt install ros-<distro>-rosbridge-server
    ```
-   Camera connected at `/dev/video0`.

## Build Instructions
```bash
cd ~/Hospibot/hospibot_teleop
colcon build --packages-select hospibot_teleop
source install/setup.bash
```

## Network Configuration (Optional)
To access the robot via `http://hospibot.local:8000`, configure Avahi:
1.  Edit `/etc/avahi/avahi-daemon.conf`:
    ```ini
    [server]
    host-name=hospibot
    ```
2.  Restart Avahi:
    ```bash
    sudo systemctl restart avahi-daemon
    ```

## Running the System
Launch the web server and rosbridge:
```bash
ros2 launch hospibot_teleop teleop.launch.py
```

## Accessing the Interface
1.  Open a web browser on a device connected to the same network.
2.  Navigate to `http://hospibot.local:8000` (or `http://<robot_ip>:8000`).
3.  **Connect**: Click the "Connect" button in the top right corner.
    -   This establishes the ROS connection and starts the camera feed.
4.  **Video**: The camera feed should appear automatically after connecting.
5.  **Control**: Use the on-screen joystick to move the robot.
6.  **Disconnect**: Click "Disconnect" to stop the stream and close the connection.

## Troubleshooting
-   **Camera not working**: Check if `/dev/video0` exists and is accessible.
-   **Connection refused**: Ensure the launch file is running and port 8000 is open.
-   **Address in use**: If `rosbridge` fails to start, check if another instance is running on port 9090.
