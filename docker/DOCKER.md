# 🐳 Docker #
This provides a ready-to-use **Ubuntu 24.04 + ROS 2 Jazzy** environment for SenseGlove development.

## Preparing the Environment ##
Before launching the containers, enable GUI forwarding on your host system so that any tools requiring X11 (e.g. RViz or PyQt GUIs) can be displayed
```
xhost +local:docker
```

Make the entrypoint scripts executable
```
chmod +x entrypoint.sh
```

## Building and Running the Docker Container ##

### Step 1: Build
```
docker compose build senseglove_ros
```

### Step 2: Run SenseCom on the Host ###
1. Pair and connect the gloves on the host (Ubuntu 22.04 or 24.04).
    - **BLE Gloves:** Pair in SenseCom and wait for connection
    - **Bluetooth Serial Gloves (legacy):** Use the provided [glove_connect.sh script](/senseglove_ros/senseglove/senseglove_bringup/scripts/glove_connect.sh) to bind each glove to a /dev/rfcomm* port. These devices will then be mounted inside the container.

2. Start SenseCom manually from the host:
```
ros2 run senseglove_com SenseCom.x86_64  
```
> The latest SenseCom supports both BLE and Bluetooth Serial. Firmware update is recommended to switch to BLE.

### Step 3: Run the ROS container ###
Start the container:
```
docker compose up senseglove_ros
```
On first run the workspace is built automatically. To enter a shell:
```
docker compose exec senseglove_ros bash
```
Inside the container, launch the SenseGlove system:
```
ros2 launch senseglove_bringup senseglove.launch.py run_sensecom:=false
```

> For specific usage instructions, refer to the [Usage Guide](../USAGE.md)
