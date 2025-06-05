## Running in Docker ##
Due to current development priorities, the migration toward full ROS 2 support is still in progress. However, because of frequent requests for ROS 2 compatibility, we provide a temporary workaround using a Dockerized environment that includes both ROS 1 and ROS 2. In this setup, topics and services from the ROS 1 container are forwarded to a ROS 2 system using a ros1_bridge.

`Important:` You can choose to run the ROS 1 container independently or connect it to a separate ROS 2 environment (e.g. ROS 2 Jazzy) using Docker Compose.

---
### Preparing the Environment ###
Before launching the containers, enable GUI forwarding so that any tools requiring X11 (e.g. RViz or PyQt GUIs) can be displayed on your host system:
```
xhost +local:docker
```

Make entrypoint scripts executable
```
chmod +x Dockerfiles/ros1_entrypoint.sh
chmod +x Dockerfiles/ros2_entrypoint.sh
```

---
### Connecting NOVA 1/2 Gloves over Bluetooth (in Docker) ###

Bluetooth-based Nova 1/2 gloves must be paired on the host machine (Ubuntu 22.04 or 24.04) using the `glove_connect.sh` script, found in the [bluetooth scripts folder](../senseglove/senseglove_launch/bluetooth_scripts/). This script binds the device to a /dev/rfcomm* port, which is then passed through to the container.
```
./glove_connect.sh
```
Once the device is bound on the host, it becomes accessible inside the container, provided that the /dev/rfcomm* devices are correctly mounted. After that, you can proceed with launching the ROS 1 SenseGlove system from within the container.

---
### Launching the ROS 1 Container ###
To start only the ROS 1 container (which will also build the ROS 1 workspace on first launch), run:

```
docker compose up senseglove_ros1
```

Once the container is running, you can open an interactive shell and launch the SenseGlove system as follows:
```
docker compose exec senseglove_ros1 bash
roslaunch senseglove_launch senseglove.launch
```
`Note:`For detailed usage instructions, refer to the [Usage Guide](../USAGE.md)

---
### Launching the ROS 2 Container ###

The ROS 2 bridge container requires the ROS 1 system to be active, with roscore running. Only then will the bridge be able to forward topics and services correctly. To launch the ROS 2 bridge container, run:
```
docker compose up ros2_bridge
```

This enables access to the ROS 1  topics and services either from your host machine (e.g. Ubuntu 24.04 running ROS 2 Jazzy) or from another Docker container on the same network. To interact with the ROS 2 container manually, you can open a shell using:

```
docker compose exec ros2_bridge bash
source /opt/ros/jazzy/setup.bash
```

`Important:` Always make sure you source the correct ROS 2 environment inside the container before running any ROS 2 commands.


### Launching Both ROS 1 and ROS 2 Containers Together ###

To launch both the ROS 1 container and the ROS 2 bridge in a single command, use the bridge profile provided in the Docker Compose configuration:
```
docker compose --profile bridge up -d
```

This will start both containers in detached mode. Once running, you can open a shell in either container to run ROS tools as needed.