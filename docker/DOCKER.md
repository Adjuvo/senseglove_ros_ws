# 🐳Running in Docker #
Due to current development priorities, the migration to full ROS 2 support is still ongoing. However, in response to frequent requests for ROS 2 compatibility, we offer a temporary workaround using a Dockerized environment that includes both ROS 1 and ROS 2.

In this setup, you can run SenseGlove standalone inside a ROS 1 Docker container, and forward topics from the ROS 1 container to a ROS 2 system (Docker or Host PC or both) using a dynamic ros1_bridge. This setup has been tested and currently works with Ubuntu 24.04 + ROS 2 Jazzy.

## Contents ##
As mentioned in the attribution section of the [Readme file](/README.md), we recommend reviewing the source files if you’re interested in the underlying structure of the Dockerized ROS 1 ⇄ ROS 2 solution. The setup is organized around a single docker-compose.yml file that defines three key services:

| Service Name             | Profile              | Description                                                                                                                                                                                                                                       |
| ------------------------ | -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`senseglove_ros`**     | *(default)*          | A ROS Noetic container used to build workspaces, run senseglove drivers, and launch glove nodes. Runs in privileged mode with hardware access (`/dev/tty*`, `/dev/rfcomm*`) and GUI forwarding.                                             |
| **`rosbridge_1to2`**     | `rosbridge_1to2`     | A bridge container that includes both ROS Noetic and ROS 2 Jazzy. It sets up a dynamic `ros1_bridge` and installs required custom message types to forward topics between ROS 1 and ROS 2.                         |
| **`host_rosbridge_tgz`** | `host_rosbridge_tgz` | A utility container that extracts the `ros1_bridge` binary as a `.tgz` archive. Intended for users who prefer to run ROS 2 natively on the host machine, either independently or alongside the Docker container. |

This modular setup allows flexible integration between ROS 1 and ROS 2 environments using either fully containerized or hybrid approaches.

## Preparing the Environment ##
Before launching the containers, enable GUI forwarding on your host system so that any tools requiring X11 (e.g. RViz or PyQt GUIs) can be displayed
```
xhost +local:docker
```

Make the entrypoint scripts executable
```
chmod +x entrypoints/ros1_entrypoint.sh
chmod +x entrypoints/rosbridge_1to2_entrypoint.sh
```

## Building the Docker Containers ##

To build only the ROS 1 container (senseglove_ros), run:
```
docker compose build senseglove_ros
```

To build both the ROS 1 container and the ROS 1 ⇄ ROS 2 bridge (rosbridge_1to2), use the rosbridge_1to2 profile:
```
docker compose --profile rosbridge_1to2 build
```

`Note:`Building these containers for the first time may take several minutes and use significant processing resources, especially when compiling ros1_bridge with custom message types.

## Running the Docker Containers ##

### Step 1: Connecting Nova 1/2 Gloves over Bluetooth (from the Host) ###
Nova 1/2 gloves must be paired and connected on the host machine (Ubuntu 22.04 or 24.04) before starting the container. Use the provided glove_connect.sh script located in the [bluetooth scripts folder](../senseglove/senseglove_launch/bluetooth_scripts/) to bind each glove to a /dev/rfcomm* port:

```
./glove_connect.sh
```

Once successfully bound, the /dev/rfcomm* devices will be mounted into the container and made accessible to the ROS 1 senseglove system.


### Step 2: Launching the ROS 1 Container (senseglove_ros) ###
To start only the ROS 1 container (senseglove_ros), run:
```
docker compose up senseglove_ros
```

During the first run, the container will automatically build the SenseGlove ROS workspace and source it. Once the container is running, open an bash shell:
```
docker compose exec senseglove_ros bash
```
Inside the container:
1. Start roscore (recommended):
```
roscore
```
2. In another bash shell, launch the SenseGlove system:

```
roslaunch senseglove_launch senseglove.launch
```
`Note:`For specific usage instructions, refer to the [Usage Guide](../USAGE.md)


### Step 3: Launching the ROS 2 Container (rosbridge_1to2) ###

The ROS 2 bridge container depends on the ROS 1 system being active, with roscore running. This ensures that topics are properly discovered and forwarded between ROS 1 and ROS 2.

To launch the ROS 2 bridge container, run:
```
docker compose up rosbridge_1to2
```
To launch both the ROS 1 container and the ROS 2 bridge container together, use the profile:
```
docker compose --profile rosbridge_1to2 up
```
`Note:`You still need to manually start roscore inside the ROS 1 container for the bridge to function correctly.

To open a bash shell inside the bridge container:

```
docker compose exec ros2_bridge bash
```
Once running, the dynamic bridge node (ros1_bridge) will automatically detect compatible topics. You’ll see output in the terminal indicating which interfaces are being bridged.


### [OPTIONAL] Step 4: Launching the Host ROS 2 rosbridge Setup (host_rosbridge_tgz) ###
This step is intended for users who prefer to run ROS 2 natively on the host machine, either independently or alongside the Docker containers.

#### Prerequisites: ####
- Ubuntu 24.04 with ROS 2 Jazzy installed
- ros2_control and trajectory controller packages installed
- Prebuilt ros1_bridge binary (extracted from the host_rosbridge_tgz container)
- [SenseGlove custom message package](/docker/shared/senseglove_shared_resources_msgs.tar.xz), built and sourced in your ROS 2 workspace 

#### Setting up the ros1_bridge binary ####
If and only if the `rosbridge_1to2` container has already been built, you can extract the precompiled ros1_bridge binary by running:
```
docker compose --profile host_rosbridge_tgz run --rm host_rosbridge_tgz | tar xvzf -
```

This will extract a .tar.gz archive containing the bridge files. You can move this folder to your home directory or any other convenient workspace location. When sourcing the workspace, use `local_setup.bash` instead of `setup.bash`, because the bridge was built inside a Docker container and may rely on a different underlay environment. 
```
source ~/ros-jazzy-ros1-bridge/install/local_setup.bash
```

#### Setting up the SenseGlove custom message package ####
In [docker/shared/](/docker/shared/), you’ll find the compressed tarball [senseglove_shared_resources_msgs.tar.xz](/docker/shared/senseglove_shared_resources_msgs.tar.xz), which contains the custom message definitions required for bridging between ROS 1 and ROS 2.

Inside the archive, you’ll find two folders:
- `ros1/` – for ROS Noetic
- `ros2/` – for ROS 2 Jazzy

In the Docker build process, both ROS 1 and ROS 2 versions are already compiled into the rosbridge_1to2 container. This allows ros1_bridge to correctly map and forward messages between the two environments.

To use the message package on a native ROS 2 system, simply extract the tarball to your home directory (or any workspace location), build only the ROS 2 portion, and source it:

```
cd ~/senseglove_shared_resources_msgs/ros2
colcon build
source install/setup.bash
```

#### Running the ros1_bridge ####
For convenience, it's recommended to source both the ros1_bridge binary workspace and the SenseGlove custom messages workspace in your ~/.bashrc, so they're always available in new terminals.

To start the bridge node with all topics bridged in both directions, run:
```
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
``` 

| Option                     | Description                                                                  |
| -------------------------- | ---------------------------------------------------------------------------- |
| `-h`, `--help`             | Show the help message.                                                       |
| `--show-introspection`     | Print introspection output of both ROS 1 and ROS 2 sides.                    |
| `--print-pairs`            | Print a list of all supported ROS 1 ⇄ ROS 2 message conversion pairs.        |
| `--bridge-all-topics`      | Bridge all topics in both directions, even without matching subscribers. |
| `--bridge-all-1to2-topics` | Bridge all ROS 1 → ROS 2 topics, even without matching subscribers.      |
| `--bridge-all-2to1-topics` | Bridge all ROS 2 → ROS 1 topics, even without matching subscribers.      |

## Important Notes ##
`Note 1:` Avoid starting roscore in the background (e.g. roscore & at entrypoints) when using network_mode: host. Background ROS processes can leak into the host system and persist after the container exits. If this happens, kill them manually from the host.

`Note 2:` If you don’t see the trajectory command topic (used to send haptics to the glove) in ros2 topic list, it’s likely because no publisher is active. The topic will appear once a SenseGlove haptics node is running.

