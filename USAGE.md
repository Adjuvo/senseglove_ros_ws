# Usage Guide
This guide provides instructions for setting up and using SenseGlove devices within the ROS 2 environment.

## Connecting SenseGloves ##
SenseGlove devices can be connected either through USB or Bluetooth, depending on the device.
- **DK1:** USB
- **Nova 1/2:** Bluetooth

#### Connecting NOVA 1/2 Gloves via Bluetooth ####
1. Pair and connect the gloves on the host.
    - **BLE Gloves:** Pair in SenseCom and wait for connection
    - **Bluetooth Classic Gloves:** Use the provided [glove_connect.sh script](/senseglove_ros/senseglove/senseglove_bringup/scripts/glove_connect.sh) to bind each glove to a /dev/rfcomm* port. These devices will then be mounted inside the container.

2. You can choose to start SenseCom manually, or you can go directly to the [Launching SenseGloves](#launching-sensegloves) section below:
```
ros2 run senseglove_com SenseCom.x86_64  
```
> ℹ️ **Firmware Note (May 2025):**
>
> A new firmware version v2.X for Nova 2 has been released.
> - v1.X: Bluetooth Classic (Serial Port Profile)
> - v2.X: Bluetooth Low Energy (BLE)
> 
> BLE pairing is much simpler on desktop and we recommend upgrading your Nova 2 firmware to BLE. 
> See the [Nova 2 BLE Guide](https://senseglove.gitlab.io/SenseGloveDocs/nova2-ble.html) and [Connecting Devices](https://senseglove.gitlab.io/SenseGloveDocs/connecting-devices.html) for detailed instructions.

## Launching SenseGloves ###
The whole infrastructure of this codebase was built upon the use with infinitely many sensegloves. Hence we have a per-glove ros2-control launch system.

1. Edit [gloves.yaml](/senseglove_ros/senseglove/senseglove_bringup/config/gloves.yaml) to specify which gloves are connected:

Example:
```
gloves:
  - type: nova2
    side: right
    index: 0
  - type: nova2
    side: left
    index: 1
  - type: dk1
    side: right
    index: 3       
```
2. Launch
```
ros2 launch senseglove_bringup senseglove.launch.py
```

#### Launch Parameters ####
- `run_rviz`
    - true:  run in simulation mode (no real hardware required)
    - false: connect to actual gloves

- `run_sensecom`
    - true:  run in simulation mode (no real hardware required)
    - false: connect to actual gloves

<!-- - `use_finger_distance`
    - true:  launch additional nodes that compute and publish finger distance data
    - false: skip these nodes -->

Example:
```
ros2 launch senseglove_bringup senseglove.launch.py run_rviz:=true run_sensecom:=true
```

## (Update soon) Finger-Tip Distances: ##
The finger distance node package publish the distances between fingertips.
```
ros2 launch rembrandt_bringup finger_distance.launch.py
```

### Calibration Manager: ###
This node is resposible for starting a `calibration` service. Simply provide the target node, it starts a GUI, saves the ros-params, saves it to the yaml.
```
ros2 run senseglove_interaction calibration_manager /
    --target-ns /senseglove/glove0/rh /
    --call-service
```
Default and calibrated parameters are found in [calibration.yaml](/senseglove_ros/senseglove/senseglove_bringup/config/calibration.yaml)

## Haptics: ##
- The force-feedback system is implemented via ros2_control. Each SenseGlove product has joints and controllers defined in the [config folder](/senseglove_ros/senseglove/senseglove_control/senseglove_hardware_interface/config/).
- Example haptic implementation in the [haptics folder](/senseglove_ros/senseglove/senseglove_interaction/senseglove_interaction/haptics/).

> ⚠️ **NOVA 2**: The vibration feedback is disabled for the Index, Thumb and the Palm locations because of a packet overloading issue. A custom_waveform will be implemented instead of employing ros2-control in future.