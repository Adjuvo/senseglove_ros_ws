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

1. Edit [gloves.yaml](/senseglove_ros/senseglove/senseglove_bringup/config/gloves.yaml) to specify which gloves (refer serial) are connected:

Example:
```
gloves:
  - type: nova2
    side: right
    serial: "01000"
    finger_distance: false
  - type: nova2
    side: left
    serial: "01001"
    finger_distance: false
  - type: dk1
    side: right
    serial: "01002"       
    finger_distance: true
```
2. Launch
```
ros2 launch senseglove_bringup senseglove.launch.py
```

#### Launch Parameters ####
- `run_rviz`
    - true:  launches rviz with default config
    - false: does not launch rviz

- `run_sensecom`
    - true:  launches sensecom with hardware_node launch
    - false: does not launch sensecom

- `run_finger_distance`
    - true:  launch additional nodes that compute and publish finger distance data
    - false: skips

Example:
```
ros2 launch senseglove_bringup senseglove.launch.py run_rviz:=true run_sensecom:=true run_finger_distance:=true
```

## Finger-Tip Distances: ##
The finger distance nodes compute and publish the distance between each fingertip and the thumb fingertip. Update the `finger_distance` bool in the 
[gloves.yaml](/senseglove/senseglove_bringup/config/gloves.yaml) and launch the nodes using:

```
ros2 launch senseglove_bringup finger_distance.launch.py
```

#### Finger-Tip-Distance Node Parameters ####
- `calib_mode`
    - nothing:  raw output
    - normalized: normalized output

```
ros2 param set /senseglove/glove0/rh/finger_tip_distance_node calib_mode normalized
```

### Calibration Manager: ###
This node is resposible for starting a `calibration` service. Simply provide the target node, it starts a GUI, saves the ros-params, saves it to the yaml.
```
ros2 run senseglove_interaction calibration_manager --target-ns /senseglove/glove0/rh --call-service
```
Default and calibrated parameters are found in [calibration.yaml](/senseglove_ros/senseglove/senseglove_bringup/config/calibration.yaml)

## Haptics: ##

### Force Feedback
The force-feedback system (brakes + wrist squeeze) is implemented via ros2_control. Each SenseGlove product has joints and controllers defined in the [config folder](/senseglove_ros/senseglove/senseglove_control/senseglove_hardware_interface/config/).

Example haptic implementation in the [haptics folder](/senseglove_ros/senseglove/senseglove_interaction/senseglove_interaction/haptics/).

### Nova 2 Vibration: Custom Waveforms

Nova 2 LRA vibration is a **fire-and-forget** event. Each command is published **once** per haptic event. Have a look at the [nova2 vibration player](/senseglove_ros/senseglove/senseglove_interaction/senseglove_interaction/haptics/nova2_vibration_player.py).

**Topic:** `/senseglove/glove{SERIAL}/{rh|lh}/vibration_waveform` of message [Nova2WaveformCommand](/senseglove_ros/senseglove/senseglove_msgs/msg/Nova2WaveformCommand.msg)

> **Architecture note:** The RT-safe handoff uses `realtime_tools::RealtimeBuffer` + `std::atomic<bool>` inside `SenseGloveRobot`. The subscriber callback (non-RT) writes to the buffer and sets the flag; `write()` in the control loop checks the flag, fires the waveform once, and clears it.

> **Optimal frequency for Nova 2 LRA motors is ~180 Hz.**

> For detailed information on waveform parameters, motor locations, frequency guidance, and hardware behaviour, see the [SenseGlove Nova 2 Vibration documentation](https://senseglove.gitlab.io/SenseGloveDocs/nova2-vibration.html).

## Real-Time Priority (Optional)

For best performance with the ros2_control loop, grant the process real-time scheduling priority.

Add to `/etc/security/limits.conf`:
```
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft priority 99
@realtime hard priority 99
@realtime soft memlock 102400
@realtime hard memlock 102400
```

Then create the group and add your user:
```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
```

Log out and back in for the group change to take effect.