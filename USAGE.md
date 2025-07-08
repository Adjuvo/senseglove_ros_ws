# Usage Guide
This guide provides instructions for setting up and using SenseGlove devices within the ROS  environment.


## Connecting SenseGloves ##
SenseGlove devices can be connected either through USB or Bluetooth, depending on the device. The `DK1` device uses USB, while the `Nova 1/2` devices rely on Bluetooth communication.

#### Connecting NOVA 1/2 Gloves via Bluetooth ####

The workspace includes helper scripts to manage the Bluetooth connection to Nova gloves. To initiate a connection with your Nova glove, run:

```
rosrun senseglove_launch glove_connect.sh
```
To disconnect all currently connected gloves, run:
```
rosrun senseglove_launch glove_disconnect.sh
```

`Note:`For a complete, step-by-step guide on how to pair and connect Nova 1/2 gloves on linux, kindly refer to [SenseGlove Docs - Connecting Devices](https://senseglove.gitlab.io/SenseGloveDocs/connecting-devices.html), under Pairing SenseGlove Nova or Wireless Kit -> Linux.

## Launching SenseGloves (Single/Dual) ###
Though the whole infrastructure of this codebase was built upon the use with infinitely many sensegloves, our example launch file only accepts two gloves. The current implementation allows you to use either of these standalone gloves. Future updates can include simultaneous use of gloves.

Moreover, due to our integration into ros-control we require the user to know what type of gloves are connected to the PC. As such, the user has to define which glove is connected to the system.

1. Source your workspace
2. Make sure your sensegloves are connected through usb or bluetooth
    - If you are checking your connection by running a sensecom instance, be sure to exit the application before proceeding to avoid runing multiple instances of sensecom during launch.

3. In the senseglove.launch script, make sure you specify the devices being used:
    - use_dk
    - use_nova
    - use_nova2

4. Finally, make sure you specify the handedness arguments:
    - use_left = true
    - use_right = true

5. Run: 
```
roslaunch senseglove_launch senseglove.launch
```

`Note 1:`A bash script is called invoking sensecom and running the hardware interface node twice for a left- and a right-handed glove.

`Note 2:`The bash **waits for the user input** to confirm whether the gloves are connected in SenseCom. 

`Note 3:`Calibration of the Nova-2 device runs on the glove itself. The SenseCom software offers a visual guide to accompany calibration. For more information, go to [SenseGlove Docs/Nova-2](https://senseglove.gitlab.io/SenseGloveDocs/nova-2.html)


## Finger-Tip Distances: ##
This workspace includes a package that publishes the distance between fingertips, which can be used for applications such as controlling robotic hands or evaluating hand gestures.

To enable accurate measurement, a calibration service is provided. This service can be called either through the terminal or by using the rqt_service_caller plugin.

To calibrate a left-handed glove, run:
```
rosservice call /senseglove_finger_distance_left/Calibrate left
```

To calibrate a right-handed glove, use: _msgs
```
rosservice call /senseglove_finger_distance_right/Calibrate right
```

Upon launching either service, a GUI will appear to guide you through the calibration process. Once completed, the resulting parameters will be stored in the [calibration folder](senseglove/senseglove_msgs/calibration/)

## Haptics: ##
The force-feedback system is implemented using ros_control, and each SenseGlove product has its joints and controllers defined in a separate configuration file. These configuration files can be found in the [config folder](/senseglove/senseglove_control/senseglove_hardware_interface/config/).

Refer to the python scripts for haptic implementation in the [senseglove_haptics folder](/senseglove/senseglove_interaction/src/senseglove_interaction/haptics/).

`⚠️ NOVA 2`: The vibration feedback is disabled for the Index,  Thumb and the palm locations because of a packet overloading issue. A custom_waveform service will be implemented instead of employing ros-control in future.