sudo docker run -it --privileged --net=host --gpus all \
    --device="/dev/ttyACM0" \
    --device="/dev/ttyACM1" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    dockerfile_nvidia_ros_melodic
 
