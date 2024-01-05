docker run -it \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v="$XAUTH:$XAUTH" \
    -v $XAUTHORITY:/root/.Xauthority \
    -v /dev/dri:/dev/dri \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /var/run/docker.sock:/var/run/docker.sock \
    ros2:ubuntu-humble-desktop-full