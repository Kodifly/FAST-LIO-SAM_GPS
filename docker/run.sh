#!/bin/bash

xhost +local:docker  # More secure than 'xhost +'

docker run -it --rm --runtime=nvidia --net=host --privileged \
  -e DISPLAY=$DISPLAY \
  -v /home/kodifly/workspaces/fastliosam_ws:/home/admin/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/home/admin/.Xauthority:rw \
  -v /etc/timezone:/etc/timezone:ro \
  -v /etc/localtime:/etc/localtime:ro\
  gc625kodifly/fast_livo2:latest  \
  /bin/bash
