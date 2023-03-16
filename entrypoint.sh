#!/bin/bash
set -e
# set -e Exit immediately if a command exits with a non-zero status.
# export DISTRIB="$(lsb_release -cs)" && \
#     debuild --preserve-env --dpkg-buildpackage-hook='sh debian/prepare' -i -us -uc -b && \ 
#     mv /tmp/*.deb /dist

# setup ros2 environment
export ROS2_WS=/home/ubuntu/ros2_ws
. $ROS2_WS/install/local_setup.bash
#cd /opt/rti.com/rti_connext_dds-5.3.1/resource/scripts && source ./rtisetenv_x64Linux3gcc5.4.0.bash; cd -
exec "$@"
