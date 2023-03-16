# This is an auto generated Dockerfile for ros2:devel
# generated from docker_images_ros2/devel/create_ros_image.Dockerfile.em
ARG FROM_IMAGE=nvidia/opengl:base-ubuntu20.04
FROM $FROM_IMAGE

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    bash-completion \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    vim \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-rospkg \
    python3-rosdistro \
    ros-foxy-desktop \
    ros-foxy-gazebo-* \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip3 install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    catkin_pkg
# This is a workaround for pytest not found causing builds to fail
# Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
RUN pip3 freeze | grep pytest \
    && python3 -m pytest --version

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update



# For testing
#RUN apt-get update && apt-get install -y x11-apps

# clone source
ENV ROS2_WS /home/ubuntu/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

# build source
RUN colcon \
    build \
    --cmake-args \
      -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install

# setup bashrc
RUN cp /etc/skel/.bashrc ~/

# setup entrypoint
COPY ./entrypoint.sh /

RUN apt update
RUN apt install nautilus wget aptitude libgazebo11-dev  -y
#RUN RTI_NC_LICENSE_ACCEPTED=yes apt install -q -y rti-connext-dds-5.3.1 -y
RUN apt install libdart-all-dev -y

# install Gazebo
# RUN apt-get install gazebo9 -y

# Create USER with USER ID matching host system to access ROS from host system

#RUN cd /opt/rti.com/rti_connext_dds-5.3.1/resource/scripts && source ./rtisetenv_x64Linux3gcc5.4.0.bash; cd -
#RUN adduser -u 1000 --disabled-password --gecos '' ubuntu_user
#RUN usermod -aG sudo ubuntu_user
#USER ubuntu_user

ENV DISPLAY :1

# Install Cpp libraries
#COPY lib/ /libs
#WORKDIR /libs
#RUN tar -xvzf ./*.tar.gz && cmake ./eigen-3.4.0 && make install

WORKDIR ${ROS2_WS}

# copy src folder
# ADD ./src ${ROS2_WS}/src

# for ROS2 environment setup
RUN echo ". /opt/ros/foxy/setup.bash" >> ~/.bashrc

# source it and build it..
RUN . ~/.bashrc

COPY ./build_ros_launch.sh /
RUN chmod +x /build_ros_launch.sh

RUN cd ${ROS2_WS} \
  && . /opt/ros/foxy/setup.sh \
  && colcon build


ENV GAZEBO_PLUGIN_PATH ${ROS2_WS}/build/simulator 



ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]