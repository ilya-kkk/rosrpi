ARG from_image
FROM ubuntu:focal

# Timezone Configuration
ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly

# COPY entrypoint.sh /entrypoint.sh
# COPY bashrc /home/${USERNAME}/.bashrc


USER root

# install required packages
RUN apt-get update && apt-get install -y apt-utils \
                                         lsb-release \
                                         mesa-utils \
                                         net-tools \
                                         build-essential \
                                         wget \
                                         unzip \
                                         curl \
                                         git \
                                         mc \
                                         vim \
                                         python3-pip \
                                         libopencv-dev libx11-dev liblapack-dev libv4l-dev libzbar-dev libpthread-stubs0-dev libsdl-dev libsdl-image1.2-dev \
                                         libeigen3-dev \
                                         tree && \
    rm -rf /var/lib/apt/lists/*


# Install ROS and navigation stack dependencies
RUN apt-get update && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' | apt-key add - && \
    apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y \
                      ros-noetic-desktop-full \
                      python3-rosdep \
                      python3-rosinstall \
                      python3-rosinstall-generator \
                      python3-wstool \
                      python3-catkin-tools \
                      ros-noetic-gazebo-ros-pkgs \
                      ros-noetic-ros-control \
                      ros-noetic-ros-controllers \
                      ros-noetic-gazebo-ros-control \
                      ros-noetic-teleop-twist-keyboard \
                      ros-noetic-navigation \
                      ros-noetic-hector-slam \
                      ros-noetic-gmapping \
                      ros-noetic-dwa-local-planner \
                      ros-noetic-global-planner \
                      ros-noetic-robot-localization \
                      ros-noetic-base-local-planner \
                      ros-noetic-tf2-tools \
                      ros-noetic-rqt-controller-manager \
                      ros-noetic-ecl-geometry \
                      ros-noetic-tf2-sensor-msgs \
                      ros-noetic-move-base-msgs \
                      ros-noetic-image-proc && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Install extra libraries
RUN pip install pathlib statistics scipy numpy scikit-image pillow opencv-contrib-python

RUN echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc
RUN echo "source /workspace/devel/setup.bash"  >> ~/.bashrc

WORKDIR /workspace

# The following lines are incorrect and should be removed.
# The workspace is mounted as a volume during 'docker run', not copied during build.
# COPY ./rosrpi/workspace/ /workspace/
# RUN chmod +x /workspace/CONTAINER_START.bash

ENTRYPOINT ["bash", "/workspace/CONTAINER_START.bash", "-i"]
#CMD ["/bin/bash", "-i"]
