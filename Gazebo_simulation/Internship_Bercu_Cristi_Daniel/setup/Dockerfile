ARG CODE_VERSION=noetic
FROM osrf/ros:${CODE_VERSION}-desktop-full

LABEL maintainer="GO1"
LABEL version="1.0.0"
LABEL description="Docker setup for Go1 robot simulation in ROS and Gazebo"
LABEL contact="cong-khanh.dinh@lcis.grenoble-inp.fr"

ENV DEBIAN_FRONTEND=noninteractive

# make sure the shell used is /bin/bash
SHELL ["/bin/bash","-c"]


RUN apt-get update

# Install essential packages for development
RUN apt-get install -y \
# net-tools package (includes ifconfig,... commands)
net-tools \
# iputils-ping package (includes ping,... commands)
iputils-ping \
git \
nano \
curl \
wget \
terminator \
software-properties-common \
build-essential \
# python package
python3 python3-pip 


RUN apt install -y python-is-python3 
# RUN apt install -y apt install python3.8-dev -y
RUN apt install ros-noetic-move-base-msgs -y
RUN apt install -y cmake 
# RUN apt-get update && apt-get install -y ros-noetic-pinocchio python3-pinocchio

# RUN apt install -y libboost-program-options-dev libusb-1.0-0-dev ros-jazzy-tf-transformations ros-jazzy-motion-capture-tracking ros-jazzy-teleop-twist-keyboard ros-jazzy-slam-toolbox
RUN pip3 install numpy scipy matplotlib pandas opencv-python \
              pillow transforms3d rowan bresenham nicegui \
              Jinja2 PyYAML rospkg catkin-pkg empy netifaces \
              sympy pygame casadi


RUN  pip install --upgrade pip setuptools wheel
RUN  pip install pin

RUN apt install -y liblcm-dev
RUN apt-get install -y python3-tk

# RUN pip3 install cflib transforms3d rowan nicegui Jinja2  --break-system-packages
# RUN pip3 install bresenham --break-system-packages

# Setup the workspace for ROS2
ARG WORKSPACE="/root/ros_ws"
RUN mkdir -p $WORKSPACE/src
RUN mkdir -p $WORKSPACE/build
RUN mkdir -p $WORKSPACE/devel


WORKDIR /root/ros_ws


RUN echo "source ~/ros_ws/install/setup.bash"


ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "bash" ]
