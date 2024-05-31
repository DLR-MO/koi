# syntax=docker/dockerfile:1
FROM ros:rolling-ros-base

RUN apt update
RUN apt install -y python3-pip \
                   ssh

RUN pip3 install optuna --break-system-packages

RUN mkdir /koi
RUN mkdir /koi/src

WORKDIR /koi/src
RUN git clone https://gitlab.dlr.de/mo-repo/koi.git

WORKDIR /koi
RUN vcs import src < src/koi/dependencies.repos
RUN rosdep install --from-path src/reach --ignore-src -r  -y
RUN source /opt/ros/rolling/setup.bash && colcon build

SHELL ["/bin/bash", "-c"]
