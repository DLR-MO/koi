# syntax=docker/dockerfile:1
FROM ros:rolling-ros-base-jammy

RUN apt update
RUN apt install -y python3-pip \
                   ssh

RUN pip3 install optuna

RUN mkdir /koi
RUN mkdir /koi/src

WORKDIR /koi/src
RUN git clone https://github.com/DLR-MO/koi.git

WORKDIR /koi
RUN vcs import src < src/koi/dependencies.repos
RUN rosdep install --from-path src/reach --ignore-src -r  -y

# some additional package to run demos
RUN apt install -y libogre1.12.10 \
                   ros-rolling-interactive-markers \
                   ros-rolling-joint-state-publisher \
                   ros-rolling-moveit-ros-move-group \
                   ros-rolling-moveit-ros-planning-interface \
                   ros-rolling-rviz2 \
                   ros-rolling-xacro
                   
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/rolling/setup.bash && colcon build --symlink-install
RUN echo 'source /koi/install/setup.bash' >> /root/.bashrc 
RUN echo 'export PYTHONPATH=$PYTHONPATH:/koi/install/reach/lib/python3/dist-packages' >> /root/.bashrc 
RUN echo 'export REACH_PLUGINS=reach_ros_plugins' >> /root/.bashrc 
RUN echo 'export DISPLAY=:0' >> /root/.bashrc 
