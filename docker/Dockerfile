# syntax=docker/dockerfile:1
FROM ros:jazzy-ros-base-noble

RUN apt update
RUN apt install -y python3-pip \
                   screen \
                   ssh

RUN pip3 install optuna --break-system-packages

RUN mkdir /koi
RUN mkdir /koi/src

WORKDIR /koi/src
RUN git clone https://github.com/DLR-MO/koi.git

WORKDIR /koi
RUN vcs import src < src/koi/dependencies.repos
RUN rosdep install --from-path src/reach --ignore-src -r  -y

# some additional package to run demos
RUN apt install -y libogre1.12.10 \
                   ros-jazzy-interactive-markers \
                   ros-jazzy-joint-state-publisher \
                   ros-jazzy-moveit-ros-move-group \
                   ros-jazzy-moveit-ros-planning-interface \
                   ros-jazzy-rviz2 \
                   ros-jazzy-xacro
                   
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
RUN echo 'source /koi/install/setup.bash' >> /root/.bashrc 
RUN echo 'export PYTHONPATH=$PYTHONPATH:/koi/install/reach/lib/python3/dist-packages' >> /root/.bashrc 
RUN echo 'export REACH_PLUGINS=reach_ros_plugins' >> /root/.bashrc 

WORKDIR /koi/src/koi
