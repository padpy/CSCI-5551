FROM osrf/ros:noetic-desktop-full

RUN apt-get update


RUN mkdir -p /root/ros_ws/src

RUN apt-get update && apt-get install -y git
RUN apt-get install -y ros-noetic-effort-controllers

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /ros_ws/src\
    && cd /ros_ws/src/\
    && wstool init .\
    && wstool merge https://gist.githubusercontent.com/padpy/ac0961f2f0a7134d99a1ac0d389e1e6c/raw/4a9ed73d710a36dd9b1159c38956efa3ae39f754/baxter_package.rosinstall\
    && wstool update\
    && source /opt/ros/noetic/setup.bash\
    && cd ..\
    && catkin_make\
    && echo 'source ~/baxterws/devel/setup.bash' >> ~/.bashrc\
    && source ~/.bashrc

WORKDIR /ros_ws
