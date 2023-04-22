FROM osrf/ros:noetic-desktop-full

RUN apt-get update


RUN mkdir -p /root/ros_ws/src

RUN apt-get update && apt-get install -y git
RUN apt-get install -y ros-noetic-effort-controllers ros-noetic-moveit ros-noetic-aruco-detect

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /ros_ws/src\
    && cd /ros_ws/src/\
    && wstool init .\
    && wstool merge https://gist.githubusercontent.com/padpy/ac0961f2f0a7134d99a1ac0d389e1e6c/raw/309fc5a92ac5abd6326184da22a0b45c24e0bb7d/baxter_package.rosinstall\
    && wstool update\
    && source /opt/ros/noetic/setup.bash\
    && cd ..\
    && catkin_make\
    && echo 'source /ros_ws/devel/setup.bash' >> ~/.bashrc\
    && source ~/.bashrc

# This is a super bad hack to get around the fact there is an issue with the generated ActionFeedback message.
# This seems to be an issue of a type mismatch after upgrading to python3
COPY resources/_FollowJointTrajectoryActionFeedback.py /opt/ros/noetic/lib/python3/dist-packages/control_msgs/msg/_FollowJointTrajectoryActionFeedback.py
WORKDIR /ros_ws
