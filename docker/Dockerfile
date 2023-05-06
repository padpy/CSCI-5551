FROM osrf/ros:noetic-desktop-full

RUN apt-get update


RUN mkdir -p /root/ros_ws/src

RUN apt-get update && apt-get install -y git python3-pip mlocate
RUN apt-get install -y ros-noetic-effort-controllers
RUN apt-get install -y ros-noetic-moveit
RUN apt-get install -y ros-noetic-aruco-detect
RUN apt-get install -y ros-noetic-rosbash
RUN apt-get install -y ros-noetic-ros-comm
RUN apt-get install -y ros-$ROS_DISTRO-realsense2-camera
RUN apt-get install -y ros-$ROS_DISTRO-realsense2-description


SHELL ["/bin/bash", "-c"]
RUN mkdir -p /ros_ws/src\
    && cd /ros_ws/src/\
    && source /opt/ros/noetic/setup.bash\
    && wstool init .\
    && wstool merge https://gist.githubusercontent.com/padpy/ac0961f2f0a7134d99a1ac0d389e1e6c/raw/081b8036d5efb78eba8fa766df928e920ab2f9dc/baxter_package.rosinstall\
    && wstool update\
    && cd /ros_ws\
    && catkin_make -DCATKIN_BLACKLIST_PACKAGES="dope"\
    && source devel/setup.bash\
    && cd /ros_ws/src/dope\
    && python3 -m pip install -r requirements.txt\
    && cd /ros_ws\
    && rosdep install --from-paths /ros_ws/src/dope -i --rosdistro noetic -y\
    && catkin_make\
    && echo 'source /ros_ws/devel/setup.bash' >> ~/.bashrc\
    && source ~/.bashrc

# This is a super bad hack to get around the fact there is an issue with the generated ActionFeedback message.
# This seems to be an issue of a type mismatch after upgrading to python3
COPY resources/_FollowJointTrajectoryActionFeedback.py /opt/ros/noetic/lib/python3/dist-packages/control_msgs/msg/_FollowJointTrajectoryActionFeedback.py
WORKDIR /ros_ws
