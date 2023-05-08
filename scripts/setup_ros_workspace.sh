mkdir -p /home/nick/development/ros_ws/src
ln -s /home/nick/development/CSCI-5551/src/project_ws/src/baxter_moveit_controller /home/nick/development/ros_ws/src/baxter_moveit_controller

sudo apt-get update

sudo apt-get update
sudo apt-get install -y git python3-pip mlocate python3-wstool

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt -y install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full

sudo apt-get install -y ros-noetic-effort-controllers
sudo apt-get install -y ros-noetic-moveit
sudo apt-get install -y ros-noetic-aruco-detect
sudo apt-get install -y ros-noetic-rosbash
sudo apt-get install -y ros-noetic-ros-comm
sudo apt-get install -y ros-noetic-realsense2-camera
sudo apt-get install -y ros-noetic-realsense2-description

cd /home/nick/development/ros_ws/src
source /
wstool init .
wstool merge https://gist.githubusercontent.com/padpy/ac0961f2f0a7134d99a1ac0d389e1e6c/raw/081b8036d5efb78eba8fa766df928e920ab2f9dc/baxter_package.rosinstall
wstool update
cd /home/nick/development/ros_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_BLACKLIST_PACKAGES="dope"
source devel/setup.bash
# cd /home/nick/development/ros_ws/src/dope
# python3 -m pip install -r requirements.txt
# cd /ros_ws
# rosdep install --from-paths /ros_ws/src/dope -i --rosdistro noetic -y
catkin_make
# echo 'source /home/nick/development/ros_ws/devel/setup.bash' >> ~/.bashrc
# source ~/.bashrc
