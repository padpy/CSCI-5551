# Baxter-Sim

This is the project directory for our ros project, sorry it is a mess. The package that we created for controlling our robot is in the following directory `src/project_ws/src/baxter_moveit_controller`. The node script used in the Demo is `BaxterIKController.py` and the move server logic is in NextMove.py.

## Installing project into local environment
To install this repository you will require the proper folder structure. These installation script will attempt to automatically instal ROS Noetic, so it is advised to run the installation process on a fresh VM or Ubunut 20.04 installation.
``` Bash
# Clone the project directory
mkdir ~/development
cd ~/development
git clone https://github.com/padpy/CSCI-5551.git

# Setup ros workspace, will request for sudo during run
cd CSCI-5551
./scripts/setup_ros_workspace.sh

# Go to the newly created workspace
cd ../ros_ws
source devel/setup.bash
```

## Running the Demo code
Currently, due to an issue with the gazebo plugin for Baxter motion planning, the demo code must be run on the Baxter's physical hardware.
### Terminal 1: Start CV
```bash
source devel/setup.bash
# Set ROS_MASTER_URI adn ROS_IP

roslaunch baxter_moveit_controller baxter_real.launch
```

### Terminal 2: Enable robot controller
```bash
source devel/setup.bash
# Set ROS_MASTER_URI adn ROS_IP

rosrun baxter_moveit_controller BaxterIKController.py
```

### Terminal 3: Enable AI move server
```bash
source devel/setup.bash
# Set ROS_MASTER_URI adn ROS_IP

rosrun baxter_moveit_controller nextmove_server.py
```

### Terminal 4: Send board state to AI move server and make robot play a move.
```bash
source devel/setup.bash
# Set ROS_MASTER_URI adn ROS_IP

# the array represents the board state with a 0 Empty, 1 Robot mark, and 2 the Player mark.
# The index represents the board stpase
# 0 | 1 | 2
# ---------
# 3 | 4 | 5
# ---------
# 6 | 7 | 8

rostopic pub /tictactoe/board std_msgs/Int8MultiArray "
  dim:
  - label: ''
    size: 9
    stride: 0
  data_offset: 0
data: [0, 0, 2, 0, 1, 0, 0, 0, 0] 
"
```

## External Docs
This repo is still a bit Jank. I am basically following these instructions after starting the docker container.

https://subscription.packtpub.com/book/iot-&-hardware/9781782175193/6/ch06lvl1sec46/launching-baxter-simulator-in-gazebo
