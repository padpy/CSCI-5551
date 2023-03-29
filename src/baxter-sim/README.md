# Baxter-Sim

This repository is used for creating a simulated Baxter environment.

## Requirements

### Docker Engine
https://docs.docker.com/engine/install/

### rocker
``` bash
pip install rocker
```

## Running instructions
Currently, the Baxter Gazebo simulation requires two terminals to run the simulation and another to enable the robot.

### Terminal 1: Run simulation
```bash
./gazebo.sh

# Inside the docker container
roslaunch baxter_gazebo baxter_world.launch
```

### Terminal 2: Enable robot
```bash
docker exec -it $(docker ps | awk 'FNR==2{print $1}') /root/ros_ws/baxter.sh sim

# Inside the docker container
# Enable robot
rosrun baxter_tools enable_robot.py -e

# Check that robot is enabled 
rosrun baxter_tools enable_robot.py -s

# Test robot accepting commands
rosrun baxter_tools tuck_arms.py -t
```

## External Docs
This repo is still a bit Jank. I am basically following these instructions after starting the docker container.

https://subscription.packtpub.com/book/iot-&-hardware/9781782175193/6/ch06lvl1sec46/launching-baxter-simulator-in-gazebo