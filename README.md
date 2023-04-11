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
Currently, the Baxter Gazebo simulation requires three terminals to run the simulation and another to enable the robot.

### Terminal 1: Run simulation
```bash
./scripts/start_ws.sh

# Inside the docker container
catkin_make
roslaunch baxter_moveit_controller baxter_sim.launch
```

### Terminal 2: Enable robot & Start MoveIt server
```bash
./scripts/attatch_shell.sh

# Inside the docker container

# Start controller sequence
rosrun baxter_moveit_controller node.py
```


## External Docs
This repo is still a bit Jank. I am basically following these instructions after starting the docker container.

https://subscription.packtpub.com/book/iot-&-hardware/9781782175193/6/ch06lvl1sec46/launching-baxter-simulator-in-gazebo