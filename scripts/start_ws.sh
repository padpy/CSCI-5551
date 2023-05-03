#! /usr/bin/env bash

echo "Building Docker image"
git_commit_hash=$(git rev-parse --short HEAD)
docker build ./docker -t csci-5551/baxter-noetic:$git_commit_hash

echo "Starting Gazbeo"
rocker --nvidia --x11 --net=host --privileged \
    --volume $PWD/src/project_ws/src/baxter_moveit_controller:/ros_ws/src/baxter_moveit_controller \
    --volume $PWD/src/project_ws/src/gazebo_fiducial_generator:/ros_ws/src/gazebo_fiducial_generator \
    --volume $PWD/weights://ros_ws/src/dope/weights \
    -- csci-5551/baxter-noetic:$git_commit_hash bash
