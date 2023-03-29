#! /usr/bin/env bash

echo "Building Docker image"
git_commit_hash=$(git rev-parse --short HEAD)
docker build . -t csci-5551/baxter:$git_commit_hash

echo "Starting Gazbeo"
rocker --nvidia --x11 csci-5551/baxter:$git_commit_hash '/root/ros_ws/baxter.sh sim'
