#! /usr/bin/env bash

echo "Building Docker image"
git_commit_hash=$(git rev-parse --short HEAD)
docker build ./docker -t csci-5551/baxter-noetic:$git_commit_hash

echo "Starting Gazbeo"
rocker --nvidia --x11 csci-5551/baxter-noetic:$git_commit_hash bash
