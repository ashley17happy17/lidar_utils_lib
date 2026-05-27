#!/bin/bash

echo "Build Docker"

sudo docker build --progress=plain -f ./Dockerfile -t lidar_utils_lib:noetic .

echo "Docker successfully build!"