#!/bin/bash
 
IMAGE_BASE_NAME="g1_isaacsim_ros"
BUILT_IMAGE_TAG=${IMAGE_BASE_NAME}:latest
DOCKER_FILE=${IMAGE_BASE_NAME}.dockerfile
 
docker build --build-arg dUID=$(id -u) --build-arg dGID=$(id -g) -f ${DOCKER_FILE} -t ${BUILT_IMAGE_TAG} .