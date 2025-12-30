#!/bin/bash
 
# Configuration
CONTAINER_NAME="g1_isaacsim_ros"
IMAGE_NAME="$CONTAINER_NAME:latest"
SOURCE_MNT_DIR="/Documents/MALAY_G1/g1_sim"  # Optional: adjust as needed
DEST_MNT_DIR="/home/ros2_nh/ros2_ws/src"
 
# Check if a container exists
container_exists() {
    docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"
}
 
# Get container status
get_container_status() {
    docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null
}
 
# Start a container (given name) if it's existing, otherwise create a new one
if container_exists; then
    STATUS=$(get_container_status)
    if [ "$STATUS" = "running" ]; then
        echo "+++ Container '$CONTAINER_NAME' is already running +++"
    else
        echo "+++ Starting existing container '$CONTAINER_NAME' +++"
        docker start "$CONTAINER_NAME"
    fi
    docker exec -it "$CONTAINER_NAME" bash
else
    echo "+++ Container '$CONTAINER_NAME' does not exist. Creating... +++"
    xhost +  > /dev/null 2>&1
    docker run --runtime=nvidia --gpus all \
        --env NVIDIA_DISABLE_REQUIRE=1 \
        --env QT_X11_NO_MITSHM=1 \
        --env NVIDIA_VISIBLE_DEVICES=all \
        --env NVIDIA_DRIVER_CAPABILITIES=all \
        --env DISPLAY="${DISPLAY}" \
        -it --network=host \
        --name "$CONTAINER_NAME" \
        --cap-add=SYS_PTRACE \
        --security-opt seccomp=unconfined \
        --privileged \
        -v "$SOURCE_MNT_DIR":"$DEST_MNT_DIR" \
        -v /etc/udev/rules.d/:/etc/udev/rules.d/ \
        --ipc=host \
        "$IMAGE_NAME" bash
fi