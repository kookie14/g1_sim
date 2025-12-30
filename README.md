# G1 IsaacSIM Package

## Introduction

This package simplifies working with IsaacSIM.

## Install
Use docker to build the image and run the container.
1. Build the docker image:
   ```bash
   chmod +x ./build.sh
   bash ./build.sh
   ```
2. Run the docker container:
   ```bash
    chmod +x ./start_docker.sh
    bash ./start_docker.sh
   ```
3. After running the container, you will be inside the isaacsim venv.Need to source the venv:
   ```bash
   source_humble
   ```

### Run Robot G1 simulation

Run `python apps/g1.py`
### In other local terminal
To check ROS2 BRIDGE topics, open another terminal and run:
```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
```
You should see topics:
```
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/depth/color/points
/clock
/imu
/joint_commands
/joint_states
/parameter_events
/rosout
```
Can check image topic:
```bash
ros2 run rqt_image_view rqt_image_view
```