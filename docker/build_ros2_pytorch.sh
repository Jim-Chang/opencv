#!/bin/sh

# ------------------------------------
# https://github.com/NVIDIA-AI-IOT/ros2_trt_pose/blob/main/docker/dockerfile.ros.eloquent.trt_pose
# ------------------------------------

# prepare entrypoint.sh for ros2
mkdir packages

tee -a packages/ros_entrypoint.sh <<EOF
#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/\$ROS_DISTRO/setup.bash"
exec "\$@"
EOF

# prepare key to download lib from nvidia
cp /etc/apt/trusted.gpg.d/jetson-ota-public.asc .

# build docker image
docker build -f Dockerfile.ros2.opencv.pytorch -t ros2_opencv_pytorch:latest .

# clean
rm jetson-ota-public.asc
rm -Rf packages