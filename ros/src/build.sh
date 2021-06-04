#!/bin/bash

rosdep install -i --from-path . -y
colcon build
. install/setup.bash
echo "install complete!"
