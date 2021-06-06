#!/bin/bash

rosdep install -i --from-path . -y
colcon build --symlink-install
