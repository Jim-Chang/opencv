#!/bin/bash

docker run -it --rm -v /home/jim/opencv/face_recognition:/app -v /home/jim/motioneye/record/MainDoor:/app/videos home-watcher python file_watcher.py $1