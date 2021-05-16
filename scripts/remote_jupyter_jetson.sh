#!/bin/bash

ssh -N -f -L localhost:9999:localhost:8888 jim@jetson
echo "Please connect http://localhost:9999"
