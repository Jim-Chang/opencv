#!/bin/sh

echo "scan I2C channel 0"
sudo i2cdetect -r -y 0
echo "scan I2C channel 1"
sudo i2cdetect -r -y 1