#!/bin/bash -eux

URL=https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules

# install udev rules
curl -SL $URL | sudo  tee  /etc/udev/rules.d/99-realsense-libusb.rules