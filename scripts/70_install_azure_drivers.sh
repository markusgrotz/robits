#!/bin/bash

curl -SL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -


# we have to use the bionic repository, i.e. packages for Ubuntu 18.04.
#curl -SL https://packages.microsoft.com/config/ubuntu/18.04/prod.list | sudo tee /etc/apt/sources.list.d/microsoft.list

sudo tee /etc/apt/sources.list.d/microsoft.list <<EOF
deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main
# deb-src https://packages.microsoft.com/ubuntu/18.04/prod bionic main
EOF

sudo apt update
sudo apt install k4a-tools=1.3.0 libk4a1.3 libk4a1.3-dev 

# install udev rules
curl -SL https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules | sudo  tee  /etc/udev/rules.d/99-k4a.rules 