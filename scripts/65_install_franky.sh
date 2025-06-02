#!/bin/bash -e

FRANKY_VERSION=v0.10.0

sudo apt install libeigen3-dev python3-pybind11

git clone -b ${FRANKY_VERSION} --recurse-submodules git@github.com:timschneider42/franky.git

cd franky
python setup.py bdist_wheel
pip install ./dist/franky_panda-*-linux_x86_64.whl