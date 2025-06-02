#!/bin/bash

LIBFRANKA_VERSION=0.8.0
FRANKY_VERSION=v0.10.0


if [ -z "$CONDA_PREFIX" ]; then
  echo "Please activate your conda environment first."
  exit 1
fi

conda install mamba -c conda forge
mamba install poco=1.9 gcc make cmake= pybind11 -c conda-forge

#export CC=${CONDA_PREFIX}/bin/gcc
#export CXX=${CONDA_PREFIX}/bin/g++
#export LD_LIBRARY_PATH=${CONDA_PREFIX}/lib:$LD_LIBRARY_PATH

git clone -b ${LIBFRANKA_VERSION} --recurse-submodules https://github.com/frankaemika/libfranka.git

mkdir build
cd build

cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX} ..
make -j4
make install

if [ $? -ne 0 ]; then
  echo "Installing libfranka failed. Exiting."
  exit 1
fi

cd ..

git clone -b ${FRANKY_VERSION} --recurse-submodules git@github.com:timschneider42/franky.git

cd franky
python setup.py bdist_wheel
pip install ./dist/franky_panda-*-linux_x86_64.whl