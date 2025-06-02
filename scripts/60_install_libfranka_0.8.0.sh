#!/bin/bash

LIBFRANKA_VERSION=0.8.0

sudo apt install libpoco-dev libeigen3-dev build-essentials git dpkg-dev

git clone -b ${LIBFRANKA_VERSION} --recurse-submodules https://github.com/frankaemika/libfranka.git

mkdir build
cd build

cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
make -j4
make -j4 package
dpkg -i dpkg -i *.deb