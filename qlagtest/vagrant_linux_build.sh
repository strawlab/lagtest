#!/bin/bash -x
set -e

sudo apt-get install -y cmake libeigen3-dev libqt5opengl5-dev libqwt-dev

THISDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# # ---- Use default compiler (g++) -----

#cd $THISDIR
#mkdir -p build
#cd build
#cmake ..
#make

# ---- Use clang since g++ has internal error ------

sudo apt-get install -y clang

sudo update-alternatives --set c++ /usr/bin/clang++

#cd $THISDIR
cd /vagrant
mkdir -p build-clang
cd build-clang
cmake ..
make
