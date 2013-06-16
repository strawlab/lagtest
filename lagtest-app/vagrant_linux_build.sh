#!/bin/bash -x
set -e

sudo apt-get install -y cmake libeigen3-dev libqt4-opengl-dev libqwt-dev build-essential

THISDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# # ---- Use default compiler (g++) -----

#cd $THISDIR
cd /vagrant
mkdir -p build
cd build
cmake ..
make
