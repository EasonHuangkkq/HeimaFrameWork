#!/bin/bash

cd ..
if [ ! -e "build" ]; then
    mkdir build
fi
cd build
cmake ..
make -j4 install

