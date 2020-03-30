#!/bin/bash
sudo apt-get install build-essential qtcreator qt5-default libeigen3-dev clang cmake
if [ ! -d "build" ]; then
  mkdir build
fi
if [ ! -d "bin" ]; then
    mkdir bin
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target all -- -j 8
