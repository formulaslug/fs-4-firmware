#! /bin/sh

rm -rf ./build
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Develop
sudo ninja -C build flash-Charger
