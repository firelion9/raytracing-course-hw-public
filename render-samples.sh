#!/usr/bin/env bash

./build.sh || exit 1

BASE_DIR=sample_data/gltf
for name in $(ls $BASE_DIR/*.gltf); do
    echo "$name... "
    time ./run.sh "$name" 1000 1000 100 "out/$name.ppm" &&  echo -e "\e[32mok\e[0m" || echo -e "\e[31mfail\e[0m"
   
done