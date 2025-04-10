#!/usr/bin/env bash

./build.sh || exit 1

BASE_DIR=sample_data/large
for name in $(ls $BASE_DIR/); do
    echo "$name... "
    time ./run.sh $BASE_DIR/$name out/$name.ppm &&  echo -e "\e[32mok\e[0m" || echo -e "\e[31mfail\e[0m"
   
done