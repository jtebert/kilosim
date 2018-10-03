#!/usr/bin/env bash

rm -f bin/simulation
mkdir -p bin
cd src

g++ robot.cpp main.cpp shapes.cpp vars.cpp -O3 -lGL -lGLU -lglut -o ../bin/simulation -std=c++11 -fopenmp
cd ..
