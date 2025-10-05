#!/bin/bash

#g++ laser_trace.cpp -fPIC -O3  -msse4 -shared -I/usr/include/python2.7 -lboost_python-py27 -lpython2.7 -o laser_trace.so
g++ -shared -fPIC -o laser_trace.so laser_trace.cpp \
    -I/usr/include/python3.8 \
    -I/usr/lib/python3/dist-packages/numpy/core/include \
    -lboost_python38 \
    -lpython3.8

