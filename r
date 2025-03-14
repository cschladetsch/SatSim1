#!/bin/bash

mkdir -p build && cd build && cmake .. && make
./SatelliteApp
./SatelliteTests
./Benchmark


