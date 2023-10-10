#!/bin/bash

cd build/
rm -rf particle_filter
cd ../
cd install/
rm -rf particle_filter
cd ../
colcon build --packages-select particle_filter
