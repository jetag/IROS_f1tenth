#!/bin/bash

cd build/
rm -rf pure_pursuit
cd ../
cd install/
rm -rf pure_pursuit
cd ../
colcon build --packages-select pure_pursuit
