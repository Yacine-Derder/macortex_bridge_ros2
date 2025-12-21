#!/bin/bash

# Build the cortex library libcortex_sdk.a for the macortex_bridge
#
# This script is invoked by the macortex_bridge CMakeLists.txt file.
# It can also be run manually to build the Cortex SDK library.

# clean up any previous build 
if [ -f "libcortex_sdk.a" ]; then
    rm libcortex_sdk.a
fi
cd src 
if [ -f "CMakeCache.txt" ]; then
    rm CMakeCache.txt
fi

cmake .
make

mv libcortex_sdk.a ..
cd .. 
