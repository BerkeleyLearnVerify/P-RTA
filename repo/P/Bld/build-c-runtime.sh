#!/usr/bin/env bash

# Build the C runtime!
mkdir -p build
cd build
cmake ../../Src
make


