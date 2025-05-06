#!/bin/bash
set -e  # Exit on error

# 🧹 Clean mode: `./build.sh clean`
if [[ "$1" == "clean" ]]; then
    echo -e "\033[1;33m[Build] Cleaning build directory...\033[0m"
    rm -rf build
    exit 0
fi

echo -e "\033[1;34m[Build] Creating build directory...\033[0m"
mkdir -p build
cd build

echo -e "\033[1;34m[Build] Running CMake...\033[0m"
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..

echo -e "\033[1;34m[Build] Building with $(nproc) cores...\033[0m"
make -j"$(nproc)"
