#!/bin/bash

set -e

SCRIPT_DIR="$(cd $(dirname $0); pwd -P)"

if [[ -z "${BUILD_TYPE}" ]]; then
  BUILD_TYPE="Release"
fi
export BUILD_TYPE=$BUILD_TYPE

echo "BUILD_TYPE=${BUILD_TYPE}"

PROJECT_ROOT=$SCRIPT_DIR/..
BUILD_DIR=$PROJECT_ROOT/build
INSTALL_DIR=$PROJECT_ROOT/install

mkdir -p $BUILD_DIR

# Configure
cmake \
    -S $PROJECT_ROOT \
    -B $BUILD_DIR \
    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR

# Build
if [ -z "$1" ]; then
    cmake \
        --build $BUILD_DIR \
        --config $BUILD_TYPE
else
    cmake \
        --build $BUILD_DIR \
        --target "$1" \
        --config $BUILD_TYPE
fi

# Install
cmake \
    --install $BUILD_DIR \
    --config $BUILD_TYPE