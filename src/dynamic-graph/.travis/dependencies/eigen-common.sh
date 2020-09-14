#!/bin/bash
#
# Setup Eigen
#
. `dirname $0`/../common.sh

set -x

EIGEN_VERSION=$1
EIGEN_HASH=$2

# Checkout Eigen
cd "$build_dir"
wget "https://github.com/eigenteam/eigen-git-mirror/archive/${EIGEN_VERSION}.tar.gz"
tar xzf ${EIGEN_VERSION}.tar.gz
EIGEN_DIR="$build_dir/eigen-git-mirror-${EIGEN_VERSION}/"
mkdir -p "${EIGEN_DIR}/_build"
cd "${EIGEN_DIR}/_build"

# Build, make and install Eigen
cmake .. -DCMAKE_INSTALL_PREFIX:STRING="$install_dir"
make
make install

# Check install
pkg-config --modversion "eigen3 >= ${EIGEN_VERSION}"
pkg-config --cflags "eigen3 >= ${EIGEN_VERSION}"
