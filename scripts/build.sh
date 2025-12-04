#!/usr/bin/env bash

# Usage: ./scripts/build.sh [DOCKER_CI_DIR]

set -e
set -o pipefail

DOCKER_CI_DIR=$1

$DOCKER_CI_DIR/scripts/build.sh sonia_common_ros2
$DOCKER_CI_DIR/scripts/build/BehaviorTree.CPP.sh

cd sonia_bt_runner

git submodule update --init --recursive

export behaviortree_cpp_DIR=/build/BehaviorTree.CPP/INSTALL_BASE/behaviortree_cpp/share/behaviortree_cpp/cmake
export sonia_common_ros2_DIR=/build/sonia_common_ros2/INSTALL_BASE/sonia_common_ros2/share/sonia_common_ros2/cmake/

colcon build --cmake-force-configure --install INSTALL_BASE
