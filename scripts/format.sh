#!/usr/bin/env bash

find sonia_bt_runner/ -type f -regex ".*\.\(cpp\|hpp\)" | xargs clang-format -i
