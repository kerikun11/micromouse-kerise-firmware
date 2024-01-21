#!/bin/sh

cppcheck --quiet --enable=all --suppressions-list=$(dirname $0)/suppressions.txt \
    --template='{file}:{line},{severity},{id},{message}' \
    --std=c++14 \
    --std=c11 \
    --platform=unix32 \
    -i build \
    --inline-suppr \
    lib/ctrl/include \
    lib/maze/include \
    lib/maze/src \
    src
