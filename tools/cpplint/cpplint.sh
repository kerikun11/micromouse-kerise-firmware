#!/bin/sh

cpplint --quiet --recursive \
    --filter=-runtime/references,-runtime/explicit,-build/c++11,-readability/casting,-build/include_subdir \
    .
