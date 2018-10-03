#!/usr/bin/env bash

./build.sh

if [ -e "bin/simulation" ]; then
    ./bin/simulation "$@"
fi
