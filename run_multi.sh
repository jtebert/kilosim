#!/usr/bin/env bash

START_TRIAL=1
NUM_TRIALS=10

# Run simulations
if [ -e "bin/simulation" ]; then
    for (( c=$START_TRIAL; c<$NUM_TRIALS+$START_TRIAL; c++ ))
    do
        echo -n "$c "
        ./bin/simulation "$@" --trial "$c"
    done
else
    echo "Error: Did not find compiled ./bin/simulation"
fi