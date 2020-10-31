#!/bin/bash

./hack_compile.sh

SEED=12345
for DYNAMIC_OBSTACLES in 0 1; do
  for TASK in gate bugtrap forest blob; do
    echo "STARTING $TASK $DYNAMIC_OBSTACLES point_first"
    ./headless_point_first.out $TASK $DYNAMIC_OBSTACLES $SEED >> final_results_all.txt
    echo "STARTING $TASK $DYNAMIC_OBSTACLES point_second"
    ./headless_point_second.out $TASK $DYNAMIC_OBSTACLES $SEED >> final_results_all.txt
    echo "STARTING $TASK $DYNAMIC_OBSTACLES stick_first"
    ./headless_stick_first.out $TASK $DYNAMIC_OBSTACLES $SEED >> final_results_all.txt
    echo "STARTING $TASK $DYNAMIC_OBSTACLES stick_second"
    ./headless_stick_second.out $TASK $DYNAMIC_OBSTACLES $SEED >> final_results_all.txt
  done
done

./analyze.sh
