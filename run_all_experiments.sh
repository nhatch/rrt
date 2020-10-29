#!/bin/bash

./hack_compile.sh

SEED=12345
for DYNAMIC_OBSTACLES in 0 1; do
  for TASK in gate bugtrap forest blob; do
    echo "STARTING $TASK $DYNAMIC_OBSTACLES point_first"
    ./headless_point_first.out $TASK $DYNAMIC_OBSTACLES $SEED >> final_results_pf.txt
    echo "STARTING $TASK $DYNAMIC_OBSTACLES point_second"
    ./headless_point_second.out $TASK $DYNAMIC_OBSTACLES $SEED >> final_results_ps.txt
    echo "STARTING $TASK $DYNAMIC_OBSTACLES stick_first"
    ./headless_stick_first.out $TASK $DYNAMIC_OBSTACLES $SEED >> final_results_sf.txt
    echo "STARTING $TASK $DYNAMIC_OBSTACLES stick_second"
    ./headless_stick_second.out $TASK $DYNAMIC_OBSTACLES $SEED >> final_results_ss.txt
  done
done

echo
echo "POINT ROBOT FIRST ORDER"
awk -f count.awk final_results_pf.txt
echo
echo "POINT ROBOT SECOND ORDER"
awk -f count.awk final_results_ps.txt
echo
echo "STICK ROBOT FIRST ORDER"
awk -f count.awk final_results_sf.txt
echo
echo "STICK ROBOT SECOND ORDER"
awk -f count.awk final_results_ss.txt
