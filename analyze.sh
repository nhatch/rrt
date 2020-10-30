#!/bin/bash

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
