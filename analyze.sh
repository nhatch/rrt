#!/bin/bash

echo
cat final_results_all.txt | awk -f awk_scripts/static_setup.awk -f awk_scripts/count.awk -f awk_scripts/static_means.awk > means.txt
cat means.txt final_results_all.txt | awk -f awk_scripts/static_setup.awk -f awk_scripts/count.awk -f awk_scripts/static_latex.awk

echo
cat final_results_all.txt | awk -f awk_scripts/dynamic_setup.awk -f awk_scripts/count.awk -f awk_scripts/dynamic_means.awk > means.txt
cat means.txt final_results_all.txt | awk -f awk_scripts/dynamic_setup.awk -f awk_scripts/count.awk -f awk_scripts/dynamic_latex.awk
