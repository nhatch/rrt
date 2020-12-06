#!/bin/bash

echo Part I and III
./viz_point_first.out gate 0 0 0 naive
./viz_stick_first.out gate 0 0 0 naive
./viz_point_first.out forest 0 3 0 naive # More interesting than planning seed 0
./viz_stick_first.out forest 0 3 0 naive
sleep 3
./viz_point_first.out bugtrap 0 0 0 naive
./viz_stick_first.out bugtrap 0 0 0 naive
./viz_point_first.out blob 0 0 0 naive
./viz_stick_first.out blob 0 0 0 naive

echo Part II
sleep 3
# It's surprisingly hard to find a seed for which 50% extra planning iters
# actually finds a better plan.
./viz_point_first.out forest 0 3 0 naive

echo Part IV
sleep 3
./viz_point_first.out forest 0 3 0 mppi_full
./viz_point_second.out forest 0 3 0 mppi_full
./viz_stick_first.out forest 0 3 0 mppi_full
./viz_stick_second.out forest 0 3 0 mppi_full
./viz_stick_second.out forest 1 3 0 mppi_full # Dynamic obstacles

sleep 3

./viz_stick_first.out forest 0 3 0 mppi_min

./viz_stick_first.out forest 0 3 0 naive

echo Part V
sleep 3
./viz_stick_first.out gate 1 12545 1 mppi_min
./viz_stick_first.out gate 1 12545 1 mppi_full
./viz_stick_second.out forest 1 13445 12354 mppi_min
./viz_stick_second.out forest 1 13445 12354 mppi_full

sleep 3
# Gets unlucky by being surrounded by dynamic obstacles (I think? I forget if that was this one)
./viz_stick_first.out forest 1 13445 12354 mppi_full
# Gets driven off of entire planning tree
./viz_stick_second.out forest 1 14345 12347 mppi_full

# This one's a bug actually---MPPI can't fit through gap
# Will want to fix in future work
#./viz_stick_second.out forest 1 14295 12347 mppi_full
