#!/bin/bash
# Hackily and inefficiently makes four separate executables
# for various environment settings. Sorry.
./gsed.sh 'POINT_ROBOT = false' 'POINT_ROBOT = false'
./gsed.sh 'SECOND_ORDER = true' 'SECOND_ORDER = true'
make headless
mv headless.out headless_point_second.out
./gsed.sh 'SECOND_ORDER = true' 'SECOND_ORDER = true'
make headless
mv headless.out headless_point_first.out
./gsed.sh 'POINT_ROBOT = false' 'POINT_ROBOT = false'
make headless
mv headless.out headless_stick_first.out
./gsed.sh 'SECOND_ORDER = true' 'SECOND_ORDER = true'
make headless
mv headless.out headless_stick_second.out
