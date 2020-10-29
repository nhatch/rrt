#!/bin/bash
# Hackily and inefficiently makes four separate executables
# for various environment settings. Sorry.
sed -i "s/SECOND_ORDER = true/SECOND_ORDER = false/" control.h
sed -i "s/POINT_ROBOT = false/POINT_ROBOT = true/" config.h
make
mv headless.out headless_point_first.out
mv viz.out viz_point_first.out
sed -i "s/SECOND_ORDER = false/SECOND_ORDER = true/" control.h
make
mv headless.out headless_point_second.out
mv viz.out viz_point_second.out
sed -i "s/POINT_ROBOT = true/POINT_ROBOT = false/" config.h
make
mv headless.out headless_stick_second.out
mv viz.out viz_stick_second.out
sed -i "s/SECOND_ORDER = true/SECOND_ORDER = false/" control.h
make
mv headless.out headless_stick_first.out
mv viz.out viz_stick_first.out
sed -i "s/SECOND_ORDER = false/SECOND_ORDER = true/" control.h
